"""GUI 链路诊断与交互路线图导出的离线测试，不联网也不启动浏览器。"""
import time
import tkinter as tk
from unittest.mock import patch

from funsport import gui_services
from funsport.gui import App
from funsport.gui_services import run_parameters
from funsport.run_plan import RunPlan
from tests.plan_fixture import fake_client, sample_plan
from tests.support import IsolatedCase


def chain_plan(**route):
    """在示例方案上补一份 route 链路字段，返回新的冻结方案。"""
    plan = sample_plan()
    data = plan.data()
    link = {"provider": "synthetic_test_fixture", "ring_length_m": 1000,
            "visit_all_points_m": 1672.0, "complete_laps": 1,
            "partial_lap_distance_m": 328.0, "returns_to_start": True}
    link.update(route)
    data["route"] = link
    return RunPlan.create(fake_client(), data)


class GuiRouteExportTests(IsolatedCase):
    """验证方案面板的链路文案和导出按钮，不访问网络或浏览器。"""

    def setUp(self):
        """建立隔离窗口并将系统消息框替换为测试替身。"""
        super().setUp()
        self.root = tk.Tk()
        self.root.withdraw()
        self.map_patch = patch("funsport.gui.tkintermapview.TkinterMapView")
        self.map_patch.start()
        self.addCleanup(self.map_patch.stop)
        self.icon_patch = patch.object(App, "_icon_cache", {})
        self.icon_patch.start()
        self.addCleanup(self.icon_patch.stop)
        self.app = App(self.root)
        self.addCleanup(self.cleanup_window)
        self.error_patch = patch("funsport.gui.messagebox.showerror")
        self.errors = self.error_patch.start()
        self.addCleanup(self.error_patch.stop)

    def cleanup_window(self):
        """测试后终止本测试自己的 Tk 窗口并恢复日志处理器。"""
        if getattr(self.app, "worker", None):
            self.app.worker.join(timeout=5)
        self.app.busy = False
        self.app.close()

    def pump(self):
        """推进 Tk 事件循环直到后台任务和队列都结束。"""
        deadline = time.monotonic() + 5
        while self.app.busy or not self.app.events.empty():
            self.root.update()
            if time.monotonic() > deadline:
                self.fail("GUI task timed out")
            time.sleep(0.01)

    def preview_parameters(self):
        """直接注入预览时也包含生成按钮读取的诊断开关。"""
        params = run_parameters(self.app.values(self.app.run_vars))
        params["diagnostic_capture"] = bool(self.app.settings_vars["diagnostic_capture"].get())
        return params

    def test_chain_status_shows_laps_partial_and_return(self):
        """链路文案给出走完点位、整圈数、尾部里程和返程状态。"""
        self.app.show_plan(chain_plan(), self.preview_parameters())
        text = self.app.chain_status.get()
        self.assertIn("链路：", text)
        self.assertIn("走完点位 1672 m", text)
        self.assertIn("已走 1 圈", text)
        self.assertIn("尾部 328 m", text)
        self.assertIn("已走返程", text)

    def test_chain_status_reports_zero_laps(self):
        """不足一圈时显示 0 圈和未走返程，不把环长当目标。"""
        self.app.show_plan(chain_plan(complete_laps=0, returns_to_start=False,
                                      partial_lap_distance_m=120.0,
                                      visit_all_points_m=900.0,
                                      ring_length_m=1200.0),
                           self.preview_parameters())
        text = self.app.chain_status.get()
        self.assertIn("已走 0 圈", text)
        self.assertIn("尾部 120 m", text)
        self.assertIn("未走返程", text)

    def test_legacy_plan_shows_missing_chain(self):
        """旧方案没有链路字段或 route 为空时显示未提供。"""
        self.app.show_plan(sample_plan(), self.preview_parameters())
        self.assertIn("未提供", self.app.chain_status.get())
        self.assertEqual(App.chain_text(None), "链路：未提供（旧方案）")
        self.assertEqual(App.chain_text([1, 2]), "链路：未提供（旧方案）")

    def test_invalidated_plan_resets_chain_status(self):
        """参数变化使方案失效后，链路文案回到尚未生成。"""
        self.app.show_plan(chain_plan(), self.preview_parameters())
        self.app.run_vars["seed"].set("99")
        self.assertIn("尚未生成方案", self.app.chain_status.get())

    def test_export_button_requires_plan(self):
        """没有方案时导出按钮禁用，参数变化后重新禁用。"""
        self.assertTrue(self.app.export_button.instate(["disabled"]))
        self.app.show_plan(chain_plan(), self.preview_parameters())
        self.assertFalse(self.app.export_button.instate(["disabled"]))
        self.app.run_vars["seed"].set("99")
        self.assertTrue(self.app.export_button.instate(["disabled"]))

    def test_export_without_plan_gives_hint(self):
        """直接触发导出时只提示先生成方案，不调用服务。"""
        with patch("funsport.gui.services.export_route_overlay") as export:
            self.app.export_route_overlay()
            export.assert_not_called()
        self.assertEqual(self.errors.call_count, 1)
        self.assertIn("先生成方案", self.errors.call_args.args[1])

    def test_export_opens_generated_html(self):
        """点击导出后后台生成并交给浏览器打开 HTML，状态栏给出路径。"""
        plan = chain_plan()
        self.app.show_plan(plan, self.preview_parameters())
        json_path = self.data / "gui_demo.json"
        html_path = self.data / "gui_demo.html"
        with patch("funsport.gui.webbrowser.open") as open_browser, \
                patch("funsport.gui.services.export_route_overlay",
                      return_value=(json_path, html_path)) as export:
            self.app.export_button.invoke()
            self.pump()
        export.assert_called_once()
        self.assertEqual(export.call_args.args[0], plan.data()["track"])
        self.assertEqual(export.call_args.args[1], plan.plan_id)
        open_browser.assert_called_once()
        uri = open_browser.call_args.args[0]
        self.assertTrue(uri.startswith("file:"), uri)
        self.assertTrue(uri.endswith(".html"), uri)
        self.assertIn(str(html_path), self.app.status.get())

    def test_export_failure_shows_error(self):
        """导出失败时不打开浏览器，改用错误框提示并恢复按钮。"""
        self.app.show_plan(chain_plan(), self.preview_parameters())
        with patch("funsport.gui.webbrowser.open") as open_browser, \
                patch("funsport.gui.services.export_route_overlay",
                      side_effect=RuntimeError("磁盘已满")):
            self.app.export_button.invoke()
            self.pump()
        open_browser.assert_not_called()
        self.assertIn("磁盘已满", self.errors.call_args.args[1])
        self.assertFalse(self.app.export_button.instate(["disabled"]))


class RouteOverlayServiceTests(IsolatedCase):
    """验证导出服务的入参校验与文件名，不写真文件也不生成 HTML。"""

    def test_requires_a_track(self):
        """空轨迹直接拒绝，避免生成空图表。"""
        with self.assertRaises(ValueError):
            gui_services.export_route_overlay(None, "abc")
        with self.assertRaises(ValueError):
            gui_services.export_route_overlay({"locations": [{}]}, "abc")

    def test_writes_named_paths_without_html(self):
        """默认目录在仓库 artifacts/humanized，文件名清洗方案 ID。"""
        track = {"locations": [{}, {}]}
        with patch("funsport.api.chain.overlay_document_from_track",
                   return_value={"name": "doc"}) as overlay, \
                patch("funsport.api.chain.write_document",
                      side_effect=lambda doc, j, h=None, title=None: (j, h)) as write:
            json_path, html_path = gui_services.export_route_overlay(
                track, "a/b c", out_dir=self.data, want_html=False)
            self.assertEqual(json_path, self.data / "gui_a_b_c.json")
            self.assertIsNone(html_path)
            overlay.assert_called_once_with(track, name="a/b c")
            write.assert_called_once()
            default_json, default_html = gui_services.export_route_overlay(track, "abc")
        self.assertEqual(default_json.name, "gui_abc.json")
        self.assertEqual(default_json.parent.name, "humanized")
        self.assertEqual(default_json.parent.parent.name, "artifacts")
        self.assertEqual(default_html.name, "gui_abc.html")
