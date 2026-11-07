# Road realism work handover

## Snapshot

- Comparison branch: research/road-realism-comparison
- Snapshot commit: 84ea86c858e6f8a50123ab78cff7474d243635b4 (base 41359e5).
- The commit contains broad generator, geometry, route-scoring, optional joint-control and sampling changes; selected synthetic tests; and tools/road_smoothing.py with tests. Scope note: docs/road-realism-comparison.md.
- Git author identity is configured locally for this repository only: John Jerry / jerry.only@qq.com.
- No platform action, real account submission, or network call was made for this commit.

## Validation

The snapshot was checked with 184 offline tests covering route geometry, joint controls, sampling, route metrics and selection, smoothing, road extension, and the road-first runner. A separate joint-fit suite passed 13 tests. The 197 total is the sum of those two runs; the overlapping sets were not run as one unified command. Staged files had no material whitespace errors; warnings seen during staging were in paths excluded from the commit.

## Integration limits

This is a comparison snapshot, not a production-ready integration or validated realism improvement.

- Optional lateral/joint controls exist in the generator, but the production flow does not pass them.
- Corner smoothing is a research helper and is not wired into production. Its radius_m parameter is a tangent cut distance, not a guaranteed realized curvature radius; smoothing can shorten the route.
- The broad branch changes default output independently of opt-in controls. Do not assume fixed-seed compatibility with the base version.
- The open sampled-GCJ path is non-wrapping and conflicts with the existing production ring/multi-lap flow. Wiring it requires an explicit behavior decision and integration tests.
- The lateral bound is a modeling bound, not a measured road-width/corridor guarantee. There is no evidence that the generated path remains on the physical carriageway.
- Prior AMap work did not complete a successful >2 km open-road build; this branch does not establish real-road generation or independent activity-level validation.

## Next steps

1. Compare this branch with the base/minimal branch using identical synthetic routes and seeds; inspect generated length, speed/cadence/stride, and API behavior.
2. Decide whether to preserve closed-loop/multi-lap semantics or move production to one open AMap pass. Do not wire the current open-path control into production before resolving this.
3. Keep corner smoothing, radius, and offset amplitude experimental. Measure realized curvature and path-length change; test seam, self-proximity, checkpoint proximity, and combined offset-plus-rounding bounds.
4. Keep the base branch intact and review diffs explicitly; this snapshot includes changes beyond the two requested realism concepts.
5. Keep private activity data, route files, keys, caches, and generated artifacts outside Git. Account submission is not part of this handover.

## Workspace state

The comparison snapshot and this handover are the intended commits on research/road-realism-comparison. The worktree still has untracked local research scripts, data, artifacts, evaluation tools, tests, and docs; these were intentionally excluded. Do not use git add . The sibling branch research/road-realism-minimal remains separate and uncommitted.
