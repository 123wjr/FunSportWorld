# Road realism comparison snapshot

This branch preserves the broader in-progress generator implementation relative to base commit `41359e5`. It includes latitude-aware geometry, revised sampling and kinematics, sampled-GCJ distance planning, optional joint/lateral controls, route-overlap scoring, postfix changes, synthetic tests, and the experimental corner-rounding helper. Default generated output is not seed-compatible with the base version.

The production flow does not yet enable the optional joint/lateral controls. Corner rounding remains a research helper, not a production feature. Its `radius_m` argument is a tangent cut length rather than a guaranteed curvature radius. This snapshot is for comparison, not a claim of completed production integration or validated realism improvement.

Private activities, precise fixed-start research configuration, route caches, credentials, generated artifacts, temporary analysis scripts, unrelated packaging scaffolding, and historical research harnesses remain local and are excluded from this commit. No network or platform action is required to run the included synthetic tests.

Validation before snapshot: 184 offline regression tests passed across generator geometry, joint controls, sampling, route metrics/selection, rounding, extension and road-first runner suites. Some of those suites belong to the local research harness and are not included in this comparison commit.
