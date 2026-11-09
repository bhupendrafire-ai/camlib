# Checked recipe planning

`camlib.planning` compares explicitly supplied three-axis milling recipes using
ordinary jobs, native toolpaths, actual decoded stock replay and the existing
machine/release checks. It produces advice, never an NC release.

The caller still supplies stock, geometry, fixtures, tool assemblies and their
register/calibration settings, cutting data, machine/controller declarations,
verification targets and completion limits. No shop presets are inferred.

## Comparing recipes

Given a caller-defined `job`, declared `machine` and two complete recipes:

```python
from dataclasses import replace
from camlib.planning import (
    PlanningCandidate, ComparisonPolicy, OperationChoices,
    recipe_candidates, compare_candidates,
)

reference = PlanningCandidate("reference", job, "top", machine)
setup = next(s for s in job.setups if s.id == "top")
operation = setup.operations[0]  # unwrap OperationNode.operation when needed

# Both recipes' cutting data must be supplied/approved for their tools.
choices = (OperationChoices(operation.id, (
    operation,
    replace(operation, feed_mm_min=approved_alternative_feed),
)),)
candidates = recipe_candidates(reference, choices, max_candidates=declared_candidate_limit)
policy = ComparisonPolicy(
    residual_limits_mm3=(("finished-part", permitted_residual_upper_mm3),),
    max_seconds=declared_search_budget_seconds,
)
report = compare_candidates(candidates, reference="reference", policy=policy)
```

`OperationChoices` enumerates the combinations of supplied operation recipes,
deduplicates identical recipes and retains the original. A required positive
`max_candidates` refuses an oversized Cartesian product before enumeration;
it never silently truncates the search. An alternative can
use another assembly or projection variant already in the setup and machine
tool pools. Its operation must declare the corresponding number/register and
its own cutting data. Operation-node dependencies, sources and limits survive.

Every candidate must share the same stock, source, tool pool, machine,
calibration, frame and verification premises. Setup/target changes are separate
problems, not competing recipes for the same comparison. Inputs are compiled
into immutable snapshots before evaluation. Existing regeneration caches can
be supplied; changing a verification premise still invokes fresh checks.

Comparison requires all scene, stock-contact and material checks, distinct final
material targets with explicit residual-volume upper limits, and a physical
machine declaration. It exposes no `accept` argument. Roughing can deliberately
leave stock through its caller-declared completion limit; the release gate's
gouge rules are never loosened to meet that limit.

`Machine.motion_policy` and `schedule_post_feeds=True` must be explicit. The
current comparison uses `junction_mode="exact_stop"`. The metric is native
**decoded nominal motion seconds**, excluding spindle/tool-change/operator
overhead, jerk and real controller blending. It is not shop-floor cycle time.

Each result records its full input digest, eligibility/refusal reasons, motion
time, actual tool assignments, tested stick-out/gauge lengths, posted blocks,
tool changes and residual bounds. `selected` minimizes motion time, then tool
changes; the reference wins exact ties. `best_tradeoffs` retains candidates
that are not beaten on both metrics. No screening estimate grants eligibility.

Results use `eligible`, `rejected` and `incomplete`. Time/count metrics can be
`None`; display them as unavailable, never zero. Skipped candidates can have no
assembly record. `residual_mm3` contains `(target_id, lower, upper)` triples;
`assemblies` contains `(tool_id, stickout_mm, gauge_length_mm)` triples. Reports
contain the input digest, reference ID, evaluations, optional selected ID,
trade-off IDs and `complete`. An incomplete report has no selected result; its
retained trade-offs are diagnostics, not adoption/release permission. Invalid
preflight inputs can raise before any report exists. Finite collections mean
lists or tuples; no candidate count is silently truncated.
The caller supplies a cooperative time budget and may supply a cancellation
token. Interrupted searches retain diagnostics and select nothing. Preparation checks cancellation and deadline before and after each snapshot.
An interrupted preparation returns one incomplete row per candidate, with no
metrics or assemblies. Those rows use explicitly request-stage digests; the
report digest is tagged `preparation_interrupted`, rather than claiming a
compiled-candidate identity. No evaluation starts until all common premises
match. Individual native operations observe cancellation cooperatively.

Use the existing job export APIs separately after choosing a recipe. They
recheck the exact program to be released. A comparison report grants no release
permission and is not a physical setup certificate.

## Local residual correction

`correct_residuals(candidate, templates, policy=..., max_passes=...)` performs a
bounded feedback loop. Each `CorrectionTemplate(target_id, operation)` supplies
an explicit native rest-finishing or `RestPocket` operation with its tool and
cutting data. The tool must already be in the declared pool.

After release checks pass, residual findings identify whole-cell XY regions.
The union of residual XY boxes is expanded by one cutter diameter: one radius
for cutter-centre reach and one for whole-footprint containment. It is then
intersected with setup/template containment to build the correction limit;
existing setup/template containment and avoid areas remain in force. The
operation is appended with an explicit dependency on the preceding operation.
The entire revised program is then generated, posted and independently checked
again. Existing targets, tolerance, source error and tool calibration do not
change. `correction_candidate` exposes proposal construction separately.

A collision/gouge or uncertain safety result cannot be repaired by this loop.
Missing/truncated residual locations stop automatic correction. No-progress,
increased residual upper bounds, cancellation and pass/time budgets stop with
diagnostics. `max_passes` is a nonnegative integer; zero performs only the
initial evaluation. The loop performs that evaluation plus at most `max_passes`
corrections. Any target's increased upper residual bound stops the loop; an
unchanged complete tuple of upper bounds is no progress. A returned candidate satisfies the declared completion limits;
otherwise the result's candidate is `None`. Supported native-operation and
material-proof limits still apply, including unresolved real-shape finishing
checks. The loop does not claim arbitrary surface reach or invent new paths.

## Declared engagement/load feed policies

`schedule_engagement(program, stock, policy=EngagementFeedPolicy(...),
initial_stock=..., sweep=...)` returns a new typed program and per-move report.
Thresholds strictly increase within 0–360 degrees; caps are positive, finite and
nonincreasing (equal adjacent caps are valid). The report binds source, policy,
stock, exact motion and committed stock generation.

Legacy empty `tool_limits` retains horizontal flat-cutter line/XY-arc slowdown
behavior and explicit `unresolved="refuse"` or `"preserve"`. Supplying
`ToolLoadLimit(tool,max_axial_mm,max_radial_mm,max_cut_feed_mm_min)` enables
conservative whole-curve load bounds for flat/ball/bull/drill/chamfer cuts,
including sloped lines, arcs, helices and unsynchronized plunges. Empty-limit
legacy plunges retain authored feeds. Radial bound is full diameter; axial
bound uses upper heights of all cells intersecting the enclosing curve box,
limited to the cutting length. These are upper bounds, not force/torque estimates.
Missing/exceeded declared limits refuse or preserve the authored move explicitly;
a smaller feed alone cannot resolve a depth/width limit. Noncutting-body clearance
still requires stock-contact checks. Taps and covered cycles retain authored feeds.

An optional `air_feed_mm_min` may increase a cut's feed only with an analytic
whole-sweep separation certificate from the ORIGINAL stock envelope. The air cap
and any declared tool-feed cap both apply. Zero sampled removal cannot authorize
an increase. Near-stock moves stay capped by authored feeds and angular/tool
limits. Unavailable bounds remain null, never zero. Turning and beam programs
are refused. No chip-thinning or universal cutting data is inferred.

Run the result through `Machine.schedule` and ordinary checked `PostedRun`
posting/replay with identical stock, initial stock and sweep declarations.
Reports grant no NC release and do not silently alter `Job.post` recipes.

## Host preparation and initial stock

`InitialStock` supplies one lower/upper height pair for every whole XY grid
cell, in row-major order. It describes a top-down solid column from the stock
base to a bounded height. Its source ID/revision/SHA-256, `source_to_setup`
(`WorkFrame`) and nonnegative `uncertainty_mm` are required. Bounds are already
in setup coordinates and must enclose source and placement uncertainty over
the **entire cell**, rather than sample only its centre. Metadata records the
placement; the low-level declaration does not transform or measure the source. The
preparation helpers below explicitly construct and validate placed proposals.
Overhangs/enclosed voids cannot be represented by this contract.

For each axis the grid count is `ceil(extent / cell_size_mm)` and actual cell
width is `extent / count`. Cell `y*nx+x` is X-fastest. Bounds cover those actual
footprints. Python `bounds_mm` becomes wire `bounds` with lower/upper pairs;
the representation is exactly `whole_cell_top_down`. Source IDs use 1–128 ASCII
letters, digits or `_`, `-`, `.`, `:`; revisions have 1–512 bytes and digests
exactly 64 hexadecimal characters. Uncertainty is finite within 0–1,000,000 mm.
There is one finite pair per cell, with
`stock.min_z <= lower <= upper <= stock.max_z`. Placement frames are orthonormal
and right-handed. Existing stock-grid validity and cell budgets also apply.

Pass the same declaration to `Setup(initial_stock=...)`, `Simulation` or
`RecordedRun`. Native generation, posted replay, saved playback, fractional
preview, initial volume and regeneration identity use that declaration. A
posted compiled setup refuses a different blank override. Old full-box runs
omit the new manifest member and retain their existing wire representation.
Rust callers constructing `StreamManifest` literals add `initial_stock: None`
for those full-box runs.
An arbitrary restored heightfield remains unproven; an initial declaration
does not upgrade later restored snapshots into machining-history proof.
Physical-machine envelope checks may still conservatively use the stock box.

`examples/host_preparation.py` is a dependency-free host integration example:

- Resolve sRGB face colours against a caller-supplied versioned chart using
  CIEDE2000 and explicit match/ambiguity thresholds. Missing, unknown and
  overlapping meanings remain unresolved.
- Preserve per-face lower/upper tolerance, allowance and keep-out declarations;
  attach chart/source/converted-region provenance through `OperationNode.inputs`.
  `intent_targets` translates supported horizontal `FaceRegion` values into
  actual operation floors, containment, keep-outs and per-face native material
  checks. `planar_cad_faces` reuses the bounded CadQuery floor exporter; other
  geometry is explicitly refused. Separate residual/gouge tolerances retain
  asymmetry; finite signed bands must satisfy −1<=lower<=0<=upper<=1 mm.
  A distinct full-part final check remains required.
- Select only an explicitly confirmed proposal from the host's measured blank
  placements. CAD import, registration and engineer confirmation stay there.
  A welded part requires a new complete measured/enclosed blank and source
  revision, never a material-adding replay delta or guessed design allowance.

## Native preparation APIs

`camlib.preparation.reach_map` checks supplied source samples against calibrated
flat/ball/bull assemblies and vertical fixture approaches. Conservative staircase
sections enclose curved cutter profiles; planner contact estimates do not prove
clearance. Each pair reports clear/obstructed/unresolved with component reasons.
Projection proposals select the smallest clear **tested** caller-calibrated
variant per family, never a continuous exact minimum. Incomplete maps select
nothing. Selected tools still require complete recipe, stock and machine checks.

`enclose_blank` bounds every actual grid footprint with native facet/rectangle
queries. It requires an explicit `top_down_outer` source premise, already placed
mesh, source/error identity and work/tolerance limits. Out-of-domain material is
refused rather than clipped. `blank_placements` proposes finite upright AABB
alignments, reports fit ambiguity and never chooses a physical placement.
`confirm_blank` records explicit engineer selection;
`confirmed_initial_stock` recomputes and validates the saved proposal before use.
AABB fit is advice, not physical registration authority. A new weld requires a
new complete measured source and confirmation. An uncertain initial blank cannot
supply whole-history proof that the target part existed before machining.

## Limits and related work

Material proof keeps the existing enclosing flat end for ball/bull cutters;
real-shape material proof remains #107. A conservative uncertain result stops
comparison and correction, and cannot be accepted by the planning API. Native
reach queries use conservative curved-cutter enclosures for their own supplied
samples; a reach result does not establish per-move material or NC clearance.

The host example is a standalone API example. It does not add a CAD parser,
viewer, server, database, application-specific integration or shop presets.
