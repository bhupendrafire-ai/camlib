# Bounded native preparation

`camlib.preparation` exposes native reach screening and explicit top-down blank
premises. These queries never release NC. The caller supplies calibrated tools,
physical source/frame/error declarations, fixtures, work and tolerance budgets.
Ordinary job generation, independent posted replay and the machine export gate
remain required.

## Reach and supplied projections

`ReachSample(id, point_mm, normal)` describes a caller-supplied surface sample.
`reach_map` takes the source surface, samples and finite calibrated tool variants,
plus a scene, base/clearance heights, tolerance and pair/work limits. Each tested
pair is clear, obstructed or unresolved and names the affected assembly component.
Curved cutters use conservative enclosing sections; sample contact estimates do
not establish clearance for a complete operation.

Projection selection chooses only the smallest clear **tested** calibrated variant
in each supplied family. It does not infer an exact continuous minimum or invent
an assembly. Cancellation or exhausted work returns no selection. Source
preparation and allocation are charged before doing their work.

This is not the full operation-local reach contract requested in #129. It does
not change a pocket's allowed top or infer an obstruction-height recommendation.

## Declared initial stock

`InitialStock` binds source ID, revision, SHA-256, source-to-setup frame,
uncertainty and one lower/upper pair for each complete XY grid footprint.
Its representation is `whole_cell_top_down`; it cannot model overhang cavities.
`Setup.initial_stock` and `Simulation(initial_stock=...)` use those same bounds.
Native streaming, receiving, cache identity, saved/fractional playback and
independently posted replay preserve the declaration and its volume origin.
A restored checkpoint does not acquire fresh-history proof.

`enclose_blank` requires an explicit `top_down_outer` source premise, an already
placed mesh, source/error identity, stock grid and finite work/tolerance limits.
It encloses whole grid footprints and refuses material outside the stock domain
rather than clipping it. Unsupported source premises are refused.

`blank_placements` proposes finite upright AABB alignments and reports ambiguity.
`confirm_blank` records an explicit engineer choice; `confirmed_initial_stock`
recomputes and validates the saved proposal before use. AABB fit is advice, not
physical registration. CAD import, measurement and confirmation belong to the
consumer. Welding needs a new complete measured source, not a material-adding
replay delta. A blank missing required original part material cannot be released.
This does not complete arbitrary mesh stock/export (#121) or resumed-history
material proof (#108).

## Material limits

Scoped asymmetric material tolerances retain separate gouge and residual limits.
Source uncertainty and actual arc/end-point gaps are spent by the continuous
proof rather than hidden by endpoint sampling. Conservative or incomplete
results stay uncertain. Ball/bull material proof retains the existing enclosing
flat end; real-shape material proof remains separate work in #107.
