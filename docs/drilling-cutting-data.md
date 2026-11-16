# Declared drilling tool and cutting-data recommendations

`recommend_drilling` is an opt-in Python helper. Supply the available
`ToolAssembly` objects, `DrillingCuttingData` records and a declared `Machine`.
CAMLib supplies no rate table or fallback recipe. Each record identifies one tool,
its nominal diameter, exact material identity, source/revision/page, process
conditions, allowed Vc range (m/min), feed per revolution (mm/rev) and maximum
**tip** depth/diameter ratio. Test records are fictional qualification inputs.

```python
from dataclasses import replace
from camlib import recommend_drilling

# available, records, material_id, diameter, tip_depth, tolerance, machine and
# drilling are explicitly declared by the caller; no shop rates are assumed.
proposal = recommend_drilling(
    available, records, material=material_id, diameter_mm=diameter,
    tip_depth_mm=tip_depth, diameter_tolerance_mm=tolerance, machine=machine,
)
if not proposal.candidates:
    raise ValueError(proposal.rejected)
selected = proposal.candidates[0]
drilling = replace(drilling, tool=selected.tool,
                   spindle_rpm=selected.cutting.spindle_rpm,
                   feed_mm_min=selected.cutting.feed_mm_min)
# Put the operation through the ordinary Job, simulation and checked post path.
```

A record requires nonempty `id`, `tool`, `material`, `source`, `conditions`, plus
positive finite `diameter_mm`, `vc_min_m_min`, `vc_max_m_min`, `feed_per_rev_mm` and
`max_depth_diameter_ratio`. The tool must be in the supplied inventory and machine
table with matching gauge length. Its actual declared conical drill point must
fit the known flutes; requested tip depth must fit both flutes and the supplied
depth ratio. Callers must include actual point depth and through-hole breakout.
Material and record/tool diameter match exactly; the requested diameter uses the
caller's explicit tolerance. Spotting and pilot use require records whose stated
conditions cover those processes and the actual tool system.

The nominal formulas are `n = 1000 × Vc / (π × D)` and `vf = fn × n`. **Fn is
mm/revolution**; it is not multiplied by flute count. The helper uses the supplied
maximum Vc, then caps rpm by the machine spindle, scalar feed and optional Z-axis
feed limits while retaining nominal fn. If this would put Vc below the supplied
minimum, the candidate is rejected. See [Sandvik Coromant, metric drilling formulas,
printed H81](https://cdn.sandvik.coromant.com/files/sitecollectiondocuments/services/metal-cutting-e-learning/formulas-and-definitions/formulas-and-deinitions-for-drilling-metric-enu.pdf).

Candidates rank by shortest adequate flutes, then greatest feasible nominal feed,
then tool/record IDs. Rejection reasons are returned separately. Input order does
not change ranking. The input/tool/record budget defaults to 10,000 and can be
explicitly set up to 100,000. A zero-argument `cancel` predicate aborts without a
partial proposal. Duplicate IDs, malformed declarations and exceeded budgets
refuse the request. Inputs and operations are never mutated.

Review and retain the selected record's source and required conditions. These
proposals do not establish coolant, chip evacuation, tool-system compatibility,
cutting performance or physical calibration. No coolant command, peck schedule,
deep-hole guide recipe or operation approval is inferred. Ordinary stock, scene,
material, tool assembly, machine and independently decoded posted-output checks
retain authority; missing declarations keep their existing refusal behavior.
Posting resolution and ordinary scheduling still apply to nominal rates.

The native stock/post regressions and installed-package tests use the same public
helper. Matched Release workloads `drilling-data-manual` and
`drilling-data-recommended` compare identical synthetic rates with full scene/stock
checks; they make no real machining or engine speedup claim.
