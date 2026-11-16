# Spot and pilot preparation

`DrillingSequence(drill, spot=spot, pilot=pilot, spot_diameter_mm=2.0).operations()`
returns ordinary `Drilling` operations in spot, pilot, final order. Either preparation
stage can be omitted. Supply actual tools, point angles, cutting parameters, hole
centres and depths; the helper does not infer a recipe. Wrap each result in an
`OperationNode` in the same setup. Existing jobs without preparation are unchanged.

The native `preparation` descriptor names preceding drilling operation IDs (`spot`
and/or `pilot`) and requires `spot_diameter_mm` exactly when a spot is named. Each
prerequisite must cover every final hole occurrence in the same work frame. The
spot must precede the pilot. Standalone planning refuses a descriptor because it
cannot establish the preceding operations. Stream regeneration, simulation and
checked posting validate the complete source-bound run.

The report computes nominal geometry from the declared conical cutters and stock
top. The requested spot diameter must match its depth while the cut remains on the
point. The spot angle must be at least every following drill's point angle, and the
spot cannot pass the next drill's tip target. A larger spot reports both radial
chamfer width and axial depth. These are nominal dimensions, not a measured finish.
See the [ISCAR Drilling Handbook, printed p.186](https://www.iscar.com/Catalogs/Publication/english_1/Drilling_Handbook/Drilling_Handbook_flipview/files/basic-html/page188.html).

The supported pilot is a **smaller pre-drill**, with full diameter below the stock
top and its entire cone inside the final drill's depth envelope. Same-diameter
deep-hole guide systems need tool-specific entry, speed and clearance contracts;
they are refused here. `depth_reference="full_diameter"` sets the pilot shoulder
target explicitly. A report includes its actual full-diameter depth and tip Z.

Preparation grants no rapid-entry clearance, feed increase or safety exemption.
Ordinary stock, scene, material, assembly and independently decoded posted-output
checks retain authority. Missing declarations remain unchecked or refused under
the existing release policy. Cancellation and bounded hole coverage apply before
motion publication; cached reports are bound to current prerequisite geometry.

Tests compare actual motion and stock with the same explicit stages without the
descriptor, exercise decoded explicit and canned posting, and reject altered
order, coverage, point angle, depth and stale cached reports. The Release benchmark
contains matched `drilling-preparation-manual` and `drilling-preparation-bound`
controls to measure the cost of checking the declaration.
