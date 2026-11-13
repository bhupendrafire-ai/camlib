# Milling post validation (#20)

The fixed synthetic fixture extends `examples/standalone_job.py`:

```sh
PYTHONPATH=python python examples/standalone_job.py out-validation --validation --accept-unchecked-machine
```

It faces a 32 × 24 × 6 mm block to Z5, pockets to Z3 around a circular
island centred at (16, 12) with radius 2 mm, then uses a 2 mm tool to remove
stock left by the 4 mm tool. Tool numbers are T7/T8, length registers H9/H10;
both assemblies have a declared 60 mm gauge length. Feed is 600 mm/min,
plunge 150 mm/min, spindle 12000 rpm, flood coolant; clearance is Z8.
These are reproducible test inputs, not recommended cutting data.

The machine declaration is `rest_machine_for(job)`: a synthetic LinuxCNC
mill with millimetre output at three decimals, XY/XZ/YZ plane capability,
arcs and helices, and an explicitly declared tool change that preserves
the compensated tip position. `LoweringOptions(native_arcs=True)` keeps
writable arcs and converts the others to checked lines. The fixture uses
XY arcs; XZ/YZ cutting arcs, wear compensation, drilling and tapping need
their own cases. It does not claim to complete #20.

The ordinary streamed stock checks and independent decoded-output replay
run before export. Without `--accept-unchecked-machine`, the example refuses
to write NC because no physical machine crash model has been declared.
The option accepts only that missing model for inspection; it does not
accept a stock-contact finding. Preserve the JSON check results with the NC.

The LinuxCNC interpreter harness in PR #287 can consume this same fixture:
build it with `build_validation_job()`, use `rest_machine_for(job)`, and
pass `lowering=LoweringOptions(native_arcs=True)` to the harness's existing
`run_job`. Interpreter execution is a separate check and needs `rs274`.

## A result record

For each controller, record the tested commit, complete machine declaration,
tool and offset registers, controller model/software, post configuration,
stock/material, actual tools and holder geometry, workholding and work zero.
Keep the generated NC and its hash, JSON checks, interpreter result and any
alarm text. Mark an unrun check as **not run**.

Before an operator runs the job, replace the synthetic machine and cutting
declarations with the actual setup and complete the ordinary export checks.
Record air-cut and wax/foam runs separately, including tool changes, coolant,
arc direction, the island diameter (4 mm), pocket floor (Z3) and faced top
(Z5), with measured values and measurement uncertainty. Any tolerance for
physical acceptance must be declared for that setup; this fixture supplies
none. Software, interpreter, air-cut and material-cut results remain separate.
