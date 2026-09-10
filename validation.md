# Validation Rules

## Required inputs

- `model_path` and `velocities` are required engineering inputs; the Agent must not guess or invent them.
- Missing model path: if the task context contains exactly one clear UUV CAD model, name it and ask the user to confirm; with multiple candidates, ask the user which one to use.
- Missing velocities: ask the user which velocities to compute; never self-assign default speeds.

## Pre-execution existence checks

Before the CFD process is launched, verify that all three of the following actually exist on disk:

1. The CAD model file at the user-specified path.
2. The solver script `D:\pyfluent\run_uuv_cfd.py`.
3. The designated Python interpreter `D:\pyfluent\pyfluent_v241\Scripts\python.exe`.

Do not silently substitute a different model, script, or interpreter when the user gave an explicit path.

## CAD model

Supported extensions:

`.x_t`, `.x_b`, `.step`, `.stp`, `.iges`, `.igs`, `.sat`, `.sab`, `.scdoc`

## Python interpreter and console encoding

- The designated interpreter is `D:\pyfluent\pyfluent_v241\Scripts\python.exe`. The system `python`, the `py` launcher, and any other Python environment are forbidden.
- On Windows, Python stdout must be set to UTF-8 before launch (`$env:PYTHONIOENCODING="utf-8"` in PowerShell; see tool-interface.md). A GBK/CP936 console crashes the driver with `UnicodeEncodeError` on progress markers such as `▶`.

## Velocity

Each requested velocity must satisfy:

`velocity > 0` and finite.

Preserve the requested engineering units. The current tool interface expects m/s.

## Processor count

`cores` must be a positive integer. Default: `6`.

Do not automatically consume all workstation cores unless the user explicitly asks for it.

## Iterations

`iterations` must be a positive integer. Default: `100` per speed.

Do not claim convergence solely because the configured iteration count was reached. Solver convergence should be judged from available residual/monitor information when that judgment is required.

## Output directory

- Use the user-specified output directory when given.
- If the user did not specify one, the Agent chooses a new independent result directory and passes it via `--workdir`. The name should include the model name and velocity information, plus a timestamp or other unique identifier when needed.
- If the target directory already exists and contains CFD results, do not silently overwrite it. Create a new unique directory, or ask the user when overwriting could cause data loss.

## Post-run result verification

After the process exits:

1. Read `result_manifest.json` in the work directory. Treat it as the primary machine-readable result contract.
2. Also check `mesh_quality_improvement_status.out` in the same directory.
3. Apply the outcome classification below.

### Clean success

Manifest `status == "success"`, every requested velocity present in `cases` with numeric force components, and no mesh-quality warning (see next section).

### SUCCESS_WITH_WARNINGS

If the manifest reports success but the mesh-quality status file shows `verified_final_quality = False`, or the final minimum Orthogonal Quality is below the tool's configured target (`quality_limit`), do NOT describe the run as "completely successful without warnings". Report the two facts separately:

- `CFD execution: completed`
- `Mesh quality: warning`

The overall engineering classification is `SUCCESS_WITH_WARNINGS`.

Important: `SUCCESS_WITH_WARNINGS` is only the Agent's engineering classification of the result for reporting purposes. Do NOT rewrite the original `status` field inside `result_manifest.json` — leave the file exactly as the tool wrote it.

### FAILED

If the CFD solve failed (manifest `status == "failed"` or `partial_failure`, or any requested velocity missing from `cases`), classify the run as `FAILED`. State which stage failed and surface the recorded error/log information. Do not fill in or estimate any numerical values.

## Numerical integrity

- Never invent force, pressure, velocity, residual, or convergence values.
- Never replace a failed speed with an interpolated value.
- Never modify the CAD, mesh rules, turbulence model, or boundary-condition physics unless the user explicitly requests an engineering change.
- Keep natural-language interpretation separate from deterministic numerical execution.
