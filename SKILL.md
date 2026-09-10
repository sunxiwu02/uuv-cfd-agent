---
name: uuv-cfd-analysis
description: >
  Run automated external-flow CFD analyses for UUV CAD models with one or more
  requested vehicle speeds. Use this skill when the user asks for UUV resistance,
  hydrodynamic force components, pressure/velocity contours, multi-speed Fluent CFD,
  or a CFD result set that will later be used to generate a report.
---

# UUV CFD Analysis

Use the deterministic CFD tool for all numerical work. The language model interprets the request and validates inputs; SpaceClaim/Fluent/Python perform the geometry, meshing, solve, extraction, and plotting. Never estimate or fabricate CFD values.

## Required inputs

Extract these from the user's request:

- `model_path`: path to a supported CAD file.
- `velocities`: one or more positive vehicle speeds in m/s.

Optional inputs:

- `workdir`: output directory.
- `cores`: Fluent processor count; default is 6.
- `iterations`: iterations per speed; default is 100.
- whether the user explicitly wants Fluent kept open for GUI inspection.
- whether contour images should be skipped.

`model_path` and `velocities` are required engineering inputs. The Agent must not guess or invent them.

- If the user did not provide a model path:
  - If the current task context contains exactly one clearly identified UUV CAD model, state which model will be used and ask the user to confirm before execution.
  - If multiple candidate CAD files exist, ask the user which one to use; never choose on their behalf.
- If the user did not provide any velocity or velocity range, ask which velocities to compute; never self-assign default speeds such as 2 m/s or 4 m/s.

## Natural-language parsing

Translate user language into explicit parameters. Examples:

- “2、4、6、8 m/s” -> `[2, 4, 6, 8]`.
- “2 到 8 m/s，每隔 2 m/s” -> `[2, 4, 6, 8]`.
- “航速 4 m/s” -> `[4]`.

Do not add unrequested speeds. Remove exact duplicates while preserving the user's intended order.

## Validation before execution

Read `references/validation.md` and apply it before running the tool. At minimum:

1. Verify the CAD path exists and has a supported solid-CAD extension.
2. Verify the solver script `D:\pyfluent\run_uuv_cfd.py` and the interpreter `D:\pyfluent\pyfluent_v241\Scripts\python.exe` exist.
3. Verify every velocity is finite and greater than zero.
4. Verify `cores > 0` and `iterations > 0`.
5. On Windows, plan to set Python stdout to UTF-8 before launch (see Tool invocation).
6. Do not edit the CFD solver source to satisfy a particular request; pass runtime parameters through the CLI.

## Tool invocation

The execution script is fixed to `D:\pyfluent\run_uuv_cfd.py`. Launch it with the project-bundled interpreter `D:\pyfluent\pyfluent_v241\Scripts\python.exe`. The system `python`, the `py` launcher, and any other Python environment are forbidden.

On Windows, set Python stdout to UTF-8 before launch — a GBK/CP936 console crashes the driver with `UnicodeEncodeError` on progress markers such as `▶`. PowerShell:

```powershell
$env:PYTHONIOENCODING="utf-8"
& "D:\pyfluent\pyfluent_v241\Scripts\python.exe" `
  -u "D:\pyfluent\run_uuv_cfd.py" `
  --model "<MODEL_PATH>" `
  --velocities <VELOCITIES> `
  --cores <CORES> `
  --iterations <ITERATIONS> `
  --workdir "<WORK_DIR>"
```

In bash-like shells, set the variable inline: `PYTHONIOENCODING=utf-8 "D:/pyfluent/pyfluent_v241/Scripts/python.exe" -u ...`.

If `workdir` is omitted, the Agent chooses a new independent result directory and passes it via `--workdir` — do not rely on or overwrite existing results. The directory name should include the model name and velocity information (e.g. `D:\pyfluent\uuv_cfd_2_4_6_8`), adding a timestamp or other unique identifier when needed.

If the target output directory already exists and contains CFD results, do not silently overwrite it. Create a new unique directory, or ask the user when overwriting could cause data loss.

Do not pass `--keep-fluent-open` for unattended Agent execution. Use it only when the user explicitly asks to inspect Fluent interactively after completion.

See `references/tool-interface.md` for the full command reference.

## Execution contract

The tool performs the deterministic engineering chain:

CAD -> SpaceClaim preprocessing -> adaptive mesh sizing -> Fluent Meshing -> fresh Fluent Solver -> one solve per requested speed -> force extraction -> pressure/velocity field extraction -> xoy/xoz contour PNGs -> Excel summary -> `result_manifest.json`.

Do not reimplement these numerical steps in the language model.

## Completion and verification

After the process exits, read `result_manifest.json` in the work directory. Treat it as the primary machine-readable result contract. Also check `mesh_quality_improvement_status.out` in the same directory.

A run is fully successful only when:

- `status == "success"`;
- every requested speed appears in `cases` with `status == "success"`;
- `forces_N.x_total`, `forces_N.y_total`, and `forces_N.z_total` are numeric for every successful case;
- the summary Excel path is present;
- expected contour files are present unless the user requested `--skip-contours`.

Outcome classification:

- Clean success: all criteria above and no mesh-quality warning.
- `SUCCESS_WITH_WARNINGS`: the manifest reports success but `mesh_quality_improvement_status.out` shows `verified_final_quality = False`, or a final minimum Orthogonal Quality below the tool's configured target. Do not describe the run as completely successful without warnings. Report `CFD execution: completed` and `Mesh quality: warning`. This is a reporting classification only — never rewrite the `status` field inside `result_manifest.json`.
- `FAILED`: the manifest says `partial_failure` or `failed`, or a requested speed is missing from `cases`. Do not claim success. State which speed or stage failed, surface the recorded error information, and do not fill in or estimate any numerical values.

Details: `references/validation.md`.

## Result handoff

For a later reporting task, hand `result_manifest.json` to the reporting workflow/Skill. The reporting layer should read the manifest and its referenced Excel/PNG artifacts instead of scraping terminal output.

For output field definitions, read `references/outputs.md`.
