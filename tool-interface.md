# Tool Interface

## Fixed execution script

The formal execution script of the UUV CFD tool is fixed to:

`D:\pyfluent\run_uuv_cfd.py`

## Fixed Python interpreter

All invocations MUST use the project-bundled ANSYS PyFluent environment interpreter:

`D:\pyfluent\pyfluent_v241\Scripts\python.exe`

Using the system default `python`, the `py` launcher, or any other Python environment is FORBIDDEN — even if the version string happens to match. Only the bundled interpreter carries the PyFluent / ANSYS 2024 R1 packages the tool requires.

## Console encoding (Windows)

On Windows, Python standard output MUST be set to UTF-8 before launching the tool. On a Chinese-locale console (GBK / CP936) the driver crashes with `UnicodeEncodeError` on Unicode progress markers such as `▶`.

In PowerShell, set the environment variable first, then invoke the bundled interpreter:

```powershell
$env:PYTHONIOENCODING="utf-8"
```

## Standard command template

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

`--model` and `--velocities` are required. Optional flags: `--workdir` (the tool itself has a default work-directory fallback when omitted), `--cores` (default 6), `--iterations` (default 100), `--skip-contours`, and `--keep-fluent-open` (see Agent mode).

Agent/Skill rule: when the user does not specify `workdir`, the Agent MUST explicitly generate and pass a new independent output directory. Do not rely on the tool's default directory, and do not silently overwrite an existing directory that contains CFD results.

In bash-like shells the equivalent is to set the variable inline:

```bash
PYTHONIOENCODING=utf-8 "D:/pyfluent/pyfluent_v241/Scripts/python.exe" -u "D:/pyfluent/run_uuv_cfd.py" --model "..." --velocities 2 4 --cores 6 --iterations 100 --workdir "..."
```

## Example

```powershell
$env:PYTHONIOENCODING="utf-8"
& "D:\pyfluent\pyfluent_v241\Scripts\python.exe" `
  -u "D:\pyfluent\run_uuv_cfd.py" `
  --model "D:\models\uuv.x_t" `
  --velocities 2 4 6 8 `
  --workdir "D:\results\uuv_2_4_6_8" `
  --cores 6 `
  --iterations 100
```

## Environment-variable alternatives

The tool also supports:

- `AUV_MODEL_PATH`
- `AUV_WORK_DIR`
- `AUV_VELOCITIES`
- `AUV_CORES`
- `AUV_ITERATIONS`

CLI values take priority over environment variables. The fixed-interpreter and UTF-8 rules above apply regardless of how parameters are passed.

## Agent mode

For unattended Agent execution, do NOT add `--keep-fluent-open`. Add it only when the user explicitly asks to keep the Fluent GUI open for interactive inspection after completion. The tool will not stop at an interactive `input()` prompt and will attempt to close Fluent sessions after completion.
