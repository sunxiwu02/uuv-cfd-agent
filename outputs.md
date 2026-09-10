# Output Contract

The primary machine-readable output is:

`result_manifest.json`

## Top-level fields

- `schema_version`: current manifest schema version.
- `status`: `running`, `success`, `partial_failure`, or `failed`.
- `started_at`, `completed_at`: local offset-aware timestamps.
- `model.path`, `model.name`: analyzed CAD model.
- `work_dir`: output directory.
- `request.velocities_m_s`: requested speeds.
- `request.processor_count`: Fluent core count.
- `request.iterations_per_velocity`: configured iterations per speed.
- `cases`: one record per completed/failed speed.
- `artifacts`: run-level files such as summary Excel and force-speed charts.
- `errors`: structured failure notes.

## Per-speed `cases[].forces_N`

- `x_total`: total X-direction hydrodynamic force, N.
- `y_total`: total Y-direction hydrodynamic force, N.
- `z_total`: total Z-direction hydrodynamic force, N.
- `x_pressure`: X pressure/form-drag component, N.
- `x_friction`: X viscous/friction-drag component, N.

The X/Y/Z totals are taken from the Fluent `Results -> Reports -> Forces` Net Total vector when available.

## Per-speed artifacts

Typical entries:

- `case_file`
- `data_file`
- `surface_total_pressure`
- `contour_images`
- `report_files`

With contour generation enabled, each speed normally produces:

- `xoy_<speedtag>.png`
- `xoy_static_pressure_<speedtag>.png`
- `xoz_<speedtag>.png`
- `xoz_static_pressure_<speedtag>.png`

## Run-level artifacts

Typical outputs include:

- `auv_force_summary_all_speeds.xlsx`
- `force_x_vs_velocity.png`
- `force_y_vs_velocity.png`
- `force_z_vs_velocity.png`
- adaptive mesh summary JSON.

## Mesh quality status

The work directory also contains:

`mesh_quality_improvement_status.out`

Key fields:

- `verified_final_quality`: whether the tool confirmed the volume mesh reached the configured quality target after improvement.
- `quality_limit`: the configured Orthogonal Quality target.
- `minimum_quality_before` / `minimum_quality_after`: recorded minimum Orthogonal Quality.
- `continue_even_if_target_not_reached`: whether the pipeline was configured to proceed with the solve anyway.

Reporting outcome classification (`SUCCESS` / `SUCCESS_WITH_WARNINGS` / `FAILED`) is defined in validation.md. The original `status` field inside `result_manifest.json` must never be rewritten.

A reporting workflow should use this manifest to locate the authoritative result artifacts.
