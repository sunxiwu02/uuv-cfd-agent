# CFD Workflow Contract

The current UUV CFD tool is intentionally a deterministic execution pipeline.

1. Resolve CAD model and work directory.
2. Run SpaceClaim geometry preprocessing and geometry-feature extraction.
3. Compute adaptive target mesh sizes.
4. Launch Fluent Meshing and generate the external-flow mesh.
5. Save the mesh and start a fresh Fluent Solver session.
6. Create `xoy` and `xoz` planes.
7. Set water material, inlet/outlet/symmetry conditions, residual criteria, and force reports.
8. For each requested speed:
   - set inlet velocity;
   - initialize;
   - run the configured number of iterations;
   - extract and save xoy/xoz velocity and static-pressure contours using PyFluent field data + Python rendering;
   - save case/data;
   - export UUV-surface total pressure;
   - extract total X/Y/Z force and X pressure/friction drag components;
   - refresh the multi-speed Excel summary.
9. Generate force-versus-speed charts when at least two speeds exist.
10. Write the final `result_manifest.json`.

The Agent should not reproduce these solver steps itself. It should validate the request, invoke the tool, inspect the manifest, and explain the outcome.
