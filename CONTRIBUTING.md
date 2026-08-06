# Contributing

Thank you for contributing to reBot Arm DLI Isaac Sim.

## Development setup

1. Install Isaac Sim and set `ISAACSIM_ROOT` if it is not installed at
   `/home/seeed/isaacsim`.
2. Configure the reBot LeRobot environment referenced by
   `config/teleop_config.json`.
3. Run the offline regression before submitting changes:

   ```bash
   ./run.sh --test
   ```

4. For camera or recording changes, also run:

   ```bash
   ./run.sh --test --test-recording
   ```

## Pull requests

- Keep the default application fully leader-controlled; do not introduce
  autonomous arm movement into the teleoperation entry point.
- Preserve the `front` and `side` dataset field names.
- Keep all scene asset references relative to the repository.
- Do not commit datasets, logs, outputs, Python caches, or machine-specific
  calibration files.
- Document physics, camera, control, and dataset schema changes in README.md.
- Include provenance and license information for every new third-party asset.

## Safety

Test mapping and gripper changes in simulation before connecting physical
hardware. Keep serial-port and UDP endpoints configurable, and do not commit
credentials or private datasets.
