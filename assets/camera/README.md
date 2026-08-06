# UVC32 wrist-camera mount asset

`UVC32_mount.usd` is a tessellated USD conversion of Seeed Studio's
`UVC32_mount.step` mechanical design for the reBot B601 DM wrist camera.
`UVC32_mount_portable.usd` contains the same geometry with the unused
`OmniPBR.mdl` binding removed; the simulation references this portable copy
so it has no dependency on an Isaac Sim material-library search path.

- Source: <https://github.com/Seeed-Projects/reBot-DevArm/blob/main/hardware/reBot_B601_DM/3D_Printed_Parts/UVC32_mount.step>
- Source repository hardware license: CERN Open Hardware Licence Version 2 —
  Weakly Reciprocal (CERN-OHL-W-2.0)
- Conversion: NVIDIA Omniverse HOOPS CAD converter, high tessellation,
  Z-up, metres per unit
- Converted bounds: approximately 36.0 × 69.4 × 30.8 mm

The USD keeps the source geometry in its original wrist-centred coordinates.
The scene adds a separate, render-only 32 mm UVC board and lens; none of these
camera visuals have collision or mass, so they do not alter arm dynamics.
