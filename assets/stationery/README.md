# Stationery task assets

The original `stationery_supplies_4k.zip` is Poly Haven's
[Stationery Supplies](https://polyhaven.com/a/stationery_supplies) asset and was imported from
`/home/seeed/Downloads` on 2026-08-06. Its SHA-256 is
`bccc519ebd26139b7882b7a4c78c2a84b4c4922105f259b92593e732083b495f`.

`stationery_models.usd` is the portable Z-up conversion used by the scene. It
keeps every source node as an independent model, including `PenRed`,
`PenBlue`, `Eraser`, and `Pencilcup`. The conversion script is
`../../scripts/convert_stationery_asset.py`.

The source glTF refers to a JPEG normal map that is not present in the archive
(the archive contains an EXR with a different extension). The scene repairs
that export mismatch explicitly: its `StationeryPBR` material uses the diffuse
JPEG, packed AO/roughness/metal JPEG, and supplied OpenGL-normal EXR. The mesh
also retains the authored vertex normals.

Poly Haven publishes this model under the
[CC0 licence](https://polyhaven.com/license), which permits use and
redistribution without attribution (credit is still appreciated). The asset
page credits Mateusz Sadek as the author.
