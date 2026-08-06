#!/usr/bin/env python3
"""Create a portable camera-mount USD without Isaac Sim MDL references."""

from __future__ import annotations

import argparse
from pathlib import Path

from pxr import Sdf


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()

    source = Sdf.Layer.FindOrOpen(str(args.source.resolve()))
    if source is None:
        raise RuntimeError(f"Cannot open source layer: {args.source}")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    output = Sdf.Layer.CreateNew(str(args.output.resolve()))
    output.TransferContent(source)

    # CAD conversion embeds the same MDL shader inside normal prims, prototype
    # prims and flattened prototypes.  Traverse the Sdf layer (not a composed
    # UsdStage) so instance/prototype specs are covered as well.
    mdl_shader_paths: list[Sdf.Path] = []

    def collect(path: Sdf.Path) -> None:
        if path.IsPrimPath() and path.name == "MDLShader":
            mdl_shader_paths.append(path)

    output.Traverse(Sdf.Path.absoluteRootPath, collect)
    edits = Sdf.BatchNamespaceEdit()
    for path in sorted(mdl_shader_paths, key=lambda item: len(str(item)), reverse=True):
        edits.Add(path, Sdf.Path.emptyPath)
    if not output.Apply(edits):
        raise RuntimeError("Failed to remove embedded MDL shader prims")

    # Connections to the removed MDL shaders are harmless but unnecessary.
    stale_properties: list[Sdf.PropertySpec] = []

    def collect_stale(path: Sdf.Path) -> None:
        spec = output.GetObjectAtPath(path)
        if isinstance(spec, Sdf.PropertySpec) and ":mdl:" in spec.name:
            stale_properties.append(spec)

    output.Traverse(Sdf.Path.absoluteRootPath, collect_stale)
    for prop in stale_properties:
        prop.owner.RemoveProperty(prop)

    output.Save()
    print(
        f"[asset] wrote {args.output.resolve()} "
        f"(removed {len(mdl_shader_paths)} MDL shaders)"
    )


if __name__ == "__main__":
    main()
