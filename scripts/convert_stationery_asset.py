#!/usr/bin/env python3
"""Convert the bundled glTF stationery meshes to a portable Z-up USD."""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt


COMPONENT_DTYPES = {
    5121: np.dtype("<u1"),
    5123: np.dtype("<u2"),
    5125: np.dtype("<u4"),
    5126: np.dtype("<f4"),
}
TYPE_WIDTHS = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}


def _read_accessor(document: dict, payload: bytes, index: int) -> np.ndarray:
    accessor = document["accessors"][index]
    view = document["bufferViews"][accessor["bufferView"]]
    dtype = COMPONENT_DTYPES[accessor["componentType"]]
    width = TYPE_WIDTHS[accessor["type"]]
    count = int(accessor["count"])
    offset = int(view.get("byteOffset", 0)) + int(accessor.get("byteOffset", 0))
    packed_stride = dtype.itemsize * width
    stride = int(view.get("byteStride", packed_stride))
    if stride == packed_stride:
        values = np.frombuffer(
            payload, dtype=dtype, count=count * width, offset=offset
        ).reshape(count, width)
        return values.copy()
    values = np.empty((count, width), dtype=dtype)
    for row in range(count):
        start = offset + row * stride
        values[row] = np.frombuffer(
            payload, dtype=dtype, count=width, offset=start
        )
    return values


def _safe_name(name: str) -> str:
    words = re.sub(r"^stationery_supplies_", "", name).split("_")
    return "".join(word.capitalize() for word in words)


def _add_material(stage: Usd.Stage, texture_path: str) -> UsdShade.Material:
    material = UsdShade.Material.Define(stage, "/Stationery/Looks/Stationery")
    shader = UsdShade.Shader.Define(
        stage, "/Stationery/Looks/Stationery/PreviewSurface"
    )
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.48)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.03)

    reader = UsdShade.Shader.Define(
        stage, "/Stationery/Looks/Stationery/PrimvarReader"
    )
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    texture = UsdShade.Shader.Define(
        stage, "/Stationery/Looks/Stationery/BaseColorTexture"
    )
    texture.CreateIdAttr("UsdUVTexture")
    texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(texture_path)
    texture.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    texture.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        reader.ConnectableAPI(), "result"
    )
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        texture.ConnectableAPI(), "rgb"
    )
    material.CreateSurfaceOutput().ConnectToSource(
        shader.ConnectableAPI(), "surface"
    )
    return material


def convert(source: Path, output: Path) -> None:
    document = json.loads(source.read_text(encoding="utf-8"))
    buffer_uri = document["buffers"][0]["uri"]
    payload = (source.parent / buffer_uri).read_bytes()

    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        output.unlink()
    stage = Usd.Stage.CreateNew(str(output.resolve()))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/Stationery")
    stage.SetDefaultPrim(root.GetPrim())
    UsdGeom.Scope.Define(stage, "/Stationery/Looks")
    material = _add_material(
        stage, "./textures/stationery_supplies_diff_4k.jpg"
    )

    for node in document["nodes"]:
        source_primitive = document["meshes"][node["mesh"]]["primitives"][0]
        attributes = source_primitive["attributes"]
        source_points = _read_accessor(
            document, payload, attributes["POSITION"]
        ).astype(np.float64)
        source_normals = _read_accessor(
            document, payload, attributes["NORMAL"]
        ).astype(np.float64)
        source_uvs = _read_accessor(
            document, payload, attributes["TEXCOORD_0"]
        ).astype(np.float32)
        # glTF and UsdUVTexture use opposite image-V conventions in this
        # pipeline. Without this flip, the red pen samples the beige atlas
        # region, the blue pen appears tan, and the pink eraser appears grey.
        source_uvs[:, 1] = 1.0 - source_uvs[:, 1]
        indices = _read_accessor(
            document, payload, source_primitive["indices"]
        ).reshape(-1).astype(np.int32)
        if len(indices) % 3:
            raise ValueError(f"Non-triangle mesh: {node['name']}")

        # glTF is Y-up. Map (x, y, z) -> USD Z-up (x, z, y). The axis swap
        # reverses handedness, so reverse each triangle's winding as well.
        points = source_points[:, [0, 2, 1]]
        normals = source_normals[:, [0, 2, 1]]
        indices = indices.reshape(-1, 3)[:, [0, 2, 1]].reshape(-1)

        model_name = _safe_name(node["name"])
        model = UsdGeom.Xform.Define(stage, f"/Stationery/{model_name}")
        mesh = UsdGeom.Mesh.Define(stage, f"/Stationery/{model_name}/Mesh")
        mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(points.astype(np.float32)))
        mesh.CreateNormalsAttr(
            Vt.Vec3fArray.FromNumpy(normals.astype(np.float32))
        )
        mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
        mesh.CreateFaceVertexCountsAttr([3] * (len(indices) // 3))
        mesh.CreateFaceVertexIndicesAttr(indices.tolist())
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        primvars = UsdGeom.PrimvarsAPI(mesh)
        st = primvars.CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex
        )
        st.Set(Vt.Vec2fArray.FromNumpy(source_uvs))
        UsdShade.MaterialBindingAPI(mesh.GetPrim()).Bind(material)

        minimum = points.min(axis=0)
        maximum = points.max(axis=0)
        mesh.CreateExtentAttr(
            [Gf.Vec3f(*minimum.tolist()), Gf.Vec3f(*maximum.tolist())]
        )
        model.GetPrim().SetCustomDataByKey("sourceNode", node["name"])

    stage.GetRootLayer().Save()
    print(f"[asset] generated {output.resolve()}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    convert(args.source.resolve(), args.output.resolve())


if __name__ == "__main__":
    main()
