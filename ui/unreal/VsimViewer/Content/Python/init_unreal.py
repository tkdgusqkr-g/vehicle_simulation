"""Runs when the editor starts (Python Editor Script Plugin): creates the PBR
materials the vsim viewer instances at run time, if they are missing.

  /Game/Vsim/M_VsimPBR           Color (vector), Metallic, Roughness (scalars)
  /Game/Vsim/M_VsimPBR_TwoSided  same, two sided (cockpit, tents)
  /Game/Vsim/M_VsimPaint         same, clear coat car paint

Without them the viewer falls back to the engine's basic shape material.
"""
import unreal

FOLDER = "/Game/Vsim"


def make_material(name, two_sided=False, clear_coat=False):
    path = f"{FOLDER}/{name}"
    if unreal.EditorAssetLibrary.does_asset_exist(path):
        return
    tools = unreal.AssetToolsHelpers.get_asset_tools()
    material = tools.create_asset(name, FOLDER, unreal.Material, unreal.MaterialFactoryNew())
    lib = unreal.MaterialEditingLibrary

    color = lib.create_material_expression(material, unreal.MaterialExpressionVectorParameter, -500, -200)
    color.set_editor_property("parameter_name", "Color")
    color.set_editor_property("default_value", unreal.LinearColor(0.5, 0.5, 0.5, 1.0))
    metallic = lib.create_material_expression(material, unreal.MaterialExpressionScalarParameter, -500, 0)
    metallic.set_editor_property("parameter_name", "Metallic")
    metallic.set_editor_property("default_value", 0.0)
    roughness = lib.create_material_expression(material, unreal.MaterialExpressionScalarParameter, -500, 120)
    roughness.set_editor_property("parameter_name", "Roughness")
    roughness.set_editor_property("default_value", 0.8)

    lib.connect_material_property(color, "", unreal.MaterialProperty.MP_BASE_COLOR)
    lib.connect_material_property(metallic, "", unreal.MaterialProperty.MP_METALLIC)
    lib.connect_material_property(roughness, "", unreal.MaterialProperty.MP_ROUGHNESS)

    if clear_coat:
        material.set_editor_property("shading_model", unreal.MaterialShadingModel.MSM_CLEAR_COAT)
        coat = lib.create_material_expression(material, unreal.MaterialExpressionConstant, -500, 260)
        coat.set_editor_property("r", 1.0)
        coat_roughness = lib.create_material_expression(material, unreal.MaterialExpressionConstant, -500, 340)
        coat_roughness.set_editor_property("r", 0.05)
        lib.connect_material_property(coat, "", unreal.MaterialProperty.MP_CUSTOM_DATA0)
        lib.connect_material_property(coat_roughness, "", unreal.MaterialProperty.MP_CUSTOM_DATA1)
    if two_sided:
        material.set_editor_property("two_sided", True)

    lib.recompile_material(material)
    unreal.EditorAssetLibrary.save_asset(path)
    unreal.log(f"vsim: created {path}")


try:
    make_material("M_VsimPBR")
    make_material("M_VsimPBR_TwoSided", two_sided=True)
    make_material("M_VsimPaint", clear_coat=True)
except Exception as error:  # never stop the editor from starting
    unreal.log_warning(f"vsim: could not create the PBR materials: {error}")
