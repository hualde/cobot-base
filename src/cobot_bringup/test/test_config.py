"""Comprobaciones estáticas: URDF con cámara y coherencia controladores <-> MoveIt."""
import os
import subprocess
import xml.etree.ElementTree as ET

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

PKG_SRC = os.path.join(os.path.dirname(__file__), "..")
UR_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


@pytest.fixture(scope="module", params=["ur5e", "ur10e"])
def urdf(request):
    xacro_file = os.path.join(PKG_SRC, "urdf", "ur_wrist_camera.urdf.xacro")
    out = subprocess.run(["xacro", xacro_file, f"ur_type:={request.param}", "name:=ur"],
                         check=True, capture_output=True, text=True).stdout
    return ET.fromstring(out)


def test_camera_attached_to_tool0(urdf):
    joints = {j.get("name"): j for j in urdf.findall("joint")}
    assert joints["wrist_camera_joint"].find("parent").get("link") == "tool0"
    # Marcos de la D455 (realsense2_description); Isaac Sim publica las imágenes en el óptico de color.
    assert joints["wrist_camera_color_optical_joint"].find("child").get("link") == "wrist_camera_color_optical_frame"
    assert joints["wrist_camera_depth_optical_joint"].find("child").get("link") == "wrist_camera_depth_optical_frame"


def test_urdf_has_ur_joints(urdf):
    names = {j.get("name") for j in urdf.findall("joint")}
    assert set(UR_JOINTS) <= names


def test_controllers_match_moveit():
    with open(os.path.join(PKG_SRC, "config", "isaac_controllers.yaml")) as f:
        ours = yaml.safe_load(f)
    moveit_cfg = os.path.join(get_package_share_directory("ur_moveit_config"), "config", "moveit_controllers.yaml")
    with open(moveit_cfg) as f:
        moveit = yaml.safe_load(f)["moveit_simple_controller_manager"]
    default = next(n for n in moveit["controller_names"] if moveit[n].get("default"))
    assert default in ours["controller_manager"]["ros__parameters"], "MoveIt no encontraría su controlador por defecto"
    assert ours[default]["ros__parameters"]["joints"] == moveit[default]["joints"] == UR_JOINTS
