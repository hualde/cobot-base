"""Celda de inspección en Isaac Sim: UR + mesa con piezas + cámara RGB-D en la muñeca, con ros2_control.

Se ejecuta en el HOST con el Python de Isaac Sim (no en el contenedor), a través de:

    ./scripts/isaac_cell.sh [--robot ur5e] [--headless]

(el script carga antes las librerías ROS 2 que trae Isaac Sim: setup_ros_env.sh)

Qué publica / ofrece hacia ROS 2 (el contenedor dev lo consume con isaac_moveit.launch.py):
    /clock                                  tiempo de simulación
    /controller_manager + controladores     ros2_control dentro de Isaac (config/isaac_controllers.yaml)
    /wrist_camera/image                     sensor_msgs/Image (rgb8)
    /wrist_camera/depth_image               sensor_msgs/Image (32FC1, metros)
    /wrist_camera/points                    sensor_msgs/PointCloud2
    /wrist_camera/camera_info               sensor_msgs/CameraInfo
Todo en el marco wrist_camera_optical_frame, el mismo que define urdf/ur_wrist_camera.urdf.xacro.

Patrón tomado de los ejemplos oficiales de Isaac Sim 6.2:
    standalone_examples/api/isaacsim.ros2.control/ur10_ros2_control_demo.py
    standalone_examples/api/isaacsim.ros2.bridge/camera_periodic.py
"""

import argparse
import math
import os
import sys

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--robot", default="ur5e", choices=["ur3e", "ur5e", "ur10e", "ur16e"])
parser.add_argument("--headless", action="store_true")
parser.add_argument("--width", type=int, default=640)
parser.add_argument("--height", type=int, default=480)
parser.add_argument("--camera-hz", type=float, default=15.0, help="frecuencia de publicación de la cámara")
parser.add_argument("--env", default="warehouse", choices=["warehouse", "warehouse_shelves", "none"],
                    help="fondo: almacén pequeño de NVIDIA, almacén con estanterías o escena vacía")
args, _ = parser.parse_known_args()

ENVIRONMENTS = {
    "warehouse": "/Isaac/Environments/Simple_Warehouse/warehouse.usd",
    "warehouse_shelves": "/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd",
}

simulation_app = SimulationApp({"renderer": "RayTracedLighting", "headless": args.headless})

# Los imports de Omniverse solo funcionan después de crear SimulationApp.
import carb  # noqa: E402
import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import isaacsim.core.experimental.utils.stage as stage_utils  # noqa: E402
import numpy as np  # noqa: E402
import omni.graph.core as og  # noqa: E402
import omni.usd  # noqa: E402
import usdrt.Sdf  # noqa: E402
from isaacsim.core.rendering_manager import ViewportManager  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402
from isaacsim.storage.native import get_assets_root_path_async  # noqa: E402
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONTROLLER_YAML = os.path.join(REPO, "src", "cobot_bringup", "config", "isaac_controllers.yaml")

ROBOT_PATH = "/World/UR"
WRIST_LINK = f"{ROBOT_PATH}/wrist_3_link"   # en estos assets, mismo marco que tool0 de ur_description
CAMERA_PATH = f"{WRIST_LINK}/wrist_camera"
CAMERA_FRAME_ID = "wrist_camera_optical_frame"

# Postura inicial: brazo "mirando" hacia abajo, delante de la base.
HOME_DEG = {
    "shoulder_pan_joint": 0.0, "shoulder_lift_joint": -90.0, "elbow_joint": 90.0,
    "wrist_1_joint": -90.0, "wrist_2_joint": -90.0, "wrist_3_joint": 0.0,
}


# --------------------------------------------------------------------------------------------- utilidades
def rot(axis: str, deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return {
        "x": np.array([[1, 0, 0], [0, c, -s], [0, s, c]]),
        "y": np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]),
        "z": np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]),
    }[axis]


def quat(r: np.ndarray) -> Gf.Quatd:
    """Matriz de rotación (convención columna) -> cuaternión USD."""
    w = math.sqrt(max(0.0, 1 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + r[0, 0] - r[1, 1] - r[2, 2])) / 2, r[2, 1] - r[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - r[0, 0] + r[1, 1] - r[2, 2])) / 2, r[0, 2] - r[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - r[0, 0] - r[1, 1] + r[2, 2])) / 2, r[1, 0] - r[0, 1])
    return Gf.Quatd(w, x, y, z)


def place(prim, pos, orient: Gf.Quatd | None = None):
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    if orient is not None:
        xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(orient)


def box(stage, path, size, pos, color, dynamic=False, mass=0.05, collision=True):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(1.0)
    place(cube.GetPrim(), pos)
    UsdGeom.Xformable(cube).AddScaleOp().Set(Gf.Vec3f(*size))
    cube.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])
    if collision:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    if dynamic:
        UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
        UsdPhysics.MassAPI.Apply(cube.GetPrim()).GetMassAttr().Set(mass)
    return cube


def cylinder(stage, path, radius, height, pos, color, dynamic=True, mass=0.05):
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.GetRadiusAttr().Set(radius)
    cyl.GetHeightAttr().Set(height)
    place(cyl.GetPrim(), pos)
    cyl.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])
    UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
    if dynamic:
        UsdPhysics.RigidBodyAPI.Apply(cyl.GetPrim())
        UsdPhysics.MassAPI.Apply(cyl.GetPrim()).GetMassAttr().Set(mass)
    return cyl


# --------------------------------------------------------------------------------------------- escena
def build_scene(stage, assets_root: str):
    UsdPhysics.Scene.Define(stage, "/World/physicsScene")

    if args.env in ENVIRONMENTS:
        # Entorno de NVIDIA: trae su propio suelo (con colisión) e iluminación interior.
        stage_utils.add_reference_to_stage(usd_path=assets_root + ENVIRONMENTS[args.env], path="/World/Environment")
    else:
        dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
        dome.GetIntensityAttr().Set(400.0)
        sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
        sun.GetIntensityAttr().Set(1200.0)
        place(sun.GetPrim(), (0, 0, 0), quat(rot("y", 30) @ rot("x", -20)))
        box(stage, "/World/Floor", (6.0, 6.0, 0.02), (0, 0, -0.01), (0.55, 0.55, 0.55))

    # Plataforma baja delante del robot (la cara superior queda a z = 0.05 m).
    box(stage, "/World/Platform", (0.40, 0.50, 0.05), (0.45, 0.15, 0.025), (0.62, 0.47, 0.30))

    # Piezas: dos "buenas" y una con una mancha (para probar detección de anomalías más adelante).
    top = 0.05
    box(stage, "/World/Parts/part_ok_1", (0.06, 0.04, 0.03), (0.40, 0.05, top + 0.015), (0.10, 0.35, 0.80), dynamic=True)
    cylinder(stage, "/World/Parts/part_ok_2", 0.025, 0.04, (0.52, 0.17, top + 0.02), (0.20, 0.70, 0.20))
    box(stage, "/World/Parts/part_defect", (0.06, 0.04, 0.03), (0.42, 0.27, top + 0.015), (0.10, 0.35, 0.80), dynamic=True)
    # La mancha es solo visual (sin colisión), pegada a la cara superior.
    stain = UsdGeom.Cylinder.Define(stage, "/World/Parts/part_defect_stain")
    stain.GetRadiusAttr().Set(0.008)
    stain.GetHeightAttr().Set(0.001)
    place(stain.GetPrim(), (0.43, 0.275, top + 0.0306))
    stain.GetDisplayColorAttr().Set([Gf.Vec3f(0.70, 0.10, 0.05)])

    # Robot (asset oficial de Isaac Sim, mismas articulaciones y eslabones que ur_description).
    stage_utils.add_reference_to_stage(
        usd_path=f"{assets_root}/Isaac/Robots/UniversalRobots/{args.robot}/{args.robot}.usd",
        path=ROBOT_PATH,
    )


def set_home_pose(stage):
    """Objetivo inicial de los drives (grados en USD). ros2_control mantiene esta postura al activar el JTC."""
    for joint, deg in HOME_DEG.items():
        prim = next((p for p in stage.Traverse()
                     if p.GetName() == joint and str(p.GetPath()).startswith(ROBOT_PATH)), None)
        if prim is None:
            carb.log_warn(f"No encuentro la articulación {joint}")
            continue
        UsdPhysics.DriveAPI.Get(prim, "angular").GetTargetPositionAttr().Set(deg)
        prim.CreateAttribute("state:angular:physics:position", Sdf.ValueTypeNames.Float).Set(deg)


def add_wrist_camera(stage):
    """Cámara montada como en urdf/ur_wrist_camera.urdf.xacro (respecto a tool0 == wrist_3_link).

    xacro:  tool0 -> wrist_camera_link:  xyz (0, -0.06, 0.02), rpy (0, -pi/2, 0)
            wrist_camera_link -> optical: rpy (-pi/2, 0, -pi/2)
    Una cámara USD mira hacia -Z con +Y arriba; el marco óptico ROS mira hacia +Z con +Y abajo:
    se pasa de uno a otro con un giro de 180 grados en X.
    """
    r_link = rot("y", -90)
    r_optical = rot("z", -90) @ rot("x", -90)
    r_usd = r_link @ r_optical @ rot("x", 180)

    cam = UsdGeom.Camera.Define(stage, CAMERA_PATH)
    place(cam.GetPrim(), (0.0, -0.06, 0.02), quat(r_usd))
    # Intrínsecos aproximados de una RealSense D435 (RGB): HFOV 69.4 grados.
    focal = 18.8
    h_aperture = 2 * focal * math.tan(math.radians(69.4 / 2))
    cam.GetFocalLengthAttr().Set(focal)
    cam.GetHorizontalApertureAttr().Set(h_aperture)
    cam.GetVerticalApertureAttr().Set(h_aperture * args.height / args.width)
    cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.1, 10.0))
    # Frecuencia del sensor (forma recomendada en 6.2; frameSkipCount está obsoleto).
    cam.GetPrim().ApplyAPI("OmniSensorAPI")
    cam.GetPrim().GetAttribute("omni:sensor:tickRate").Set(args.camera_hz)

    # Cuerpo de la cámara, solo visual (sin colisión: no altera la dinámica del brazo).
    box(stage, f"{WRIST_LINK}/wrist_camera_body", (0.025, 0.09, 0.025), (0.0, -0.06, 0.02),
        (0.15, 0.15, 0.15), collision=False)


# --------------------------------------------------------------------------------------------- ROS 2
def build_ros_graph():
    keys = og.Controller.Keys
    cam_types = [("Rgb", "rgb", "wrist_camera/image"),
                 ("Depth", "depth", "wrist_camera/depth_image"),
                 ("Points", "depth_pcl", "wrist_camera/points")]
    og.Controller.edit(
        {"graph_path": "/ROS2Graph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("ROS2ControlManager", "isaacsim.ros2.control.ROS2ControlManager"),
                ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("CameraInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ] + [(f"Camera{n}", "isaacsim.ros2.bridge.ROS2CameraHelper") for n, _, _ in cam_types],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("OnPlaybackTick.outputs:tick", "ROS2ControlManager.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "RenderProduct.inputs:execIn"),
                ("RenderProduct.outputs:execOut", "CameraInfo.inputs:execIn"),
                ("RenderProduct.outputs:renderProductPath", "CameraInfo.inputs:renderProductPath"),
            ] + [c for n, _, _ in cam_types for c in (
                ("RenderProduct.outputs:execOut", f"Camera{n}.inputs:execIn"),
                ("RenderProduct.outputs:renderProductPath", f"Camera{n}.inputs:renderProductPath"),
            )],
            keys.SET_VALUES: [
                ("ROS2ControlManager.inputs:targetPrim", [usdrt.Sdf.Path(ROBOT_PATH)]),
                ("ROS2ControlManager.inputs:controllerConfig", CONTROLLER_YAML),
                # La descripción del robot (con la cámara) la publica robot_state_publisher en el contenedor.
                ("ROS2ControlManager.inputs:publishRobotDescription", False),
                ("RenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path(CAMERA_PATH)]),
                ("RenderProduct.inputs:width", args.width),
                ("RenderProduct.inputs:height", args.height),
                ("CameraInfo.inputs:frameId", CAMERA_FRAME_ID),
                ("CameraInfo.inputs:topicName", "wrist_camera/camera_info"),
            ] + [v for n, kind, topic in cam_types for v in (
                (f"Camera{n}.inputs:type", kind),
                (f"Camera{n}.inputs:topicName", topic),
                (f"Camera{n}.inputs:frameId", CAMERA_FRAME_ID),
            )],
        },
    )


# --------------------------------------------------------------------------------------------- main
def main() -> int:
    for ext in ("isaacsim.ros2.core", "isaacsim.ros2.bridge", "isaacsim.ros2.control"):
        app_utils.enable_extension(ext)
    simulation_app.update()

    stage_utils.create_new_stage()
    stage_utils.set_stage_units(meters_per_unit=1.0)
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    try:
        assets_root = simulation_app.run_coroutine(get_assets_root_path_async())
    except RuntimeError as exc:
        carb.log_error(f"No encuentro los assets de Isaac Sim: {exc}")
        simulation_app.close()
        return 1

    build_scene(stage, assets_root)
    simulation_app.update()
    set_home_pose(stage)
    add_wrist_camera(stage)
    simulation_app.update()
    ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[1.6, 1.4, 1.1], target=[0.35, 0.1, 0.2])

    build_ros_graph()

    SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
    app_utils.play()
    carb.log_warn("Celda en marcha. En el contenedor dev: ros2 launch cobot_bringup isaac_moveit.launch.py "
                  f"ur_type:={args.robot}")

    try:
        while simulation_app.is_running():
            simulation_app.update()
    except KeyboardInterrupt:
        pass

    app_utils.stop()
    simulation_app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
