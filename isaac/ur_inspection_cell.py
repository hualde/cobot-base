"""Celda de inspección en Isaac Sim: UR + mesa con piezas + Intel RealSense D455 en la muñeca, con ros2_control.

Se ejecuta en el HOST con el Python de Isaac Sim (no en el contenedor), a través de:

    ./scripts/isaac_cell.sh [--robot ur5e] [--headless]

(el script carga antes las librerías ROS 2 que trae Isaac Sim: setup_ros_env.sh)

Qué publica / ofrece hacia ROS 2 (el contenedor dev lo consume con isaac_moveit.launch.py):
    /clock                                  tiempo de simulación
    /controller_manager + controladores     ros2_control dentro de Isaac (config/isaac_controllers.yaml)
    /wrist_camera/color/image_raw                   sensor_msgs/Image (rgb8)
    /wrist_camera/color/camera_info                 sensor_msgs/CameraInfo
    /wrist_camera/aligned_depth_to_color/image_raw  sensor_msgs/Image (32FC1, metros)
    /wrist_camera/depth/color/points                sensor_msgs/PointCloud2
Los nombres son los del driver oficial realsense2_camera (camera_name:=wrist_camera, align_depth.enable:=true),
así el mismo código vale en simulación y con la cámara real. Única diferencia: el driver real publica la
profundidad en 16UC1 (milímetros) y aquí sale en 32FC1 (metros); cv_bridge / image_geometry aceptan ambas.
Todo en el marco wrist_camera_color_optical_frame, el mismo que define urdf/ur_wrist_camera.urdf.xacro.

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
# Asset oficial de NVIDIA (malla, IMU y cámaras con los intrínsecos/posiciones de la D455 real).
REALSENSE_USD = "/Isaac/Sensors/RealSense/D455/rsd455.usd"
# De sus cámaras (Pseudo_Depth, OV9782_Color, OV9782_Left/Right) usamos la de color: RGB + profundidad alineada.
COLOR_CAMERA_PATH = f"{CAMERA_PATH}/RSD455/Camera_OmniVision_OV9782_Color"
CAMERA_FRAME_ID = "wrist_camera_color_optical_frame"
CAMERA_TOPIC = "wrist_camera"

# Montaje: COPIA de urdf/ur_wrist_camera.urdf.xacro (tool0 -> wrist_camera_bottom_screw_frame).
MOUNT_XYZ = (0.0, -0.052, -0.01)
MOUNT_RPY_DEG = (90.0, -90.0, 0.0)
# Origen del asset USD (entre los dos sensores IR, en el plano de profundidad cero; X adelante, Z arriba)
# respecto a la rosca de trípode. Sale de realsense2_description/_d455.urdf.xacro:
# x = 0.0158 - 0.0001 - 0.00455 (plano de profundidad), z = 0.029 / 2 (mitad de la altura de la carcasa).
SCREW_TO_ASSET = (0.01115, 0.0, 0.0145)

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


def add_wrist_camera(stage, assets_root: str):
    """Monta la RealSense D455 de NVIDIA tal y como la coloca urdf/ur_wrist_camera.urdf.xacro.

    Los ejes del asset coinciden con los de wrist_camera_link de realsense2_description (X hacia delante, Y a la
    izquierda, Z arriba), así que basta con la misma rotación del montaje y el desplazamiento SCREW_TO_ASSET.
    Las cámaras USD del asset ya están giradas por dentro (miran hacia +X), no hace falta corregirlas.
    """
    roll, pitch, yaw = MOUNT_RPY_DEG
    r_mount = rot("z", yaw) @ rot("y", pitch) @ rot("x", roll)   # convención rpy de URDF
    pos = np.array(MOUNT_XYZ) + r_mount @ np.array(SCREW_TO_ASSET)

    stage_utils.add_reference_to_stage(usd_path=assets_root + REALSENSE_USD, path=CAMERA_PATH)
    place(stage.GetPrimAtPath(CAMERA_PATH), pos, quat(r_mount))

    # El asset viene como sólido rígido independiente (para dejarlo caer sobre una mesa). Colgado de la muñeca,
    # PhysX lo trataría como otro cuerpo y se quedaría atrás al mover el brazo: se le quita, y así pasa a formar
    # parte de wrist_3_link (su masa de 103 g se suma a la del eslabón, como en el robot real).
    body = stage.GetPrimAtPath(f"{CAMERA_PATH}/RSD455")
    body.RemoveAPI(UsdPhysics.RigidBodyAPI)
    body.RemoveAppliedSchema("PhysxRigidBodyAPI")
    # Su IMU busca ese sólido rígido y llena el log de errores; no se publica a ROS, así que se desactiva.
    stage.GetPrimAtPath(f"{CAMERA_PATH}/RSD455/Imu_Sensor").SetActive(False)

    cam_prim = stage.GetPrimAtPath(COLOR_CAMERA_PATH)
    if not cam_prim.IsValid():
        raise RuntimeError(f"El asset {REALSENSE_USD} no tiene {COLOR_CAMERA_PATH}: ¿ha cambiado de versión?")
    cam = UsdGeom.Camera(cam_prim)
    # Intrínsecos del asset (HFOV ~90 grados, como el RGB de la D455); solo se ajusta la apertura vertical a la
    # resolución pedida para que los píxeles sean cuadrados.
    h_aperture = cam.GetHorizontalApertureAttr().Get()
    cam.GetVerticalApertureAttr().Set(h_aperture * args.height / args.width)
    # El asset trae 1 cm..1000 km; la D455 real no ve nada a menos de ~0,4 m, pero en la celda las piezas
    # quedan cerca, así que se deja un rango útil para la simulación.
    cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.1, 10.0))
    cam_prim.GetAttribute("omni:sensor:tickRate").Set(args.camera_hz)


# --------------------------------------------------------------------------------------------- ROS 2
def build_ros_graph():
    keys = og.Controller.Keys
    cam_types = [("Rgb", "rgb", f"{CAMERA_TOPIC}/color/image_raw"),
                 ("Depth", "depth", f"{CAMERA_TOPIC}/aligned_depth_to_color/image_raw"),
                 ("Points", "depth_pcl", f"{CAMERA_TOPIC}/depth/color/points")]
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
                ("RenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path(COLOR_CAMERA_PATH)]),
                ("RenderProduct.inputs:width", args.width),
                ("RenderProduct.inputs:height", args.height),
                ("CameraInfo.inputs:frameId", CAMERA_FRAME_ID),
                ("CameraInfo.inputs:topicName", f"{CAMERA_TOPIC}/color/camera_info"),
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
    add_wrist_camera(stage, assets_root)
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
