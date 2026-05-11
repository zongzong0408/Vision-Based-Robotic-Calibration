from typing import Optional, List, Tuple
import pybullet as p
import pybullet_data
import numpy as np
import threading
import queue
import time
import math
import sys
import cv2
import os
GRAVITY = -10.0
UPDATE = 1.0 / 1000.0
VIEWS = [
    {"name": "Default (XYZ)", "yaw": 45, "pitch": -30, "distance": 1.5},
    {"name": "Top-down (XY)", "yaw": 90, "pitch": -89, "distance": 1.5},
    {"name": "Front (XZ)", "yaw": 90, "pitch": 0, "distance": 1.5},
    {"name": "Side (YZ)", "yaw": 0, "pitch": 0, "distance": 1.5},
    {"name": "Back (-XZ)", "yaw": -90, "pitch": 0, "distance": 1.5},
    {"name": "Left (-YZ)", "yaw": 180, "pitch": 0, "distance": 1.5}
]
BASE_PLANE_SIZE = [0.5, 0.5, 0.05]
BASE_PLANE_MASS = 0
BASE_PLANE_FRICTION_LATERAL = 0.5
BASE_PLANE_FRICTION_SPINNING = 0.165
BASE_PLANE_FRICTION_ROLLING = 0.025
BASE_PLANE_POSITION = [0.0, 0.0, 0.0]
BASE_PLANE_COLOR = [1, 1, 1, 0.9]
GOOD_PLANE_SIZE = [0.5, 0.5, 0.05]
GOOD_PLANE_MASS = 0
GOOD_PLANE_FRICTION_LATERAL = 0.5
GOOD_PLANE_FRICTION_SPINNING = 0.16
GOOD_PLANE_FRICTION_ROLLING = 0.025
GOOD_PLANE_POSITION = [0.5, 0, 0]
GOOD_PLANE_COLOR = [0, 1, 0, 1]
POOR_PLANE_SIZE = [0.5, 0.5, 0.05]
POOR_PLANE_MASS = 0
POOR_PLANE_FRICTION_LATERAL = 0.5
POOR_PLANE_FRICTION_SPINNING = 0.16
POOR_PLANE_FRICTION_ROLLING = 0.025
POOR_PLANE_POSITION = [-0.5, 0, 0]
POOR_PLANE_COLOR = [1, 0, 0, 1]
GOOD_OBJECT_SIZE = [0.09, 0.09, 0.03]
GOOD_OBJECT_MASS = 0.3
GOOD_OBJECT_FRICTION_LATERAL = 5.0
GOOD_OBJECT_FRICTION_SPINNING = 1.0
GOOD_OBJECT_FRICTION_ROLLING = 0.05
GOOD_OBJECT_POSITION = [0, 0, 0.07]
GOOD_OBJECT_COLOR = [1, 1, 0, 1]
POOR_OBJECT_SIZE = [1.00, 0.09, 0.03]
POOR_OBJECT_MASS = 0.3
POOR_OBJECT_FRICTION_LATERAL = 5.0
POOR_OBJECT_FRICTION_SPINNING = 1.0
POOR_OBJECT_FRICTION_ROLLING = 0.05
POOR_OBJECT_POSITION = [0, 0.25, 0.07]
POOR_OBJECT_COLOR = [1, 1, 0, 1]
ROBOT_URDF_PATH = "xarm/xarm6_with_gripper.urdf"
ROBOT_POSITION = [0.0, 0.3, 0.0]
ROBOT_JOINTS_DEFAULT_RADIAN = [0, 0, -2, 0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0, 0]
ROBOT_GRIPPER_INDEXS = [9, 12]
ROBOT_GRIP_OPEN_RADIAN = 0.85
ROBOT_GRIP_CLOSE_RADIAN = 0.0
ROBOT_JOINT_FORCE = 250
ROBOT_GRIP_FORCE = 2000
ROBOT_STABLE_STEP = 100
ROBOT_STABLE_FRAME_WAIT = 100
CAMERA_SIZE = [0.07, 0.03, 0.03]
CAMERA_MASS = 0.01
CAMERA_INDEX = 0
CAMERA_LINK_ROBOT_INDEX = 6
CAMERA_IMAGE_WIDTH = 500
CAMERA_IMAGE_HEIGHT = 500
CAMERA_FIELD_OF_VIEW = 60
CAMERA_ASPECT_RATIO = 1.0
CAMERA_NEAR_PLANE = 0.01
CAMERA_FAR_PLANE = 5.0
CAMERA_POSITION = [0.0, 0.0, 0.0]
CAMERA_OFFSET_POSITION = [0.05, 0.00, 0.10]
CAMERA_WATCH_TARGET_POSITION = [0.0, 0.0, 0.2]
CAMERA_COLOR = [0, 0, 0, 1]
CAMERA_CAPTURE_STABLE_FRAME_WAIT = 200
alive = True
mode = "init"
view = 3
state = 1
move = False
slope = [0.0, 0.0, 0.0]
terminal_lock = True
INIT_WAIT_TIME = 1
trajectory_lock = threading.Lock()
_open = ROBOT_GRIP_OPEN_RADIAN
_close = ROBOT_GRIP_CLOSE_RADIAN
trajectory_smooth_step = 5
zero = 0.000
INIT_POINT  = [ zero,  0.000, -2.000,  0.000,  zero,  0.500,  zero,  zero,  zero, _open,  zero,  zero, _open,  zero]
ZERO_POINT  = [ zero, -1.570,  0.000, -1.600,  zero,  0.000,  zero,  zero,  zero, _open,  zero,  zero, _open,  zero]
PXY_POINT   = [ zero, -1.570, -0.800, -0.250,  zero,  1.050,  zero,  zero,  zero, _open,  zero,  zero, _open,  zero]
PYZ_POINT   = [ zero, -1.570,  1.500, -2.700,  zero,  2.770,  zero,  zero,  zero, _open,  zero,  zero, _open,  zero]
trajectory  = [
    INIT_POINT,
    ZERO_POINT,
    PXY_POINT,
    [0.000, -1.570, 1.350, -2.700, 0.000, 2.750, 0.000, 0.000, 0.000, _open, 0.000, 0.000, _open, 0.00],
    PYZ_POINT,
]
current_trajectory_accumulate = 0
can_move_next = True
mode_input_queue = queue.Queue()
current_robot_joints_radian_lock = threading.Lock()
current_robot_joints_radian = ROBOT_JOINTS_DEFAULT_RADIAN.copy()
plus_xy_length: List[float] = [-1.0, -1.0, -1.0]
minus_yz_length: List[float] = [-1.0, -1.0, -0.0]
def initialize_environment_simulation(using_GUI: bool = True) -> None:
    """
    初始化 PyBullet 模擬環境。
    設定連接模式、重力、時間步長，並配置調試視覺化選項。
    繪製環境座標軸並加載地面 URDF 模型。
    Args:
        using_GUI (bool): 如果為 True，則使用 GUI 連接模式；否則使用 DIRECT 模式。
    Returns:
        None
    """
    if using_GUI:
        p.connect(p.GUI)
    else:
        p.connect(p.DIRECT)
    p.setGravity(0, 0, GRAVITY)
    p.setTimeStep(UPDATE)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
    p.addUserDebugLine([0, 0, 0], [100, 0, 0], [1, 0, 0], lineWidth=1)
    p.addUserDebugLine([0, 0, 0], [0, 100, 0], [0, 1, 0], lineWidth=1)
    p.addUserDebugLine([0, 0, 0], [0, 0, 100], [0, 0, 1], lineWidth=1)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    return None
def set_pybullet_camera_view(view_index: int, target: List[float] = [0, 0, 0.1]) -> None:
    """
    設定 PyBullet 模擬環境的相機視角。
    Args:
        view_index (int): VIEWS 列表中視角的索引。
        target (List[float]): 相機觀察的目標位置 [x, y, z]。
    Returns:
        None
    """
    global view
    cam = VIEWS[view_index]
    p.resetDebugVisualizerCamera(
        cameraDistance=cam["distance"],
        cameraYaw=cam["yaw"],
        cameraPitch=cam["pitch"],
        cameraTargetPosition=target
    )
    view = view_index
    return None
def load_box(
        size: List[float],
        position: List[float],
        color: List[int],
        mass: float = 0.0,
        friction_lateral: float = 0.0,
        friction_spinning: float = 0.0,
        friction_rolling: float = 0.0
    ) -> int:
    """
    在 PyBullet 模擬環境中加載一個箱型物體。
    Args:
        size (List[float]): 箱子的尺寸 [長, 寬, 高]。
        position (List[float]): 箱子的初始位置 [x, y, z]。
        color (List[int]): 箱子的顏色 [R, G, B, Alpha]。
        mass (float): 箱子的質量，0 表示靜態物體。
        friction_lateral (float): 箱子的側向摩擦係數。
        friction_spinning (float): 箱子的旋轉摩擦係數。
        friction_rolling (float): 箱子的滾動摩擦係數。
    Returns:
        int: 加載物體在 PyBullet 中的唯一 ID。
    """
    visual_shape_id = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[l / 2 for l in size],
        rgbaColor=color
    )
    collision_shape_id = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=[l / 2 for l in size]
    )
    box_id = p.createMultiBody(mass, collision_shape_id, visual_shape_id, position)
    p.changeDynamics(
        box_id,
        -1,
        lateralFriction=friction_lateral,
        spinningFriction=friction_spinning,
        rollingFriction=friction_rolling
    )
    return box_id
def load_robot(
        initial_joint_radian: List[float] = ROBOT_JOINTS_DEFAULT_RADIAN,
        gripper_friction_lateral: float = 0.5,
        gripper_friction_spinning: float = 0.165,
        gripper_friction_rolling: float = 0.025
    ) -> int:
    """
    在 PyBullet 模擬環境中加載機器人模型。
    Args:
        initial_joint_radian (List[float]): 機器人各關節的初始角度（弧度）。
        gripper_friction_lateral (float): 夾爪的側向摩擦係數。
        gripper_friction_spinning (float): 夾爪的旋轉摩擦係數。
        gripper_friction_rolling (float): 夾爪的滾動摩擦係數。
    Returns:
        int: 加載機器人模型在 PyBullet 中的唯一 ID。
    Raises:
        ValueError: 如果提供的初始關節角度列表長度與機器人實際關節數量不符。
    """
    robot_id = p.loadURDF(ROBOT_URDF_PATH, ROBOT_POSITION, useFixedBase=True)
    actual_joints_count = p.getNumJoints(robot_id)
    if len(initial_joint_radian) != actual_joints_count:
        raise ValueError(
            f"[ERROR]:\t您提供的 initial_joint_radian(list) 長度 ({len(initial_joint_radian)}) "
            f"[ERROR]:\t與機器人預期關節數量 ({actual_joints_count}) 不符。"
            f"[ERROR]:\t請確保列表長度正確。"
        )
    for index, rad in enumerate(initial_joint_radian):
        p.resetJointState(robot_id, index, targetValue=rad)
    p.changeDynamics(robot_id, ROBOT_GRIPPER_INDEXS[0],
                     lateralFriction=gripper_friction_lateral,
                     spinningFriction=gripper_friction_spinning,
                     rollingFriction=gripper_friction_rolling
                     )
    p.changeDynamics(robot_id, ROBOT_GRIPPER_INDEXS[1],
                     lateralFriction=gripper_friction_lateral,
                     spinningFriction=gripper_friction_spinning,
                     rollingFriction=gripper_friction_rolling
                     )
    return robot_id
def load_camera(
        robot_id: int,
        camera_id: int,
    ) -> int:
    """
    將相機模型加載並附加到機器人手臂的指定連桿上。
    Args:
        robot_id (int): 機器人模型在 PyBullet 中的唯一 ID。
        camera_id (int): 相機模型在 PyBullet 中的唯一 ID。
    Returns:
        int: 相機模型在 PyBullet 中的唯一 ID。
    """
    link_state = p.getLinkState(robot_id, CAMERA_LINK_ROBOT_INDEX, computeForwardKinematics=True)
    link_position = link_state[0]
    link_orientation = link_state[1]
    additional_rotation_quaternion = p.getQuaternionFromEuler([0, 0, np.pi / 2])
    rotated_camera_orientation = p.multiplyTransforms(
        [0, 0, 0],
        link_orientation,
        [0, 0, 0],
        additional_rotation_quaternion
    )[1]
    link_rotation_matrix = np.array(p.getMatrixFromQuaternion(link_orientation)).reshape(3, 3)
    camera_world_offset = link_rotation_matrix @ np.array(CAMERA_OFFSET_POSITION)
    camera_world_position = np.array(link_position) + camera_world_offset
    p.resetBasePositionAndOrientation(camera_id, camera_world_position.tolist(), rotated_camera_orientation)
    p.createConstraint(
        parentBodyUniqueId=robot_id,
        parentLinkIndex=CAMERA_LINK_ROBOT_INDEX,
        childBodyUniqueId=camera_id,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        parentFramePosition=CAMERA_OFFSET_POSITION,
        parentFrameOrientation=additional_rotation_quaternion,
        childFramePosition=[0, 0, 0],
        childFrameOrientation=[0, 0, 0, 1]
    )
    return camera_id
def robot_move(
        robot_id: int,
        target: List[float],
    ) -> None:
    """
    設定機器人各關節的目標位置（弧度），使機器人移動。
    Args:
        robot_id (int): 機器人模型在 PyBullet 中的唯一 ID。
        target (List[float]): 機器人各關節的目標角度（弧度）列表。
    Returns:
        None
    """
    global move
    num_joints = p.getNumJoints(robot_id)
    for index in range(min(len(target), num_joints)):
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=index,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target[index],
            force=ROBOT_JOINT_FORCE
        )
    move = True
    return None
def robot_move_smoothly(
        robot_id: int,
        current: List[float],
        target: List[float],
        max_difference_radian_step: float,
    ) -> List[float]:
    """
    計算機器人平滑移動到目標位置的下一步關節角度。
    每個關節的移動步長會限制在 `max_difference_radian_step` 之內。
    Args:
        robot_id (int): 機器人模型在 PyBullet 中的唯一 ID。
        current (List[float]): 機器人當前各關節的角度（弧度）列表。
        target (List[float]): 機器人各關節的目標角度（弧度）列表。
        max_difference_radian_step (float): 每個模擬步長中，關節角度的最大變化量（弧度）。
    Returns:
        List[float]: 計算出的下一步關節角度列表。
    """
    updated_positions: List[float] = []
    for current_rad, target_rad in zip(current, target):
        delta = target_rad - current_rad
        if abs(delta) < max_difference_radian_step:
            updated_positions.append(target_rad)
        else:
            updated_positions.append(current_rad + max_difference_radian_step * np.sign(delta))
    robot_move(robot_id, updated_positions)
    return updated_positions
def robot_move_to_xyz(
        robot_id: int,
        target_xyz: List[float],
        link_index: int = CAMERA_LINK_ROBOT_INDEX,
        target_orientation: Optional[List[float]] = None
    ) -> Optional[List[float]]:
    """
    計算機器人手臂末端執行器移動到指定 XYZ 座標所需的關節角度。
    Args:
        robot_id (int): 機器人模型在 PyBullet 中的唯一 ID。
        target_xyz (List[float]): 目標位置 [x, y, z]。
        link_index (int): 機器人手臂上要移動到目標位置的連桿索引。
        target_orientation (Optional[List[float]]): 可選的目標方向 (四元數)。
                                                     如果為 None，則 PyBullet 會嘗試保持當前方向。
    Returns:
        Optional[List[float]]: 如果成功計算出逆運動學，則返回目標關節角度列表，否則返回 None。
    """
    if target_orientation is None:
        current_joint_states = p.getJointStates(robot_id, range(p.getNumJoints(robot_id)))
        current_joint_positions = [state[0] for state in current_joint_states]
        link_state = p.getLinkState(robot_id, link_index)
        current_orientation = link_state[1]
        joint_positions = p.calculateInverseKinematics(
            robot_id,
            link_index,
            targetPosition=target_xyz,
            targetOrientation=current_orientation,
            maxNumIterations=1000,
            residualThreshold=0.1
        )
    else:
        joint_positions = p.calculateInverseKinematics(
            robot_id,
            link_index,
            targetPosition=target_xyz,
            targetOrientation=target_orientation,
            maxNumIterations=1000,
            residualThreshold=0.1
        )
    num_controllable_joints = p.getNumJoints(robot_id)
    if joint_positions is not None and len(joint_positions) >= num_controllable_joints:
        return list(joint_positions[:num_controllable_joints])
    else:
        return None
def robot_gripper_open(
        robot_id: int,
    ) -> None:
    """
    控制機器人夾爪張開。
    Args:
        robot_id (int): 機器人模型在 PyBullet 中的唯一 ID。
    Returns:
        None
    """
    p.setJointMotorControl2(
        robot_id,
        ROBOT_GRIPPER_INDEXS[0],
        p.POSITION_CONTROL,
        targetPosition=ROBOT_GRIP_OPEN_RADIAN,
        force=ROBOT_GRIP_FORCE
    )
    p.setJointMotorControl2(
        robot_id,
        ROBOT_GRIPPER_INDEXS[1],
        p.POSITION_CONTROL,
        targetPosition=ROBOT_GRIP_OPEN_RADIAN,
        force=ROBOT_GRIP_FORCE
    )
    return None
def robot_gripper_close(
        robot_id: int,
    ) -> None:
    """
    控制機器人夾爪閉合。
    Args:
        robot_id (int): 機器人模型在 PyBullet 中的唯一 ID。
    Returns:
        None
    """
    p.setJointMotorControl2(
        robot_id,
        ROBOT_GRIPPER_INDEXS[0],
        p.POSITION_CONTROL,
        targetPosition=ROBOT_GRIP_CLOSE_RADIAN,
        force=ROBOT_GRIP_FORCE
    )
    p.setJointMotorControl2(
        robot_id,
        ROBOT_GRIPPER_INDEXS[1],
        p.POSITION_CONTROL,
        targetPosition=ROBOT_GRIP_CLOSE_RADIAN,
        force=ROBOT_GRIP_FORCE
    )
    return None
def calculate_camera_position(
        camera_id: int,
        target_offset: List[float] = CAMERA_WATCH_TARGET_POSITION,
    ) -> Tuple[List[float], List[float], List[int]]:
    """
    計算相機在世界坐標系中的實際位置、觀察目標點和上向量。
    Args:
        camera_id (int): 相機模型在 PyBullet 中的唯一 ID。
        target_offset (List[float]): 相機相對於自身位置的觀察目標偏移量 [x, y, z]。
    Returns:
        Tuple[List[float], List[float], List[int]]: 包含相機眼點位置、觀察目標位置和上向量的元組。
    """
    camera_position, camera_orientation = p.getBasePositionAndOrientation(camera_id)
    rotation_matrix = np.array(p.getMatrixFromQuaternion(camera_orientation)).reshape(3, 3)
    target_position = np.array(camera_position) + rotation_matrix @ np.array(target_offset)
    camera_up_vector = rotation_matrix @ np.array([0, 0, 1])
    return list(camera_position), target_position.tolist(), camera_up_vector.tolist()
def capture_camera_image(
        camera_position: List[float],
        target_position: List[float],
        up_vector: List[int] = [0, 0, 1]
    ) -> Tuple[np.ndarray, np.ndarray]:
    """
    從 PyBullet 虛擬相機捕獲 RGB 圖像和深度圖像。
    Args:
        camera_position (List[float]): 相機在世界坐標系中的位置 (眼點)。
        target_position (List[float]): 相機觀察的目標位置。
        up_vector (List[int]): 相機的上向量。
    Returns:
        Tuple[np.ndarray, np.ndarray]: 包含 RGB 圖像陣列和深度圖像陣列的元組。
    """
    view_matrix: List = p.computeViewMatrix(
        cameraEyePosition=camera_position,
        cameraTargetPosition=target_position,
        cameraUpVector=up_vector
    )
    projection_matrix: List = p.computeProjectionMatrixFOV(
        fov=CAMERA_FIELD_OF_VIEW,
        aspect=CAMERA_ASPECT_RATIO,
        nearVal=CAMERA_NEAR_PLANE,
        farVal=CAMERA_FAR_PLANE
    )
    width, height, rgb_img, depth_img, segmentation_mask = p.getCameraImage(
        width=CAMERA_IMAGE_WIDTH,
        height=CAMERA_IMAGE_HEIGHT,
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix,
        renderer=p.ER_BULLET_HARDWARE_OPENGL
    )
    rgb_array: np.ndarray = np.reshape(rgb_img, (height, width, 4))[:, :, :3]
    rgb_array = cv2.cvtColor(rgb_array, cv2.COLOR_RGB2BGR)
    depth_array: np.ndarray = np.reshape(depth_img, (height, width))
    return rgb_array, depth_array
def display_capture_image(
        image: np.ndarray,
        window_title_name: str = "Real Time Streams of Measurement"
    ) -> None:
    """
    使用 OpenCV 顯示捕獲的圖像。
    Args:
        image (np.ndarray): 要顯示的圖像陣列。
        window_title_name (str): 顯示窗口的標題名稱。
    Returns:
        None
    """
    cv2.imshow(window_title_name, image)
    cv2.waitKey(1)
def calculate_image_contour(
    image: np.ndarray,
    depth: float = 0.05,
    target_color_rgb: List[float] = [1, 1, 0],
    color_deviation: int = 30,
    min_contour_area: int = 100,
    max_contour_area: int = int(CAMERA_IMAGE_WIDTH * CAMERA_IMAGE_HEIGHT * 0.9)
) -> Tuple[np.ndarray, float, float]:
    """
    計算圖像中最大輪廓的實際寬度和高度（以公尺為單位），
    並返回一張同時帶有輪廓、邊界框和尺寸標註的 RGB 圖片。
    可選擇根據指定的顏色範圍創建遮罩來識別物體。
    Args:
        image (np.ndarray): 輸入圖像陣列。假定為 PyBullet 輸出，如果為 3 通道，則為 RGB 格式。
                            否則為灰度格式。
        depth (float): 物體相對於相機的深度（距離），用於將像素轉換為實際尺寸。
        threshold (int): 灰度二值化閾值。如果提供了 target_color_bgr，則此參數可能不使用。
        area_threshold (float): 過濾小面積輪廓的最小面積閾值。
        target_color_bgr (Optional[List[float]]): 目標物件的 BGR 顏色列表，例如黃色可能為 [0.0, 1.0, 1.0]。
                                                 值範圍為 0.0-1.0 的浮點數。
                                                 如果提供，函數將使用顏色分割來創建遮罩。
    Returns:
        Tuple[np.ndarray, float, float]: 包含以下元素的元組：
            - `annotated_image`: 帶有最大輪廓、邊界框和尺寸標註的 RGB 圖片。
            - `real_width_m`: 最大輪廓的實際寬度（公尺）。
            - `real_height_m`: 最大輪廓的實際高度（公尺）。
    """
    processed_image_bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    target_color_bgr = target_color_rgb
    annotated_image = processed_image_bgr.copy()
    lower_bound_bgr = np.array([
        max(0, target_color_bgr[0] - color_deviation),
        max(0, target_color_bgr[1] - color_deviation),
        max(0, target_color_bgr[2] - color_deviation)
    ], dtype=np.uint8)
    upper_bound_bgr = np.array([
        min(255, target_color_bgr[0] + color_deviation),
        min(255, target_color_bgr[1] + color_deviation),
        min(255, target_color_bgr[2] + color_deviation)
    ], dtype=np.uint8)
    if 0 <= target_color_rgb[0] <= 1 and 0 <= target_color_rgb[1] <= 1 and 0 <= target_color_rgb[2] <= 1:
        target_color_bgr_255 = [int(c * 255) for c in target_color_bgr]
        lower_b = max(0, target_color_bgr_255[0] - color_deviation)
        upper_b = min(255, target_color_bgr_255[0] + color_deviation)
        lower_g = max(0, target_color_bgr_255[1] - color_deviation)
        upper_g = min(255, target_color_bgr_255[1] + color_deviation)
        lower_r = max(0, target_color_bgr_255[2] - color_deviation)
        upper_r = min(255, target_color_bgr_255[2] + color_deviation)
        lower_bound_bgr = np.array([lower_b, lower_g, lower_r], dtype=np.uint8)
        upper_bound_bgr = np.array([upper_b, upper_g, upper_r], dtype=np.uint8)
    color_mask = cv2.inRange(processed_image_bgr, lower_bound_bgr, upper_bound_bgr)
    processed_binary_for_contours = np.ones_like(color_mask) * 0
    processed_binary_for_contours[color_mask > 0] = 255
    contours, _ = cv2.findContours(processed_binary_for_contours, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    largest_contour = None
    for idx, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if area > min_contour_area and area < max_contour_area:
            largest_contour = contour
    if largest_contour is None:
        annotated_image = cv2.cvtColor(annotated_image, cv2.COLOR_BGR2RGB)
        return annotated_image, 0.0, 0.0
    x, y, w_px, h_px = cv2.boundingRect(largest_contour)
    cv2.drawContours(annotated_image, [largest_contour], -1, (255, 0, 0), 2)
    fov_rad = np.deg2rad(CAMERA_FIELD_OF_VIEW)
    view_width_at_depth = 2 * depth * np.tan(fov_rad / 2)
    meter_per_pixel = view_width_at_depth / CAMERA_IMAGE_WIDTH
    real_width_m = w_px * meter_per_pixel
    real_height_m = h_px * meter_per_pixel
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.4
    font_thickness = 1
    text_color = (0, 255, 0)
    line_type = cv2.LINE_AA
    width_text = f"Width:  {real_width_m:.3f} m"
    height_text = f"Height: {real_height_m:.3f} m"
    text_w_pos = (10, 20)
    text_h_pos = (10, 40)
    cv2.putText(annotated_image, width_text, text_w_pos, font, font_scale, text_color, font_thickness, line_type)
    cv2.putText(annotated_image, height_text, text_h_pos, font, font_scale, text_color, font_thickness, line_type)
    annotated_image = cv2.cvtColor(annotated_image, cv2.COLOR_BGR2RGB)
    return annotated_image, real_width_m, real_height_m
def utility_clear_terminal_screen() -> None:
    """
    清除終端機螢幕。
    Returns:
        None
    """
    os.system("cls" if os.name == "nt" else "clear")
    sys.stdout.flush()
    return None
def utility_trajectory_smoothly(
        trajectory_data: List[List[float]],
        difference_step: int = 15
    ) -> List[List[float]]:
    """
    對給定的機器人軌跡數據進行平滑處理，在每個原始軌跡點之間插入中間點。
    Args:
        trajectory_data (List[List[float]]): 原始的機器人軌跡點列表，每個點是一個關節角度列表。
        difference_step (int): 在兩個相鄰原始軌跡點之間插入的中間點數量。
    Returns:
        List[List[float]]: 平滑後的軌跡點列表。
    """
    smoothed_trajectory = []
    for i in range(len(trajectory_data) - 1):
        start_joint_angles = np.array(trajectory_data[i])
        end_joint_angles = np.array(trajectory_data[i + 1])
        for step in range(difference_step):
            alpha = step / difference_step
            interpolated_angles = (1 - alpha) * start_joint_angles + alpha * end_joint_angles
            smoothed_trajectory.append(interpolated_angles.tolist())
    smoothed_trajectory.append(trajectory_data[-1])
    return smoothed_trajectory
def display_panel_input() -> None:
    """
    負責在單獨的執行緒中處理使用者從終端機的輸入，並將輸入放入佇列。
    """
    global mode_input_queue
    while (alive):
        try:
            user_input = input("請輸入模式選擇： ")
            mode_input_queue.put(user_input)
        except Exception as e:
            break
        time.sleep(0.5)
    return None
def display_manual_xyz_input(robot_id: int, command_string: str) -> bool:
    """
    處理手動模式下輸入的 XYZ 座標指令。
    解析座標，計算逆運動學，並更新機器人軌跡。
    Args:
        robot_id (int): 機器人模型在 PyBullet 中的唯一 ID。
        command_string (str): 包含 XYZ 座標的字串，格式為 "(x, y, z)"。
    Returns:
        bool: 如果成功處理指令並需要刷新顯示，則返回 True；否則返回 False。
    """
    global trajectory, current_trajectory_accumulate
    try:
        coords_str = command_string[1:-1].split(',')
        if len(coords_str) == 3:
            target_x = float(coords_str[0].strip())
            target_y = float(coords_str[1].strip())
            target_z = float(coords_str[2].strip())
            if robot_id is not None:
                target_joint_angles = robot_move_to_xyz(robot_id, [target_x, target_y, target_z])
                if target_joint_angles:
                    with trajectory_lock:
                        trajectory.clear()
                        trajectory.append(target_joint_angles)
                        current_trajectory_accumulate = 0
                    print(f"\n[INFO]: 設定機器人手臂移動至座標: ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})", end="")
                    sys.stdout.flush()
                    return True
                else:
                    print(f"\n[ERROR]: 無法計算到達座標 ({target_x:.3f}, {target_y:.3f}, {target_z:.3f}) 的逆運動學。", end="")
                    sys.stdout.flush()
                    return True
            else:
                print(f"\n[ERROR]: 機器人模型尚未加載。", end="")
                sys.stdout.flush()
                return True
        else:
            print(f"\n[ERROR]: 無效的座標格式。請輸入 <(x, y, z)>。", end="")
            sys.stdout.flush()
            return True
    except ValueError:
        print(f"\n[ERROR]: 無效的輸入。請確認座標格式正確。", end="")
        sys.stdout.flush()
        return True
def display_panel(robot_id: int) -> None:
    """
    負責在單獨的執行緒中管理和顯示終端機使用者介面。
    根據當前的模式 (mode) 和使用者輸入來刷新顯示內容。
    Args:
        robot_id (int): 機器人模型在 PyBullet 中的唯一 ID。
    """
    global alive, mode, move, trajectory, current_trajectory_accumulate
    last_displayed_mode = ""
    try:
        while alive:
            if mode == "init":
                mode = "menu"
                display_menu_mode()
                last_displayed_mode = "menu"
                continue
            mode_input: str = ""
            try:
                mode_input = mode_input_queue.get_nowait()
            except queue.Empty:
                mode_input = ""
            should_refresh = False
            if mode_input:
                should_refresh = True
            if mode_input.lower() == 'r':
                should_refresh = True
                mode_input = ""
            current_display_func = None
            if mode == "menu":
                if mode_input == '1':
                    mode = "monitor"
                    should_refresh = True
                elif mode_input == '2':
                    mode = "manual"
                    should_refresh = True
                elif mode_input == '3':
                    alive = False
                    mode = "init"
                    should_refresh = True
                if not should_refresh and not move and last_displayed_mode != "menu":
                    should_refresh = True
                current_display_func = display_menu_mode
            elif mode == "monitor":
                if mode_input == 'M' or mode_input == 'm':
                    mode = "menu"
                    should_refresh = True
                if not should_refresh and (move or last_displayed_mode != "monitor"):
                    should_refresh = True
                current_display_func = display_monitor_mode
            elif mode == "manual_help":
                if mode_input == 'M' or mode_input == 'm':
                    mode = "menu"
                    should_refresh = True
                elif mode_input == 'Q' or mode_input == 'q':
                    mode = "manual"
                    should_refresh = True
                if not should_refresh and last_displayed_mode != "manual_help":
                    should_refresh = True
                current_display_func = display_manual_help_mode
            elif mode == "manual":
                if mode_input == 'M' or mode_input == 'm':
                    mode = "menu"
                    should_refresh = True
                elif mode_input == 'H' or mode_input == 'h':
                    mode = "manual_help"
                    should_refresh = True
                elif mode_input != "":
                    if mode_input.startswith('(') and mode_input.endswith(')'):
                        should_refresh = display_manual_xyz_input(robot_id, mode_input)
                    else:
                        try:
                            parts = mode_input.split(" ")
                            if len(parts) == 2:
                                idx = int(parts[0])
                                rad = float(parts[1])
                                with current_robot_joints_radian_lock:
                                    rads = current_robot_joints_radian.copy()
                                if 0 <= idx < len(rads):
                                    rads[idx] = rad
                                    with trajectory_lock:
                                        trajectory.clear()
                                        trajectory.append(rads)
                                        current_trajectory_accumulate = 0
                                    should_refresh = True
                                else:
                                    print(f"\n[ERROR]: 關節索引 {idx} 超出範圍。", end="")
                                    sys.stdout.flush()
                                    should_refresh = True
                            else:
                                print(f"\n[ERROR]: 無效的指令格式。請輸入 <關節索引 弧度> 或 <(x, y, z)>。", end="")
                                sys.stdout.flush()
                                should_refresh = True
                        except ValueError:
                            print(f"\n[ERROR]: 無效的輸入。請確認關節索引和弧度為數字，或座標格式正確。", end="")
                            sys.stdout.flush()
                            should_refresh = True
                if not should_refresh and (move or last_displayed_mode != "manual"):
                    should_refresh = True
                current_display_func = display_manual_mode
            if should_refresh and current_display_func:
                current_display_func()
                last_displayed_mode = mode
            time.sleep(0.5)
    except Exception as e:
        print(f"\n[ERROR]:\t關閉輸入顯示執行緒: {e}")
    return None
def display_system_mode() -> None:
    """
    顯示系統的通用狀態訊息，包括運行狀態、模式、機器人移動狀態等。
    """
    utility_clear_terminal_screen()
    if alive:
        print(f"[SYSTEM\tINFO]:\tsystem alive...")
    else:
        print(f"[SYSTEM\tINFO]:\tsystem shutdown...")
    print(f"[SYSTEM\tINFO]:\tprocess on path\t\t[{state}]")
    print(f"[SYSTEM\tINFO]:\tterminal mode is\t[{mode}]")
    print(f"[ROBOT\tINFO]:\trobot move to\t\t[ ({current_trajectory_accumulate}) / ({len(trajectory) - 1}) points]")
    if move and can_move_next:
        print(f"[ROBOT\tINFO]:\tis moving...")
    else:
        print(f"[ROBOT\tINFO]:\tis stationary...")
    print(f"[ROBOT\tINFO]:\twatch vector slope on\t{slope}")
    print(f"[CAMERA\tINFO]:\twatch from\t\t[{VIEWS[view]['name']}]")
    print()
    return None
def display_monitor_mode() -> None:
    """
    在終端機顯示監控模式的介面，包括機器人關節弧度、工件測量數據等。
    """
    with current_robot_joints_radian_lock:
        rad = current_robot_joints_radian.copy()
    display_system_mode()
    print("------------------------------------------- 監控模式 -------------------------------------------")
    print("----------機器手臂 關節弧度---------      ------待測工件------          ------預設工件------")
    print(f"[01]: {rad[1]:6.3f} rad     [09]: {rad[9]:6.3f} rad     測量長度： 0.080 m           預設長度： 0.090 m", flush=True)
    print(f"[02]: {rad[2]:6.3f} rad     [12]: {rad[12]:6.3f} rad     測量寬度： 0.080 m           預設寬度： 0.090 m", flush=True)
    print(f"[03]: {rad[3]:6.3f} rad     [10]: {rad[10]:6.3f} rad     測量高度： 0.030 m           預設高度： 0.050 m", flush=True)
    print(f"[05]: {rad[5]:6.3f} rad     [13]: {rad[13]:6.3f} rad     測量狀態： OK                預設狀態： OK", flush=True)
    print(f"\t\t\t\t\t\t\t\t       預設座標： [{GOOD_OBJECT_POSITION[0]:.3f}, {GOOD_OBJECT_POSITION[1]:.3f}, {GOOD_OBJECT_POSITION[2]:.3f}]", flush=True)
    print("---------[2]---------         ---------[3]---------")
    print("(+XY)\t\t\t      (-XZ)")
    print(f"正確寬度 X：  [{GOOD_OBJECT_SIZE[0]:.2f}] m        正確寬度 X：  [{GOOD_OBJECT_SIZE[0]:.2f}] m")
    print(f"正確高度 Y：  [{GOOD_OBJECT_SIZE[1]:.2f}] m        正確高度 Z：  [{GOOD_OBJECT_SIZE[2]:.2f}] m")
    print(f"測量高度 X：   {plus_xy_length[0]:.3f} m       測量高度 X：   {minus_yz_length[0]:.3f} m")
    print(f"測量高度 Y：   {plus_xy_length[1]:.3f} m       測量高度 Z：   {minus_yz_length[1]:.3f} m")
    print(f"測量正交距離： {plus_xy_length[2]:.3f} m       測量正交距離： {minus_yz_length[2]:.3f} m")
    print()
    print("預設執行路徑：\t[1]初始化 -> [2]觀測工件+XY面 -> [3]觀測工件-XZ面 -> [4]觀測工件YZ面(PASS) -> [5]移動到合適的平台")
    print()
    print(f"[目前進度]：\t正在進行 [{state}] 階段")
    if alive:
        print("[目前狀態]：\t觀測正常")
    else:
        print("[目前狀態]：\t觀測異常")
    print()
    print("按下 [M] 開啟模式選單")
    print("按下 [R] 刷新終端機")
    return None
def display_manual_mode() -> None:
    """
    在終端機顯示手動模式的介面，主要顯示機器人所有關節的當前弧度。
    """
    with current_robot_joints_radian_lock:
        rad = current_robot_joints_radian.copy()
    display_system_mode()
    print("------------------------------- 手動模式 -------------------------------")
    print("---------------------------機器手臂  關節弧度---------------------------")
    for i in range(0, 14, 4):
        row = []
        for j in range(i, min(i + 4, 14)):
            row.append(f"[{j:02d}]: {rad[j]:6.3f} rad")
        print("    ".join(row))
    print()
    print("按下 [M] 開啟模式選單")
    print("按下 [H] 開啟手動選單")
    print("按下 [R] 刷新終端機")
    return None
def display_manual_help_mode() -> None:
    """
    在終端機顯示手動模式的幫助介面，說明各種控制指令。
    """
    display_system_mode()
    print("------------------------------- 手動模式 -------------------------------")
    print("---------------------------機器手臂  指令說明---------------------------")
    print("\nF1: 給座標，讓機器手臂自己計算到該座標的弧度。")
    print("C1: 給定<(x, y, z)>，代入你要的目標座標值。")
    print("---------")
    print("(0.1, 0.2, 0.3)")
    print("---------\n")
    print("F2: 給指定關節與弧度，讓機器手臂依據移動。")
    print("C2: 給定<指定關節 弧度>")
    print("----")
    print("5 1.0")
    print("----\n")
    print("[提示]：請在 [手動選單] 底下 [請輸入模式選擇] 輸入 <5 1.0> or <(0.1, 0.2, 0.3)>，就可以移動了。")
    print("[提示]：建議在 [手動選單] -> [機器手臂 關節弧度] 頁面操作")
    print()
    print("按下 [M] 開啟模式選單")
    print("按下 [Q] 返回上一頁")
    print("按下 [R] 刷新終端機")
    return None
def display_menu_mode() -> None:
    """
    在終端機顯示主選單模式的介面，提供不同的操作選項。
    """
    display_system_mode()
    print("----------選單模式----------")
    print("(1) monitor\t監控數據模式")
    print("(2) manual\t手動操作模式")
    print("(3) exit\t結束主程式")
    print("---------------------------")
    print()
    return None
def main() -> None:
    """
    程式的主入口點。
    初始化模擬環境、加載物體和機器人、管理多執行緒，並執行主模擬迴圈。
    處理機器人移動、相機圖像捕獲和使用者交互。
    """
    global alive, state, slope, view, move, trajectory, current_robot_joints_radian, current_trajectory_accumulate, plus_xy_length, minus_yz_length, can_move_next
    initialize_environment_simulation(True)
    set_pybullet_camera_view(0)
    base_plane = load_box(
        BASE_PLANE_SIZE,
        BASE_PLANE_POSITION,
        BASE_PLANE_COLOR,
        BASE_PLANE_MASS,
        BASE_PLANE_FRICTION_LATERAL,
        BASE_PLANE_FRICTION_SPINNING,
        BASE_PLANE_FRICTION_ROLLING
    )
    good_plane = load_box(
        GOOD_PLANE_SIZE,
        GOOD_PLANE_POSITION,
        GOOD_PLANE_COLOR,
        GOOD_PLANE_MASS,
        GOOD_PLANE_FRICTION_LATERAL,
        GOOD_PLANE_FRICTION_SPINNING,
        GOOD_PLANE_FRICTION_ROLLING
    )
    poor_plane = load_box(
        POOR_PLANE_SIZE,
        POOR_PLANE_POSITION,
        POOR_PLANE_COLOR,
        POOR_PLANE_MASS,
        POOR_PLANE_FRICTION_LATERAL,
        POOR_PLANE_FRICTION_SPINNING,
        POOR_PLANE_FRICTION_ROLLING
    )
    robot_id = load_robot(
        ROBOT_JOINTS_DEFAULT_RADIAN,
        gripper_friction_lateral=0.5
    )
    camera = load_box(
        CAMERA_SIZE,
        CAMERA_POSITION,
        CAMERA_COLOR,
        CAMERA_MASS
    )
    camera = load_camera(robot_id, camera)
    good_object = load_box(
        GOOD_OBJECT_SIZE,
        GOOD_OBJECT_POSITION,
        GOOD_OBJECT_COLOR,
        GOOD_OBJECT_MASS,
        GOOD_OBJECT_FRICTION_LATERAL,
        GOOD_OBJECT_FRICTION_SPINNING,
        GOOD_OBJECT_FRICTION_ROLLING
    )
    trajectory = utility_trajectory_smoothly(trajectory, trajectory_smooth_step)
    utility_clear_terminal_screen()
    terminal_input = threading.Thread(target=display_panel_input, daemon=True)
    terminal_output = threading.Thread(target=display_panel, args=(robot_id,), daemon=True)
    terminal_input.start()
    terminal_output.start()
    print(f"[INFO]: {INIT_WAIT_TIME}s 時間選擇模式")
    time.sleep(INIT_WAIT_TIME)
    stable_frame_counter = 0
    stable_capture_frame_counter = 0
    cache_x = -1000.0
    cache_y = -1000.0
    while alive:
        object_position, object_orientation = p.getBasePositionAndOrientation(good_object)
        camera_position, camera_orientation = p.getBasePositionAndOrientation(camera)
        with current_robot_joints_radian_lock:
            for i in range(p.getNumJoints(robot_id)):
                current_robot_joints_radian[i] = p.getJointState(robot_id, i)[0]
        robot_current_pos = [p.getJointState(robot_id, i)[0] for i in range(p.getNumJoints(robot_id))]
        camera_eye_position, camera_target_point, camera_up_vector = calculate_camera_position(camera)
        slope = [round(camera_target_point[0] - camera_eye_position[0], 5), round(camera_target_point[1] - camera_eye_position[1], 5), round(camera_target_point[2] - camera_eye_position[2], 5)]
        p.addUserDebugLine(camera_eye_position, camera_target_point, [0, 1, 0], lineWidth=2.0, lifeTime=0)
        view_direction = np.array(camera_target_point) - np.array(camera_eye_position)
        view_direction = view_direction / (np.linalg.norm(view_direction) + 1e-6)
        if abs(np.dot(view_direction, camera_up_vector)) > 0.99:
            camera_up_vector = [0, 0, 1]
        else:
            camera_up_vector = [0, 0, -1]
        rgb_image_array, _ = capture_camera_image(camera_eye_position, camera_target_point, camera_up_vector)
        display_capture_image(rgb_image_array)
        if can_move_next == False:
            move = False
        if (mode == "monitor" or mode == "manual") and can_move_next == True:
            if current_trajectory_accumulate < len(trajectory):
                robot_target_pos = trajectory[current_trajectory_accumulate]
                robot_current_pos = robot_move_smoothly(robot_id, robot_current_pos, robot_target_pos, ROBOT_STABLE_STEP)
                if np.allclose(robot_current_pos, robot_target_pos, atol=0.001):
                    stable_frame_counter += 1
                    if stable_frame_counter >= ROBOT_STABLE_FRAME_WAIT:
                        current_trajectory_accumulate += 1
                        stable_frame_counter = 0
                        if current_trajectory_accumulate >= len(trajectory):
                            move = False
                        else:
                            move = True
                    else:
                        move = True
                else:
                    stable_frame_counter = 0
                    move = True
            elif len(trajectory) > 0 and current_trajectory_accumulate >= len(trajectory):
                move = False
        else:
            move = False
        if np.allclose(robot_current_pos, PXY_POINT, atol=0.1) and state < 2:
            object_x = object_position[0]
            object_y = object_position[1]
            object_z = object_position[2]
            camera_x = camera_eye_position[0]
            camera_y = camera_eye_position[1]
            camera_z = camera_eye_position[2] - 0.015
            p.addUserDebugLine([camera_x, camera_y, camera_z], [object_x, object_y, object_z], [1, 0, 0], lineWidth=2.0, lifeTime=0)
            distance = math.sqrt((math.pow(object_x - camera_x, 2) + math.pow(object_y - camera_y, 2) + math.pow(object_z - camera_z, 2)))
            plus_xy_length[2] = distance
            image, plus_xy_length[0], plus_xy_length[1] = calculate_image_contour(rgb_image_array, distance, target_color_rgb = [227, 227, 0], color_deviation = 30)
            if 0.088 <= plus_xy_length[0] <= 0.092 and 0.088 <= plus_xy_length[1] <= 0.092:
                can_move_next = False
                if cache_x != -1000.0 and cache_y != -1000.0:
                    if np.allclose(plus_xy_length[0], cache_x, atol=0.001) and np.allclose(plus_xy_length[1], cache_y, atol=0.001):
                        stable_capture_frame_counter += 1
                    else:
                        stable_capture_frame_counter = 0
                else:
                    cache_x = plus_xy_length[0]
                    cache_y = plus_xy_length[1]
                cache_x = plus_xy_length[0]
                cache_y = plus_xy_length[1]
            if stable_capture_frame_counter >= CAMERA_CAPTURE_STABLE_FRAME_WAIT:
                stable_capture_frame_counter = 0
                cache_x = -1000.0
                cache_y = -1000.0
                state = 2
                can_move_next = True
            display_capture_image(image, "calculate_image_contour()")
        elif np.allclose(robot_current_pos, PYZ_POINT, atol=0.001) and state < 3:
            object_x = object_position[0]
            object_y = object_position[1] - 0.045
            object_z = object_position[2]
            camera_x = camera_eye_position[0]
            camera_y = camera_eye_position[1] + 0.015
            camera_z = camera_eye_position[2]
            p.addUserDebugLine([camera_x, camera_y, camera_z], [object_x, object_y, object_z], [1, 0, 0], lineWidth=2.0, lifeTime=0)
            distance = math.sqrt((math.pow(object_x - camera_x, 2) + math.pow(object_y - camera_y, 2) + math.pow(object_z - camera_z, 2)))
            minus_yz_length[2] = distance
            image, minus_yz_length[0], minus_yz_length[1] = calculate_image_contour(rgb_image_array, distance, target_color_rgb = [164, 164, 0], color_deviation = 30)
            if 0.075 <= minus_yz_length[0] <= 0.095 and 0.025 <= minus_yz_length[1] <= 0.035:
                can_move_next = False
                if cache_x != -1000.0 and cache_y != -1000.0:
                    if np.allclose(minus_yz_length[0], cache_x, atol=0.001) and np.allclose(minus_yz_length[1], cache_y, atol=0.001):
                        stable_capture_frame_counter += 1
                    else:
                        stable_capture_frame_counter = 0
                else:
                    cache_x = minus_yz_length[0]
                    cache_y = minus_yz_length[1]
                cache_x = minus_yz_length[0]
                cache_y = minus_yz_length[1]
            if stable_capture_frame_counter >= CAMERA_CAPTURE_STABLE_FRAME_WAIT:
                stable_capture_frame_counter = 0
                cache_x = -1000.0
                cache_y = -1000.0
                state = 3
                can_move_next = True
            display_capture_image(image, "calculate_image_contour()")
        keys = p.getKeyboardEvents()
        if ord('i') in keys and keys[ord('i')] & p.KEY_WAS_TRIGGERED:
            view = (view + 1) % 6
            set_pybullet_camera_view(view)
        p.removeAllUserDebugItems()
        p.stepSimulation()
        time.sleep(UPDATE)
if __name__ == "__main__":
    main()
