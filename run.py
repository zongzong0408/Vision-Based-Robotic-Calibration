from typing import Optional, List, Tuple
import pybullet as p
import pybullet_data
import numpy as np
import threading
import queue
import time
import sys
import cv2
import os

GRAVITY     = -10.0
UPDATE      = 1.0 / 1000.0
VIEWS       = [
    {"name": "Default (XYZ)",    "yaw": 45,     "pitch": -30,   "distance": 1.5},
    {"name": "Top-down (XY)",    "yaw": 90,     "pitch": -90,   "distance": 1.5},
    {"name": "Front (XZ)",       "yaw": 90,     "pitch": 0,     "distance": 1.5},
    {"name": "Side (YZ)",        "yaw": 0,      "pitch": 0,     "distance": 1.5},
    {"name": "Back (-XZ)",       "yaw": -90,    "pitch": 0,     "distance": 1.5},
    {"name": "Left (-YZ)",       "yaw": 180,    "pitch": 0,     "distance": 1.5}
]

base_plane_size = [0.5, 0.5, 0.05]
base_plane_mass = 0
base_plane_friction_lateral = 0.5
base_plane_friction_spinning = 0.165
base_plane_friction_rolling = 0.025
base_plane_position = [0.0, 0.0, 0.0]
base_plane_color = [1, 1, 1, 0.9]

good_plane_size = [0.5, 0.5, 0.05]
good_plane_mass = 0
good_plane_friction_lateral = 0.5
good_plane_friction_spinning = 0.16
good_plane_friction_rolling = 0.025
good_plane_position = [0.5, 0, 0]
good_plane_color = [0, 1, 0, 1]

poor_plane_size = [0.5, 0.5, 0.05]
poor_plane_mass = 0
poor_plane_friction_lateral = 0.5
poor_plane_friction_spinning = 0.16
poor_plane_friction_rolling = 0.025
poor_plane_position = [-0.5, 0, 0]
poor_plane_color = [1, 0, 0, 1]

good_object_size = [0.09, 0.09, 0.03]
good_object_mass = 0.3
good_object_friction_lateral = 5.0
good_object_friction_spinning = 1.0
good_object_friction_rolling = 0.05
good_object_position = [0, 0.25, 0.07]
good_object_color = [1, 1, 0, 1]

poor_object_size = [1.00, 0.09, 0.03]
poor_object_mass = 0.3
poor_object_friction_lateral = 5.0
poor_object_friction_spinning = 1.0
poor_object_friction_rolling = 0.05
poor_object_position = [0, 0.25, 0.07]
poor_object_color = [1, 1, 0, 1]

robot_urdf_path	= "xarm/xarm6_with_gripper.urdf"
robot_idle_position	= [0.0, 0.5, 0.0]
robot_joints_default_radian = [0, 0, -2, 0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0, 0]
robot_gripper_indexs = [9, 12]
robot_grip_open_radian = 0.85
robot_grip_close_radian = 0.0
robot_stable_step = 15
robot_joint_force = 250
robot_grip_force = 2000
robot_stable_frame_wait = 5

camera_size = [0.03, 0.07, 0.03]
camera_mass = 0.01
camera_index = 0
camera_link_robot_index = 6
camera_image_width = 640
camera_image_height = 480
camera_field_of_view = 60
camera_aspect_ratio = 1.0
camera_near_plane = 0.01
camera_far_plane = 5.0
camera_position = [0.0, 0.0, 0.0]
camera_offset_position = [0.05, 0.00, 0.10]
camera_watch_target_position = [0.0, 0.0, 0.2]
camera_color = [0, 0, 0, 1]

trajectory = [
    [0.000, -1.570, -0.100, -1.200, 0.000, 1.300, 0.000, 0.000, 0.000, robot_grip_open_radian, 0.000, 0.000, robot_grip_open_radian, 0.00]
]

alive   = True
mode    = "init"
view    = 0
state   = 1
moving  = False
time_for_init_wait = 3

# mode_input_queue_lock = threading.Lock()
mode_input_queue = queue.Queue()
# current_object_position = [0, 0.25, 0.07]
current_robot_joints_radian_lock = threading.Lock()
current_robot_joints_radian = robot_joints_default_radian.copy()

current_trajectory_accumulate = 0

def initialize_environment_simulation(using_GUI: bool = True) -> None:
    
    if using_GUI:
        p.connect(p.GUI)
    else:
        p.connect(p.DIRECT)
    p.setGravity(0, 0, GRAVITY)
    p.setTimeStep(UPDATE)
    
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
    
    p.addUserDebugLine([0, 0, 0], [100, 0, 0], [1, 0, 0], lineWidth = 1)
    p.addUserDebugLine([0, 0, 0], [0, 100, 0], [0, 1, 0], lineWidth = 1)
    p.addUserDebugLine([0, 0, 0], [0, 0, 100], [0, 0, 1], lineWidth = 1)
    
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    p.loadURDF("plane.urdf")
    
    return None

def set_pybullet_camera_view(view_index: int, target: List[float] = [0, 0, 0.1]) -> None:
    
    global view
    cam = VIEWS[view_index]
    
    p.resetDebugVisualizerCamera(
        cameraDistance          = cam["distance"],
        cameraYaw               = cam["yaw"],
        cameraPitch             = cam["pitch"],
        cameraTargetPosition    = target
    )
    view = view_index
    
    return None

def load_box(
        size:               List[float],
        position:           List[float],
        color:              List[int],
        mass:               float = 0.0,
        friction_lateral:   float = 0.0,
        friction_spinning:  float = 0.0,
        friction_rolling:   float = 0.0
    ) -> int:
    
    visual_shape_id = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents = [l / 2 for l in size],
        rgbaColor   = color
    )
    collision_shape_id = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents = [l / 2 for l in size]
    )
    
    box_id = p.createMultiBody(mass, collision_shape_id, visual_shape_id, position)

    p.changeDynamics(
        box_id,
        -1,
        lateralFriction     = friction_lateral,
        spinningFriction    = friction_spinning,
        rollingFriction     = friction_rolling
    )

    return box_id

def load_robot(
        initial_joint_radian: List[float]   = robot_joints_default_radian, 
        gripper_friction_lateral: float     = 0.5,
        gripper_friction_spinning: float    = 0.165,
        gripper_friction_rolling: float     = 0.025
    ) -> int:
    
    robot_id = p.loadURDF(robot_urdf_path, robot_idle_position, useFixedBase = True)
    actual_joints_count = p.getNumJoints(robot_id)

    if len(initial_joint_radian) != actual_joints_count:
        raise ValueError(
            f"[ERROR]:\t您提供的 initial_joint_radian(list) 長度 ({len(initial_joint_radian)}) "
            f"[ERROR]:\t與機器人預期關節數量 ({actual_joints_count}) 不符。"
            f"[ERROR]:\t請確保列表長度正確。"
        )
    
    for index, rad in enumerate(initial_joint_radian):
        p.resetJointState(robot_id, index, targetValue = rad)
        
    p.changeDynamics(robot_id, robot_gripper_indexs[0],
                        lateralFriction     = gripper_friction_lateral,
                        spinningFriction    = gripper_friction_spinning,
                        rollingFriction     = gripper_friction_rolling
                    )
    p.changeDynamics(robot_id, robot_gripper_indexs[1],
                        lateralFriction     = gripper_friction_lateral,
                        spinningFriction    = gripper_friction_spinning,
                        rollingFriction     = gripper_friction_rolling
                    )

    return robot_id

def load_camera(
        robot_id:   int,
        camera_id:  int,
    ) -> int:
    
    link_state = p.getLinkState(robot_id, camera_link_robot_index, computeForwardKinematics = True)
    link_position = link_state[0]
    link_orientation = link_state[1]
    
    rotation_matrix = np.array(p.getMatrixFromQuaternion(link_orientation)).reshape(3, 3)

    camera_world_offset = rotation_matrix @ np.array(camera_offset_position)
    camera_world_position = np.array(link_position) + camera_world_offset

    p.resetBasePositionAndOrientation(camera_id, camera_world_position.tolist(), link_orientation)

    p.createConstraint(
        parentBodyUniqueId      = robot_id,
        parentLinkIndex         = camera_link_robot_index,
        childBodyUniqueId       = camera_id,
        childLinkIndex          = -1,
        jointType               = p.JOINT_FIXED,
        jointAxis               = [0, 0, 0],
        parentFramePosition     = camera_offset_position,
        childFramePosition      = [0, 0, 0]
    )
    
    return camera_id

def robot_move(
        robot_id:   int,
        target:     List[float],
    ) -> None:
    
    global moving
    
    num_joints = p.getNumJoints(robot_id)
    
    for index in range(min(len(target), num_joints)):
        p.setJointMotorControl2(
            bodyUniqueId        = robot_id,
            jointIndex          = index,
            controlMode         = p.POSITION_CONTROL,
            targetPosition      = target[index],
            force               = robot_joint_force
        )
        
    moving = True
        
    return None

def robot_move_smoothly(
        robot_id:                       int,
        current:                        List[float], 
        target:                         List[float],
        max_difference_radian_step:     float,
    ) -> List[float]:

    updated_positions: List[float] = []
    
    for current_rad, target_rad in zip(current, target):
        delta = target_rad - current_rad
        
        if abs(delta) < max_difference_radian_step:
            updated_positions.append(target_rad)
        
        else:
            updated_positions.append(current_rad + max_difference_radian_step * np.sign(delta))
    
    robot_move(robot_id, updated_positions)
    
    return updated_positions

def robot_gripper_open(
        robot_id:   int,
    ) -> None: 

    p.setJointMotorControl2(
            robot_id, 
            robot_gripper_indexs[0], 
            p.POSITION_CONTROL, 
            targetPosition = robot_grip_open_radian, 
            force = robot_grip_force
        )
    p.setJointMotorControl2(
            robot_id, 
            robot_gripper_indexs[1], 
            p.POSITION_CONTROL, 
            targetPosition = robot_grip_open_radian, 
            force = robot_grip_force
        )
    
    return None

def robot_gripper_close(
        robot_id:   int,
    ) -> None: 

    p.setJointMotorControl2(
            robot_id, 
            robot_gripper_indexs[0], 
            p.POSITION_CONTROL, 
            targetPosition = robot_grip_close_radian, 
            force = robot_grip_force
        )
    p.setJointMotorControl2(
            robot_id, 
            robot_gripper_indexs[1], 
            p.POSITION_CONTROL, 
            targetPosition = robot_grip_close_radian, 
            force = robot_grip_force
        )
    
    return None

def calculate_camera_position(
        camera_id:      int,
        target_offset:  List[float] = camera_watch_target_position,
    ) -> Tuple[List[float], List[float], List[int]]:
    
    camera_position, camera_orientation = p.getBasePositionAndOrientation(camera_id)
    
    rotation_matrix = np.array(p.getMatrixFromQuaternion(camera_orientation)).reshape(3, 3)

    target_position = np.array(camera_position) + rotation_matrix @ np.array(target_offset)
    camera_up_vector = rotation_matrix @ np.array([0, 0, 1])

    return list(camera_position), target_position.tolist(), camera_up_vector.tolist()

def capture_camera_image(
        camera_position:    List[float],
        target_position:    List[float],
        up_vector:          List[int] = [0, 0, 1]
    ) -> Tuple[np.ndarray, np.ndarray]:

    view_matrix: List = p.computeViewMatrix(
        cameraEyePosition       = camera_position,
        cameraTargetPosition    = target_position,
        cameraUpVector          = up_vector
    )

    projection_matrix: List = p.computeProjectionMatrixFOV(
        fov     = camera_field_of_view,
        aspect  = camera_aspect_ratio,
        nearVal = camera_near_plane,
        farVal  = camera_far_plane
    )

    width, height, rgb_img, depth_img, segmentation_mask = p.getCameraImage(
        width            = camera_image_width,
        height           = camera_image_height,
        viewMatrix       = view_matrix,
        projectionMatrix = projection_matrix,
        renderer         = p.ER_BULLET_HARDWARE_OPENGL
    )

    rgb_array: np.ndarray = np.reshape(rgb_img, (height, width, 4))[:, :, :3]
    
    depth_array: np.ndarray = np.reshape(depth_img, (height, width))

    return rgb_array, depth_array

def display_capture_image(
        image:              np.ndarray,
        window_title_name:  str = "Real Time Streams of Measurement"
    ) -> None:

    cv2.imshow(window_title_name, image)
    cv2.waitKey(1)
    
def calculate_image_contour(
        image: np.ndarray,
        depth: float            = 0.05,
        threshold: int          = 127,
        area_threshold: float   = 100.0
    ) -> Tuple[float, float]:
    
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image.copy()

    _, binary = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    largest_contour = None
    max_area = 0

    for contour in contours:
        area = cv2.contourArea(contour)
        if area > area_threshold and area > max_area:
            largest_contour = contour
            max_area = area

    if largest_contour is None:
        return 0.0, 0.0

    _, _, w_px, h_px = cv2.boundingRect(largest_contour)

    fov_rad = np.deg2rad(camera_field_of_view)
    width_m = 2 * depth * np.tan(fov_rad / 2)
    pixel_per_meter = camera_image_width / width_m
    meter_per_pixel = 1 / pixel_per_meter

    real_width_m = w_px * meter_per_pixel
    real_height_m = h_px * (width_m / camera_image_width) * (camera_image_height / camera_image_width)

    return real_width_m, real_height_m

def terminal_clear_screen() -> None:
    
    os.system("cls" if os.name == "nt" else "clear")

    return None

def trajectory_smoothly(
        trajectory_data:    List[List[float]],
        difference_step:    int = 15
    ) -> List[List[float]]:
    
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
    
    global mode_input_queue
    
    while alive:
        try:
            user_input = input()
            mode_input_queue.put(user_input)
        
        except Exception as e:
            print("[ERROR]: 無法讀取輸入")
            break
        
        time.sleep(0.08)
        
    return None

def display_panel() -> None:
    
    global alive, mode
    
    try:
        while alive:
            
            if mode == "init":
                # mode = "menu"
                display_menu_mode()
            
                continue
            
            mode_input = "init"
            try:
                mode_input = mode_input_queue.get_nowait()
            except queue.Empty:
                mode_input = None
            
            if mode == "menu":
                
                if mode_input == '1':
                    mode = "monitor"
                    display_monitor_mode()
                
                elif mode_input == '2':
                    mode = "manual"
                    display_manual_mode()
                
                elif mode_input == '3':
                    alive = False
                    mode = "init"
                    display_system_mode()
                
                else:
                    display_menu_mode()
                    
            elif mode == "monitor":
                
                if mode_input == 'M':
                    mode = "menu"
                    display_menu_mode()
                
                else:
                    display_monitor_mode()
            
            elif mode == "manual_help":
                
                if mode_input == 'M':
                    mode = "menu"
                    display_menu_mode()
                    
                elif mode_input == 'Q':
                    mode = "manual"
                    display_manual_mode()
                
                else:
                    display_manual_help_mode()
                    
            elif mode == "manual":
                
                if mode_input == 'M':
                    mode = "menu"
                    display_menu_mode()
                
                elif mode_input == 'H':
                    mode = "manual_help"
                    display_manual_help_mode()
                
                else:
                    display_manual_mode()
            
            time.sleep(0.01)
    
    except Exception as e:   
        
        print("[ERROR]:\t關閉輸入")
        
    return None  

def display_system_mode() -> None:
    
    if moving == True or mode == "init":
        terminal_clear_screen()
    print(f"[INFO]:\tsystem running")
    print(f"[INFO]:\trobot state [{state}]")
    print(f"[INFO]:\tcamera view [{view}]")
    print(f"[INFO]:\tterminal interface [{mode}]")
    print()
    
    return None

def display_monitor_mode() -> None:
    global test
    with current_robot_joints_radian_lock:        
        rad = current_robot_joints_radian.copy()
    
    display_system_mode()
    print("------------------------------------------- 監控模式 -------------------------------------------")
    print("----------機器手臂 關節弧度---------     ------待測工件------                ------預設工件------")
    print(f"[01]: {rad[1]:6.3f} rad    [09]: {rad[9]:6.3f} rad     測量長度： 0.080 m                  預設長度： 0.090 m")
    print(f"[02]: {rad[2]:6.3f} rad    [12]: {rad[12]:6.3f} rad     測量寬度： 0.080 m                  預設寬度： 0.090 m")
    print(f"[03]: {rad[3]:6.3f} rad    [10]: {rad[10]:6.3f} rad     測量高度： 0.030 m                  預設高度： 0.050 m")
    print(f"[05]: {rad[5]:6.3f} rad    [13]: {rad[13]:6.3f} rad     測量狀態： OK                       預設狀態： OK")
    print(f"                                         測量座標： [0.000, -0.010, 0.000]   預設座標： [0.000, -0.010, 0.000]")
    print()
    print("預設執行路徑：[1]初始化 -> [2]觀測工件XY面 -> [3]觀測工件XZ面 -> [4]觀測工件YZ面 -> [5]移動到合適的平台")
    print(f"[目前進度]：\t路徑 [{state}] 進行中")
    print("[目前狀態]：\t觀測正常")
    print()
    print("按下 [M] 開啟模式選單")
    print("請輸入模式選擇：", end = "")
    
    return None
    
def display_manual_mode() -> None:
    
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
    print("請輸入模式選擇：", end = "")
    
    return None
    
def display_manual_help_mode() -> None:
    
    display_system_mode()
    print("------------------------------- 手動模式 -------------------------------")
    print("---------------------------機器手臂  指令說明---------------------------")
    print("\nF1: 給座標，讓機器手臂自己計算到該座標的弧度。")
    print("C1: 給定<(x, y, z)>，代入你要的目標座標值。")
    print("---------")
    print("(x, y, z)")
    print("---------\n")
    print("F2: 給指定關節與弧度，讓機器手臂依據移動。")
    print("C2: 給定<指定關節 弧度>")
    print("----")
    print("5 10")
    print("----\n")
    print("[提示]：請在 [手動選單] 底下 [請輸入模式選擇] 輸入 <5 10> or <(0, 1, 2)>，就可以移動了。")
    print("[提示]：建議在 [手動選單] -> [機器手臂 關節弧度] 頁面操作")
    print()
    print("按下 [M] 開啟模式選單")
    print("按下 [Q] 返回上一頁")
    print("請輸入模式選擇：", end = "")
    
    return None
    
def display_menu_mode() -> None:
    
    display_system_mode()
    print("----------選單模式----------")
    print("(1) monitor\t監控數據模式")
    print("(2) manual\t手動操作模式")
    print("(3) exit\t結束主程緒")
    print("---------------------------")
    print()
    print("請輸入模式選擇：", end = "")
        
    return None

def main() -> None:
    
    global alive, view, moving, trajectory, current_robot_joints_radian, current_trajectory_accumulate
        
    initialize_environment_simulation(True)
    set_pybullet_camera_view(0)
    
    base_plane  = load_box(
        base_plane_size, 
        base_plane_position, 
        base_plane_color, 
        base_plane_mass, 
        base_plane_friction_lateral,
        base_plane_friction_spinning,
        base_plane_friction_rolling
    )
    good_plane  = load_box(
        good_plane_size, 
        good_plane_position, 
        good_plane_color, 
        good_plane_mass, 
        good_plane_friction_lateral,
        good_plane_friction_spinning,
        good_plane_friction_rolling
    )	
    poor_plane  = load_box(
        poor_plane_size, 
        poor_plane_position, 
        poor_plane_color, 
        poor_plane_mass, 
        poor_plane_friction_lateral,
        poor_plane_friction_spinning,
        poor_plane_friction_rolling
    )
    
    robot       = load_robot(
        robot_joints_default_radian,
        gripper_friction_lateral = 0.5
    )
    
    camera      = load_box(
        camera_size, 
        camera_position, 
        camera_color, 
        camera_mass
    )
    camera = load_camera(robot, camera)
    
    good_object  = load_box(
        good_object_size, 
        good_object_position, 
        good_object_color, 
        good_object_mass, 
        good_object_friction_lateral,
        good_object_friction_spinning,
        good_object_friction_rolling
    )
    # poor_object  = load_box(
    #     poor_object_size, 
    #     poor_object_position, 
    #     poor_object_color, 
    #     poor_object_mass, 
    #     poor_object_friction_lateral,
    #     poor_object_friction_spinning,
    #     poor_object_friction_rolling
    # )

    # trajectory = trajectory_smoothly(trajectory, 10)

    terminal_clear_screen()
    
    print(f"[INFO]: {time_for_init_wait}s 時間選擇模式")
    time.sleep(time_for_init_wait)

    stable_frame_counter = 0

    try:
        while alive:

            with current_robot_joints_radian_lock:
                for i in range(p.getNumJoints(robot)):
                    current_robot_joints_radian[i] = p.getJointState(robot, i)[0]

            robot_current_pos                   = [p.getJointState(robot, i)[0] for i in range(p.getNumJoints(robot))]
            gripper_link_state                  = p.getLinkState(robot, camera_link_robot_index, computeForwardKinematics = True)
            gripper_coords_display              = list(gripper_link_state[0])
            
            object_base_pos, object_base_ori    = p.getBasePositionAndOrientation(good_object)
            object_coords_display               = list(object_base_pos)
            
            camera_eye_position, camera_target_point, camera_up_vector = calculate_camera_position(camera)
            p.addUserDebugLine(camera_eye_position, camera_target_point, [1, 0, 0], lineWidth = 1.0, lifeTime = 0)
            
            view_direction = np.array(camera_target_point) - np.array(camera_eye_position)
            view_direction = view_direction / (np.linalg.norm(view_direction) + 1e-6)
            if abs(np.dot(view_direction, camera_up_vector)) > 0.99:
                camera_up_vector = [1, 0, 0]

            rgb_image_array, depth_image_array = capture_camera_image(camera_eye_position, camera_target_point, camera_up_vector)
            display_capture_image(rgb_image_array)
            
            if mode == "monitor":
                
                if current_trajectory_accumulate < len(trajectory):
                    
                    robot_target_pos = trajectory[current_trajectory_accumulate]
                    robot_current_pos = robot_move_smoothly(robot, robot_current_pos, robot_target_pos, 1)

                    if np.allclose(robot_current_pos, robot_target_pos, atol = 0.01):
                        stable_frame_counter += 1
                        
                        if stable_frame_counter >= robot_stable_frame_wait:
                            current_trajectory_accumulate += 1
                            stable_frame_counter = 0
                            moving = False
                            # if point = A
                            # calculate_image_contour(rgb_image_array, depth_image_array)
                    else:
                        stable_frame_counter = 0
                        
                elif robot_stable_frame_wait >= len(trajectory) and len(trajectory) > 0:
                    current_trajectory_accumulate = 0
                    stable_frame_counter = 0
                    
            keys = p.getKeyboardEvents()
            
            if ord('I') in keys and keys[ord('I')] & p.KEY_WAS_TRIGGERED:
                
                view = (view + 1) % 6
            
                set_pybullet_camera_view(view + 1)
            
            
            
            p.removeAllUserDebugItems()
            p.stepSimulation()
            time.sleep(UPDATE)

    except Exception as e:
        
        alive = False
        
        print("[INFO]:\tPyBullet 連接已安全斷開")
        
        p.disconnect()
        cv2.destroyAllWindows()
        cv2.waitKey(1)
        exit()
    
if __name__ == "__main__":
    main()