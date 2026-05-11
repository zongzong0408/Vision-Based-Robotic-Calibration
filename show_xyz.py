import pybullet as p
import pybullet_data
import time

GRAVITY = -10.0
TIME_STEP = 1.0 / 1000.0

alive = True

# 連接 PyBullet 伺服器，使用 GUI 模式可視化模擬
p.connect(p.GUI)
# 設定模擬環境的重力
p.setGravity(0, 0, GRAVITY)
# 設定模擬的時間步長
p.setTimeStep(TIME_STEP)
# 關閉陰影渲染，提升性能
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
# 啟用 PyBullet 的 GUI 介面元件
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)

# 添加 PyBullet 預設的資料路徑，以便載入像 'plane.urdf' 這樣的模型
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# 載入一個無限大的平面作為地面
p.loadURDF("plane.urdf")

# 繪製世界座標軸線條
# X 軸 (紅色)
p.addUserDebugLine([0, 0, 0], [1, 0, 0], [1, 0, 0], lineWidth = 5)
# Y 軸 (綠色)
p.addUserDebugLine([0, 0, 0], [0, 1, 0], [0, 1, 0], lineWidth = 5)
# Z 軸 (藍色)
p.addUserDebugLine([0, 0, 0], [0, 0, 1], [0, 0, 1], lineWidth = 5)

# 顯示對應的軸英文文字
# X 軸文字 (紅色)
p.addUserDebugText("X", [1.2, 0, 0], textColorRGB = [1, 0, 0], textSize = 1.5, lifeTime = 0)
# Y 軸文字 (綠色)
p.addUserDebugText("Y", [0, 1.2, 0], textColorRGB = [0, 1, 0], textSize = 1.5, lifeTime = 0)
# Z 軸文字 (藍色)
p.addUserDebugText("Z", [0, 0, 1.2], textColorRGB = [0, 0, 1], textSize = 1.5, lifeTime = 0)

while alive:
    
    keys = p.getKeyboardEvents()
    
    if ord('q') in keys and keys[ord('q')] & p.KEY_WAS_TRIGGERED:
        print("INFO: 退出模擬")
        alive = False
    
    # 推進模擬一步
    p.stepSimulation()
    # 控制模擬速度
    time.sleep(TIME_STEP)
    
print("INFO: 安全退出模擬")
p.disconnect()