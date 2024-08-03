import pybullet as p
import time
import pybullet_data

# 连接到PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

# 加载平面和机器人URDF
planeId = p.loadURDF("plane.urdf")
startPos = [0, 0, 1]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
boxId = p.loadURDF("r2d2.urdf", startPos, startOrientation)

# 获取可用的车轮关节
available_joints_indexes = [i for i in range(p.getNumJoints(boxId)) if p.getJointInfo(boxId, i)[2] != p.JOINT_FIXED]
wheel_joints_indexes = [i for i in available_joints_indexes if "wheel" in str(p.getJointInfo(boxId, i)[1])]

# 初始化每个车轮的目标速度
target_v = {i: 0 for i in wheel_joints_indexes}
max_force = 10

# 根据键盘输入更新速度的函数
def update_velocities(keys):
    global target_v
    forward_speed = 10
    
    if p.B3G_UP_ARROW in keys and keys[p.B3G_UP_ARROW] & p.KEY_IS_DOWN:
        for i in wheel_joints_indexes:
            target_v[i] = forward_speed
    elif p.B3G_DOWN_ARROW in keys and keys[p.B3G_DOWN_ARROW] & p.KEY_IS_DOWN:
        for i in wheel_joints_indexes:
            target_v[i] = -forward_speed
    elif p.B3G_LEFT_ARROW in keys and keys[p.B3G_LEFT_ARROW] & p.KEY_IS_DOWN:
        for i in wheel_joints_indexes:
            if i % 2 == 0:  # 左边的车轮
                target_v[i] = forward_speed
            else:  # 右边的车轮
                target_v[i] = 0
    elif p.B3G_RIGHT_ARROW in keys and keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN:
        for i in wheel_joints_indexes:
            if i % 2 == 0:  # 左边的车轮
                target_v[i] = 0
            else:  # 右边的车轮
                target_v[i] = forward_speed
    else:
        for i in wheel_joints_indexes:
            target_v[i] = 0

    # 将速度应用到车轮
    for i in wheel_joints_indexes:
        p.setJointMotorControl2(
            bodyUniqueId=boxId,
            jointIndex=i,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=target_v[i],
            force=max_force
        )

for i in range(10000):
    keys = p.getKeyboardEvents()
    update_velocities(keys)
    
    p.stepSimulation()
    location, _ = p.getBasePositionAndOrientation(boxId)
    p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=110, cameraPitch=-30, cameraTargetPosition=location)
    time.sleep(1/240)

p.disconnect()