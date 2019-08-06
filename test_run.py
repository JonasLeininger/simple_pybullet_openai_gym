import pybullet as p
import time
import pybullet_data

def getJointNames(quadruped):
    nJoints = p.getNumJoints(quadruped)
    jointNameToId = {}

    for i in range(nJoints):
        jointInfo = p.getJointInfo(quadruped, i)
        jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
    return jointNameToId


physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,0]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
# boxId = p.loadURDF("r2d2.urdf",cubeStartPos, cubeStartOrientation)
armId = p.loadURDF("cougarbot.urdf",cubeStartPos, cubeStartOrientation)
jointNameToId = getJointNames(armId)

kp = 0.1  # 0.012
kd = 10.4  # .2
maxForce = 120.5
print("hip index : ", jointNameToId["hip"])
p.setJointMotorControl2(bodyIndex=armId,
                        jointIndex=jointNameToId["hip"],
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=-3.141,
                        positionGain=kp,
                        velocityGain=kd,
                        force=maxForce)

cubePos, cubeOrn = p.getBasePositionAndOrientation(armId)
asf = p.getJointInfo(armId, jointNameToId["elbow"])
print(asf)
print('Position :', cubePos, ' ', cubeOrn)
while True:
    p.stepSimulation()
    time.sleep(1./240.)

cubePos, cubeOrn = p.getBasePositionAndOrientation(armId)
print(cubePos,cubeOrn)
p.disconnect()
