import pybullet as p
import time
import pybullet_data

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
startPos = [0,0,0]
startOrientation = p.getQuaternionFromEuler([0,0,0])
kinovaId = p.loadURDF("kinova_gym/models/urdf/gen3.urdf",startPos, startOrientation)

num_links = p.getNumJoints(kinovaId)
working_surface_link = None
for i in range(num_links):
    print(f"kinova arm: {p.getJointInfo(kinovaId, i)[12]} {p.getJointInfo(kinovaId, i)[0]}")

print()

for i in range (10000):
    p.stepSimulation()
    # set all joint positions to 0
    num_joints = p.getNumJoints(kinovaId)
    for j in range(num_joints):
        p.resetJointState(kinovaId, j, 0)
    
    time.sleep(1./240.)

p.disconnect()
