import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))

import robotic as ry
import manipulation as manip
import time
import numpy as np
import utils.robot_execution as robex

C = ry.Config()
# C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))
# qHome = C.getJointState()   

# start_pos = np.array([0.3, 0.3, 0.1125]) 
# target_pos = np.array([0.4, -0.2, 0.1125]) 
# midpoint = np.array([-0.105, 0.4, 0.705-.025])

C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandasTable.g'))
qHome = C.getJointState()   

start_pos = np.array([-0.3, 0.3, 0.1125]) 
target_pos = np.array([0.0, 0.3, 0.1125]) 
midpoint = np.array([-0.105, 0.3, 0.705-.025])

C.addFrame('box', 'table') \
    .setRelativePosition(start_pos) \
    .setShape(ry.ST.ssBox, size=[0.12, 0.12, 0.12, 0.001]) \
    .setColor([1, 1, 0]) \
    .setJoint(ry.JT.rigid) \
    .setContact(1) \
    .setMass(.1)

# for convenience, a few definitions:
gripper = "l_gripper"
palm = "l_palm"
box = "box"
table = "table"

C.view()

def pull(object_, target_pos, info) -> bool:
    # build the manipulation model with the object and the gripper
    M = manip.ManipulationModelling(C, info, ['l_gripper'])
    M.setup_pick_and_place_waypoints(gripper, object_, 1e-1)

    # compute pulling motion
    M.pull([1.,2.], object_, gripper, table)
    M.komo.addObjective([2.], ry.FS.position, [object_], ry.OT.eq, 1e1*np.array([[1,0,0],[0,1,0]]), target_pos)
    M.solve()
    if not M.feasible:
        return False

    # submotion at timestep 0: retract gripper and approach object for pushing
    M1 = M.sub_motion(0, accumulated_collisions=False)
    M1.retractPush([.0, .15], gripper, .03)
    M1.approachPush([.85, 1.], gripper, .03)
    path1 = M1.solve()
    if not M1.feasible:
        return False
    
    # submotion at timestep 1: check for collisions during pushing object to target
    M2 = M.sub_motion(1, accumulated_collisions=False)
    path2 = M2.solve()
    if not M2.feasible:
         return False
    
    M1.play(C, 1.)
    C.attach(gripper, object_)
    M2.play(C, 1.)
    C.attach(table, object_)

    return True

waypoints = [target_pos, start_pos, target_pos]

def pull_multipleWaypoints(object_, info, waypoints) -> bool:

    for i in range(len(waypoints)):
        # build the manipulation model with the object and the gripper
        M = manip.ManipulationModelling(C, info, ['l_gripper'])
        M.setup_pick_and_place_waypoints(gripper, object_, 1e-1)
        # compute pulling motion        
        M.pull([1,2], object_, gripper, table)
        M.komo.addObjective([2], ry.FS.position, [object_], ry.OT.eq, 1e1*np.array([[1,0,0],[0,1,0]]), waypoints[i])
        M.solve()
        if not M.feasible:
            return False


        # submotion at timestep 0: retract gripper and approach object for pushing
        M1 = M.sub_motion(0, accumulated_collisions=False)
        # M1.retractPush([.0, .15], gripper, .03)
        # M1.approachPush([.85, 1.], gripper, .03)
        # close gripper with gripperMove:
               

        path1 = M1.solve()
        if not M1.feasible:
            return False
        
        # submotion at timestep 1: push object to target
        M2 = M.sub_motion(1, accumulated_collisions=False)
        path2 = M2.solve()
        if not M2.feasible:
            return False
        
        M1.play(C, 1.)
        C.attach(gripper, object_)
        M2.play(C, 1.)
        # C.attach(table, object_)
        time.sleep(0.5)

        print(f'Way point {i}: {waypoints[i]} reached')
    return True


# -------- Usage --------
# success_singlePull = pull(box, target_pos, "")
success_multipleWaypoints = pull_multipleWaypoints(box, "", waypoints)
