import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import time
import robotic as ry
import numpy as np
import utils.buildMaze as pWorld
import utils.robot_execution as robex
import utils.expanded_manipulation as expManip

def solve_maze_rrt(C: ry.Config, visual: bool, distance: float) -> ry._robotic.SolverReturn:
    """
    Takes a ry.Config with just some obstacles and a start
    and goal marker frames.

    Returns the output of the RRT solver.

    Will generate an XYPhi joint and ssBox frame.
    """
    start_pos = C.getFrame("start").getPosition()
    goal_pos = C.getFrame("goal").getPosition()
    C.addFrame("base").setPosition([0.0, 0.0, 0.0])
    C.addFrame("moving_box", "base") \
        .setRelativePosition(start_pos) \
        .setJoint(ry.JT.transXYPhi, [-1., 1., -1., 1., -3., 3.]) \
        .setShape(ry.ST.ssBox, size=[box_size[0]+distance, box_size[1]+distance, box_size[2], box_size[3]]) \
        .setColor([0, 1., 1.]) \
        .setContact(1)
    q0 = [*start_pos[:2], 0]
    qT = [*goal_pos[:2], 0]

    C.setJointState(qT)
    rrt = ry.PathFinder()
    rrt.setProblem(C, [q0], [qT])
    ret = rrt.solve()
    
    if visual and ret.feasible:
        C.view(True)
        for i, q in enumerate(ret.x):
            C.addFrame(f"marker{i}") \
                .setShape(ry.ST.marker, size=[.04]) \
                .setPosition([q[0], q[1], 0.0]) \
                .setColor([1, 1, 1])
            ret.x[i] = [q[0], q[1], table_height]   # move path to table height 0.6 in z axis 
            print(f'found path:{ret.x}')
            C.addFrame(f"marker_exec{i}") \
                .setShape(ry.ST.marker, size=[.04]) \
                .setPosition([q[0], q[1], table_height]) \
                .setColor([1, 1, 1])
            C.setJointState(q)
            C.view()
        C.view(True)
    
    return ret

def plan_path(q_pWorld, q_start, q_goal, table_height, scenario, box_size):
    ry.params_add({'rrt/stepsize': .05})
    C = ry.Config()
    puzzle_world = pWorld.PuzzleWorld(C, None, q_pWorld, q_start, q_goal)
    puzzle_world.build()
    ret = solve_maze_rrt(C, True, 0.0) 
    if not ret.feasible:
        print("The RRT solver was unable to find a feasible path.")
        exit()
    
    path = ret.x
    # move up path at half height of box (here: 0.03)
    for i, q in enumerate(path):
        path[i] = [q[0], q[1], 0.015]

    # move puzzle world up to table position
    C.addFile(ry.raiPath(scenario))
    C.getFrame("puzzle_world").setPosition([q_pWorld[0], q_pWorld[1], table_height])
    C.delFrame("moving_box")
    
    box_pos = C.getFrame("start").getRelativePosition()
    box_pos[2] = 0.015
    C.addFrame("box", "puzzle_world") \
        .setRelativePosition(box_pos) \
        .setShape(ry.ST.ssBox, size=box_size) \
        .setColor([1, 1, .0]) \
        .setJoint(ry.JT.rigid) \
        .setContact(1) \
        .setMass(1.)
    if withHold == True:
        C.addFrame("hold", "box") \
            .setRelativePosition([0.0, 0.0, box_size[2]/2 + 0.025]) \
            .setShape(ry.ST.ssBox, size=[.04, .04, .05, .001]) \
            .setColor([1, 1, .0]) \
            .setContact(1) \
            .setMass(1.)
    C.view()
    return C, path

def execute(C: ry.Config, path: list, grasp_height: float, withHold: bool):
    man = expManip.ManipulationModelling(C)
    man.setup_inverse_kinematics()
    if withHold == True:
        # man.grasp_top_box(1.,"l_gripper", "hold", "xz")
        hold_height = C.getFrame("hold").getPosition()[2]
        man.grasp_top_box_atGivenHeight(1.,"l_gripper", "hold", grasp_height, "xz")
    else:    
        man.grasp_top_box(1.,"l_gripper", "box", "xz")
    pose = man.solve()

    if man.feasible:
        # move to box
        robot = robex.Robot(C, False)   # True if real robot
        robot.execute_path_blocking(C, pose)
        robot.grasp(C)
        
        # compute path 
        man = expManip.ManipulationModelling(C)
        man.follow_path_on_plane(path, "z", "l_gripper")
        path_solution = man.solve()
               
        # execute path
        try:
            robot.execute_path_blocking(C, path_solution)
        except:
            print("Path is not feasible!")
        
        # return to home position
        robot.bot.gripperMove(ry._left, .078, .4)
        while not robot.bot.gripperDone(ry._left):
            robot.bot.sync(C, .1)
        robot.bot.home(C)

    C.view(True)

if __name__ == "__main__":
    
    # ----- GLOBAL VARIABLES -----

    q_pWorld=[-0.3, 0.3, 0.0]
    q_start=[0.10, .10, .0]
    q_goal=[-0.14, .14, .0]
    
    # scenario, table_height = 'scenarios/pandaSingle.g', 0.65
    scenario, table_height = 'scenarios/pandasTable.g', 0.6

    box_size = [.07, .075, .06, .001]
    hold = True


    # ----- START MAIN -----

    C, path = plan_path(q_pWorld, q_start, q_goal, table_height, scenario, box_size)
    execute(C, path, 0.015, hold)
    
