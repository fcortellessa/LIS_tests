"""
Proof of concept for solving a puzzle in which obstacles have to be moved in order to reach the goal.
In future work, an algorithm will find first first find a path to the goal and then plan the manipulation of the obstacles.
In this proof of concept, the obstacles are moved in a predefined way subsequentially.
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import time
import robotic as ry
import numpy as np
import utils.buildMaze as pWorld
import utils.robot_execution as robex
import utils.expanded_manipulation as expManip

def solve_maze_rrt(C: ry.Config, moving_object: str, start_pos: list, goal_pos: list, distance: float, visual: bool, stepsize: float) -> ry._robotic.SolverReturn:
    """
    Takes a ry.Config with just some obstacles and a start
    and goal marker frames.

    Returns the output of the RRT solver.

    Will generate an XYPhi joint and ssBox frame.
    """
    ry.params_add({'rrt/stepsize': stepsize})
    try:
        C.delFrame("q0")
        C.delFrame("qT")
    except:
        pass

    C.addFrame("{moving_object}start", "puzzle_world") \
        .setRelativePosition(start_pos) \
        .setShape(ry.ST.marker, size=[.04]) \
        .setColor([0, 1, 0])
    C.addFrame("{moving_object}goal", "puzzle_world") \
        .setRelativePosition(goal_pos) \
        .setShape(ry.ST.marker, size=[.04]) \
        .setColor([0, 1, 0])

    q0 = [*start_pos[:2], 0.0]
    qT = [*goal_pos[:2], 0.0]

    C.addFrame("q0", "puzzle_world") \
        .setShape(ry.ST.marker, size=[.1]) \
        .setRelativePosition(q0) \
        .setColor([1, 0, 0])
    C.addFrame("qT", "puzzle_world") \
        .setShape(ry.ST.marker, size=[.1]) \
        .setRelativePosition(qT) \
        .setColor([1, 0, 0])

    C.setJointState(qT)
    rrt = ry.PathFinder()
    rrt.setProblem(C, [q0], [qT])
    ret = rrt.solve()
    
    if visual and ret.feasible:
        for i, q in enumerate(ret.x):
            C.addFrame(f"marker_{moving_object}_{i}", "puzzle_world") \
                .setShape(ry.ST.marker, size=[.04]) \
                .setRelativePosition([q[0], q[1], 0.0]) \
                .setColor([1, 1, 1])
            ret.x[i] = [q[0], q[1], table_height]
            C.view()
        C.view(True)

    # move up path at half height of box
    if not ret.feasible:
        print("The RRT solver was unable to find a feasible path for the object.")
        exit()
    if ret.feasible:
        path = ret.x
        moving_obj_size = C.getFrame(moving_object).getSize()
        for i, q in enumerate(path):
            path[i] = [q[0]+q_pWorld[0], q[1]+q_pWorld[1], moving_obj_size[2]/2.0]  
    print(f'found path:{path}')
    return path

def plan_path(q_pWorld: list, q_start: list, path: list, table_height: float, scenario: str, moving_object: str, moving_obj_size: list, hold_size: list):
    # move puzzle world up to table position
    C.addFile(ry.raiPath(scenario))
    C.getFrame("puzzle_world").setPosition([q_pWorld[0], q_pWorld[1], table_height])
    C.getFrame(moving_object).setRelativePosition([*q_start[:2], moving_obj_size[2]/2.0])
    C.getFrame(moving_object).setJoint(ry.JT.rigid)
    if hold_size != None:
        C.addFrame("hold", moving_object) \
            .setRelativePosition([0.0, 0.0, moving_obj_size[2]/2.0 + hold_size[2]/2.0]) \
            .setShape(ry.ST.ssBox, size=hold_size) \
            .setColor([1, 1, .0]) \
            .setContact(1) \
            .setMass(1.)
        C.addFrame("holdcenter", "hold") \
            .setRelativePosition([0.0, 0.0, 0.0]) \
            .setShape(ry.ST.marker, size=[0.1, 0.1, 0.1]) \
            .setColor([1, 1, 1])
    return C, path

def execute(C: ry.Config, object: str, path: list, grasp_height: float, withHold: bool, onRobot: bool):
    man = expManip.ManipulationModelling(C)
    man.setup_inverse_kinematics()
    if withHold == True:
        hold_height = C.getFrame("hold").getSize()[2]
        grasp_pos = hold_height/2.0 - grasp_height
        man.grasp_top_box_atGivenHeight(1.,"l_gripper", "hold", grasp_pos, "xz")
    else:    
        box_height = C.getFrame(object).getSize()[2]
        grasp_pos = box_height/2.0 - grasp_height
        man.grasp_top_box_atGivenHeight(1.,"l_gripper", object, grasp_pos, "xz")
    pose = man.solve()

    if man.feasible:
        # move to box
        robot = robex.Robot(C, onRobot)
        robot.execute_path_blocking(C, pose)
        robot.grasp(C)

        # compute path 
        man = expManip.ManipulationModelling(C)
        man.follow_path_on_plane(path, "z", "l_gripper")
        path_solution = man.solve()
               
        # execute path
        try:
            robot.execute_path_blocking(C, path_solution, time_to_solve=(len(path_solution)-1)*0.1)
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
    q_start=[0.12, .12, 0.0]
    q_goal=[-0.16, .16, 0.0]
    
    scenario, table_height = 'scenarios/pandasTable.g', 0.6
    # scenario, table_height = 'scenarios/pandaSingle.g', 0.65

    moving_obstacle_size = [.03, .03, .03, .001]
    moving_obstacle_pos = [-0.1, 0.1, 0.0]
    moving_obstacle_goal = [0.06, -0.14, 0.0]

    box_size = [.07, .075, .06, .001]
    hold_size = [.04, .04, .05, .001]
    grasp_height = 0.02
    hold = True


    # ----- START MAIN -----
    
    # build configuration
    C = ry.Config()
    puzzle_world = pWorld.PuzzleWorld(C, None, q_pWorld, q_start, q_goal)
    puzzle_world.build()

    C.addFrame("moving_obstacle", "puzzle_world") \
        .setRelativePosition(moving_obstacle_pos) \
        .setJoint(ry.JT.transXYPhi, [-1., 1., -1., 1., -3., 3.]) \
        .setShape(ry.ST.ssBox, size=moving_obstacle_size) \
        .setColor([0, 1., 1.]) \
        .setContact(1) \
        .setMass(1.)
    C.view(True)

    C.addFrame("box", "puzzle_world") \
        .setRelativePosition([*q_start[:2], box_size[2]/2.0]) \
        .setShape(ry.ST.ssBox, size=box_size) \
        .setColor([1, 1, .0]) \
        .setJoint(ry.JT.rigid) \
        .setContact(1) \
        .setMass(1.)
    if hold_size != None:
        C.addFrame("hold", "box") \
            .setRelativePosition([0.0, 0.0, box_size[2]/2.0 + hold_size[2]/2.0]) \
            .setShape(ry.ST.ssBox, size=hold_size) \
            .setColor([1, 1, .0]) \
            .setContact(1) \
            .setMass(1.)
        C.addFrame("holdcenter", "hold") \
            .setRelativePosition([0.0, 0.0, 0.0]) \
            .setShape(ry.ST.marker, size=[0.1, 0.1, 0.1]) \
            .setColor([1, 1, 1])
    C.view(True)
    
    # solve maze for each moving object
    obstacle_path = solve_maze_rrt(C, "moving_obstacle", moving_obstacle_pos, moving_obstacle_goal, 0.0, False, 0.05)
    C.getFrame("moving_obstacle").setJoint(ry.JT.rigid)
    C.getFrame("box").setJoint(ry.JT.transXYPhi, [-1., 1., -1., 1., -3., 3.])
    box_path = solve_maze_rrt(C, "box", q_start, q_goal, 0.0, False, 0.05)
    C.getFrame("box").setJoint(ry.JT.rigid)

    # plan path and execute in scenario
    C, obstacle_path = plan_path(q_pWorld, moving_obstacle_pos, obstacle_path, table_height, scenario, "moving_obstacle", moving_obstacle_size, None)
    execute(C, "moving_obstacle", obstacle_path, grasp_height, False, False)

    C, box_path = plan_path(q_pWorld, q_start, box_path, table_height, scenario, "box", box_size, hold_size) 
    execute(C, "box", box_path, grasp_height, hold, False)
