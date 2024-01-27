#conda activate pybullet-tutorials

import pybullet as p
import pybullet_data

import util
from util import move_to_joint_pos, gripper_open, gripper_close


def move_to_ee_pose(robot_id, target_ee_pos, target_ee_orientation=None):
    """
    Moves the robot to a given end-effector pose.
    :param robot_id: pyBullet's body id of the robot
    :param target_ee_pos: (3,) list/ndarray with target end-effector position
    :param target_ee_orientation: (4,) list/ndarray with target end-effector orientation as quaternion
    """
    # TODO (student): implement this function

    joint_pos = p.calculateInverseKinematics(
        robot_id,
        util.ROBOT_EE_LINK_ID,
        targetPosition=target_ee_pos,
        targetOrientation=target_ee_orientation,
        maxNumIterations=100,
        residualThreshold=0.001
    )

    target_values = joint_pos[:7]
    move_to_joint_pos(robot_id,target_values)

    

def pick_and_place_cube(robot_id, cube_position, target_position, quat):
    """
    Picks a cube from a given position and places it at the target position.
    :param robot_id: pyBullet's body id of the robot
    :param cube_position: (3,) list/ndarray with the cube's position
    :param target_position: (3,) list/ndarray with the target position
    :param quat: (4,) list/ndarray with the end-effector orientation as quaternion
    """
    # Approach above the cube
    move_to_ee_pose(robot_id, [cube_position[0], cube_position[1], cube_position[2] + 0.075], quat)
    gripper_open(robot_id)

    # Lower the gripper to the cube and close it
    move_to_ee_pose(robot_id, cube_position, quat)
    gripper_close(robot_id)

    # Lift the cube
    move_to_ee_pose(robot_id, [cube_position[0], cube_position[1], cube_position[2] + 0.075], quat)
    move_to_joint_pos(robot_id, util.ROBOT_HOME_CONFIG)

    # Move to the target position and release the cube
    move_to_ee_pose(robot_id, [target_position[0], target_position[1], target_position[2] + 0.075], quat)
    gripper_open(robot_id)
    move_to_joint_pos(robot_id, util.ROBOT_HOME_CONFIG)


def main():
    # connect to pybullet with a graphical user interface
    p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.resetDebugVisualizerCamera(1.7, 60, -30, [0.2, 0.2, 0.25])

    # basic configuration
    p.setGravity(0, 0, -9.81)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # allows us to load plane, robots, etc.
    plane_id = p.loadURDF('plane.urdf')  # function returns an ID for the loaded body

    # load the robot
    robot_id = p.loadURDF('franka_panda/panda.urdf', useFixedBase=True)

    # load an object to grasp and a box
    red_cube_position =[0.5, -0.3, 0.025]
    red_cube_id = p.loadURDF('cube_small.urdf', basePosition=red_cube_position, baseOrientation=[0, 0, 0, 1])
    p.resetVisualShapeData(red_cube_id, -1, rgbaColor=[1, 0, 0, 1])
    tray_id = p.loadURDF('tray/traybox.urdf', basePosition=[0.5, 0.5, 0.0], baseOrientation=[0, 0, 0, 1])

    # load a blue cube
    blue_cube_position = [0.7, -0.3, 0.025]
    blue_cube_id = p.loadURDF('cube_small.urdf', basePosition=blue_cube_position, baseOrientation=[0, 0, 0, 1])
    p.resetVisualShapeData(blue_cube_id, -1, rgbaColor=[0, 0, 1, 1])  # blue color

    # load a yellow cube
    yellow_cube_position = [0.3, -0.3, 0.025]
    yellow_cube_id = p.loadURDF('cube_small.urdf', basePosition=yellow_cube_position, baseOrientation=[0, 0, 0, 1])
    p.resetVisualShapeData(yellow_cube_id, -1, rgbaColor=[1, 1, 0, 1])  # yellow color


    print('******************************')
    input('press enter to start simulation')
    config1 = [-0.7854, 0.75, -1.3562, -1.5708, 0.0, 1.5708, 0.7854]
    config2 = [0.7854, 0.1, -0.7854, -2.1, 0.0, 1.5708, 0.7854]

    print('going to home configuration')
    move_to_joint_pos(robot_id, util.ROBOT_HOME_CONFIG)
    # gripper_open(robot_id)
    # print('going to configuration 1')
    # move_to_joint_pos(robot_id, config1)
    # print('going to configuration 2')
    # move_to_joint_pos(robot_id, config2)
    # print('going to home configuration')
    # move_to_joint_pos(robot_id, util.ROBOT_HOME_CONFIG)

    pos,quat, *_ = p.getLinkState(
        robot_id,
        util.ROBOT_EE_LINK_ID,
        computeForwardKinematics=True
    )
    
    base_tray_position = [0.5, 0.5, 0.0]

    # Offsets for each cube on the tray
    offset_distance = 0.05  # You can adjust this based on the size of the cubes and the tray

    # Define specific positions on the tray for each cube
    tray_position_red = [base_tray_position[0], base_tray_position[1], base_tray_position[2]]
    tray_position_blue = [base_tray_position[0] + offset_distance, base_tray_position[1], base_tray_position[2]]
    tray_position_yellow = [base_tray_position[0] + 2* offset_distance, base_tray_position[1], base_tray_position[2]]



    # Pick and place the red cube
    pick_and_place_cube(robot_id, red_cube_position, tray_position_red, quat)

    # Pick and place the blue cube
    pick_and_place_cube(robot_id, blue_cube_position, tray_position_blue, quat)

    # Pick and place the yellow cube
    pick_and_place_cube(robot_id, yellow_cube_position, tray_position_yellow, quat)


    print('program finished. hit enter to close.')
    input()
    # clean up
    p.disconnect()


if __name__ == '__main__':
    main()
