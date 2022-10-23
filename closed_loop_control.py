import rospy
import numpy as np
import tf
from tf import transformations as t
from tf import LookupException
import geometry_msgs.msg as gm


class Robot:

	def __init__(self, init_pose):
		self.pose = np.array(init_pose)
		self.tl = tf.TransformListener()
		self.perceived_pose = None
		self.r = rospy.Rate(10)

def commands_to_waypoints(current_position, commands):
    """
    Returns the expected poses based upon a series of commands
    """
    sub_waypoints = []
    for command in commands:
        command_str, duration = command
        x_f, y_f, theta_f = current_position

        if command_str == "rotate cw":
            proportion = duration / 5.4
            delta_theta = -1*2*np.pi*proportion
            theta_f = current_position[2] + delta_theta
        elif command_str == "rotate ccw":
            proportion = duration / 5.4
            delta_theta = 1*2*np.pi*proportion
            theta_f = current_position[2] + delta_theta
        elif command_str == "forward" or command_str == "backward":
            c = duration / 5.4
            theta = current_position[2]
            if command_str == "forward":
                x_f = current_position[0] + c*np.cos(theta)
                y_f = current_position[1] + c*np.sin(theta)
            else:
                x_f = current_position[0] - c*np.cos(theta)
                y_f = current_position[1] - c*np.sin(theta)
        else:
            raise Exception("Invalid control command generated")

        sub_waypoints.append([x_f, y_f, theta_f])
    return sub_waypoints



if __name__ == "__main__":
    # Repeat the process for list of waypoints


    # go from [0, 0, 0] to [1, 1, 0]
    commands = generate_command_sequence(current_position, waypoint)
    sub_waypoints = commands_to_waypoints(commands)
    control_expectations = list(zip(commands, sub_waypoints))
    for expectation in control_expectations:
        command, sub_wp = expectation
        command_str, duration = command
        if duration < 2:
            execute_command(command_str, duration)
        else:
            execute_command(command_str, 2):
            current_position = estimate_pose()
            flag = True
            while difference(current_position, sub_wp) > epsilon and flag:
                if command_str == "rotate cw" or command_str == "rotate ccw":
                    com, dur = regenerate_rotate_command() # for cw and ccw
                    execute_regenerated_command(com, dur)

                else:
                    com, dur regenerate_forward_command() # or backward
                    execute_regenerated_command(com, dur)
                flag = dur > 2 # becomes false to exit while loop when duration is short enough
                current_position = estimate_pose()

        current_position = estimate_pose()

    error = difference(current_position, waypoint)
    print("localization error")
