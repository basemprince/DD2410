#!/usr/bin/env python2
import rospy
import actionlib
import irob_assignment_1.msg
from irob_assignment_1.srv import GetSetpoint, GetSetpointRequest, GetSetpointResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import tf2_ros
import tf2_geometry_msgs
from math import atan2, hypot

# Use to transform between frames
tf_buffer = None
listener = None

# The exploration simple action client
goal_client = None
# The collision avoidance service client
control_client = None
# The velocity command publisher
pub = None

# The robots frame
robot_frame_id = "base_link"

# Min allowed gain to move along path (in feedback)
min_allowed_gain = 3

# Max linear velocity (m/s)
max_linear_velocity = 0.5
# Max angular velocity (rad/s)
max_angular_velocity = 1.0


def goal_active():
    rospy.loginfo("I got activated")


def goal_feedback(feedback):
    rospy.loginfo("I got feedback")

    # Check if this path has higher gain than min_allowed_gain

    # If it has cancel goal and move along the path


def goal_result(state, result):
    rospy.loginfo("I got a result")

    # If the state is succeeded then

    # Move along the path if path is not empty


def move(path):
    global control_client, robot_frame_id, pub

    # Call service client with path

    # Transform Setpoint from service client

    # Create Twist message from the transformed Setpoint

    # Publish Twist

    # Call service client again if the returned path is not empty and do stuff again

    # Send 0 control Twist to stop robot

    # Get new path from action server


def get_path():
    global goal_client
    goal_client.wait_for_server()
    goal = irob_assignment_1.msg.GetNextGoalGoal()
    goal_client.send_goal(goal, active_cb=goal_active,
                          feedback_cb=goal_feedback, done_cb=goal_result)
    # Get path from action server

    # Call move with path from action server


if __name__ == "__main__":
    # Initializing the controller node
    rospy.init_node("controller")
    
    # initializing publishing to the cmd_vel topic with a twist message. limit the queued messages to 10 if any subscribers are not receiving them fast enough
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # Initialzing simple action library "get_next_goal" client from the exploration node
    rospy.loginfo("initializing simple action client the exploration")
    goal_client = actionlib.SimpleActionClient("get_next_goal", irob_assignment_1.msg.GetNextGoalAction)
    
    # Initititalizing the service "get_setpoint" client from the collision avoidance node
    rospy.loginfo("initializing service client obstical avoidance")
    #block untill the service is available
    rospy.wait_for_service('get_setpoint')
    control_client = rospy.ServiceProxy('get_setpoint',GetSetpoint)


    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    # Call get path
    get_path()
    # Spin to keep node from exiting before node shutdown
    rospy.spin()
