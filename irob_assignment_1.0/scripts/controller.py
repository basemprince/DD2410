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
from std_msgs.msg import String

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

# Max linear velocity (m/s)
max_linear_velocity = 0.5
# Max angular velocity (rad/s)
max_angular_velocity = 2.0


def move(path):
    global control_client, robot_frame_id, pub

    # Initialzing a twist message
    msg = Twist()
    # Maintain particular rate for loops
    rate = rospy.Rate(10.0)
    # A while loop to keep running if the path still contains poses
    while path.poses:
        # Call service client with path
        setpoint_result = control_client(path)
        # extracting the setpoint from the service client object in the map frame
        setpoint = setpoint_result.setpoint

        # Transforming from source_frame (map) to target_frame (base_link) at the time time. because subscribers to the cmd_vel assume the velocity command is in the robots frame
        transform = tf_buffer.lookup_transform(robot_frame_id,setpoint.header.frame_id, rospy.Time.now(),rospy.Duration(1))
        transformed_setpoint = tf2_geometry_msgs.do_transform_point(setpoint, transform)
        
        # Calculating the linear and angular components of the twist message from the transformed Setpoint   
        msg.angular.z = min(max_angular_velocity, atan2(transformed_setpoint.point.y,transformed_setpoint.point.x))
        # To make the robot turn on a dime if angular required velocity is 80% or higher of the maximum angluar velocity
        if msg.angular.z >= (0.8*max_angular_velocity):
            msg.linear.x = 0
        else:
            msg.linear.x = min(max_linear_velocity, hypot(transformed_setpoint.point.x , transformed_setpoint.point.y))

        # Publishing the twist message to the cmd_vel topic in the base_link frame
        pub.publish(msg)
    
        rate.sleep()
        #Call the service client again to get the next setpoint (if any)
        path = setpoint_result.new_path
    
    # Changing the linear and angluar velocities of robot to zero and publishing it to stop it from moving      
    msg.linear.x = 0
    msg.angular.z = 0
    pub.publish(msg)

    # Get a new path from the action server
    get_path()

def get_path():
    global goal_client
    # Get path from action server
    rospy.loginfo("sending goal to action server")
    #wait untill action server starts up
    goal_client.wait_for_server()
    #sending an empty goal to "get_next_goal"
    goal_client.send_goal('')
    rospy.loginfo("waiting for results from action server")
    #Wait for server to perform the action
    goal_client.wait_for_result()
    next_goal_result = goal_client.get_result()
    #extracting path from the "next_goal_result" object
    path = next_goal_result.path
    rospy.loginfo("result received")
    # Call move function with path from action server
    move(path)

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

    #initializng a buffer for the tf2 transform
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    # Call get path function
    get_path()
    # Spin to keep node from exiting before node shutdown
    rospy.spin()
