#importing the necessary tools
import rospy
from std_msgs.msg import Float64 
from std_msgs.msg import Bool
import RPi.GPIO as GPIO
import time

# setting I/O mode to use the board numbers
GPIO.setmode(GPIO.BOARD)

# setting pin numbers
GPIO.setup(12, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(10, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# initializing helping variables
count = 0
count_2 = 0

# initiliazing ROS node
rospy.init_node("switch")

# in this script, we are using 2 push buttons as switches
# the count variables sets the mode to one of the two switch modes depending on the count variable (even or odd)

#importing the necessary tools
import rospy
from std_msgs.msg import Float64 
from std_msgs.msg import Bool
import RPi.GPIO as GPIO
import time

# setting I/O mode to use the board numbers
GPIO.setmode(GPIO.BOARD)

# setting pin numbers
GPIO.setup(12, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(10, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# initializing helping variables
count = 0
count_2 = 0

# initiliazing ROS node
rospy.init_node("switch")

# in this script, we are using 2 push buttons as switches
# the count variables sets the mode to one of the two switch modes depending on the count variable (even or odd)

# in this script, we are using 2 push buttons as switches
# the count variables sets the mode to one of the two switch modes depending on the count variable (even or odd)

while True:

        # 1st push button: yaw control 
        # getting input from push button
        state_1 = GPIO.input(12)
        # if buttons is pressed
        if state_1 == False:
                count = count + 1
                # if count is even: mode 1 of the switch, default mode
                if (count%2) == 0:
                # when in default mode, we do not control the yaw of the robot, so ROS does not need to do anything here
                # actions ROS takes in this case
                        print("Yaw is free, press button to regulate it")
                # if count is odd: mode 2 of the switch
                if (count%2) == 1:
                # actions ROS takes in this case
                        print("Yaw is regulated")
                # here we detect the current yaw heading and maintain it
                # we will subscribe to a topic that knows our current yaw and publish the same value to the PID to control the yaw
                        current_yaw = rospy.wait_for_message("/sensors/yaw", Float64)
                        pub_yaw = rospy.Publisher('/topic_for_yaw_PID', Float64)
                        pub_yaw.publish(current_yaw) 
                # time off added for a smooth button press
                time.sleep(1)



# same as the previous block but for the second push button
        # 2nd push button: depth
        state_2 = GPIO.input(10)
        if state_2 == False:
                count_2 = count_2 + 1
                pub_depth_regul = rospy.Publisher('depth_regulation', Bool)
                if (count_2%2) == 0:
                        print("Depth is free, push button to regulate it")
                        depth_control = False
                        pub_depth_regulation.publish(depth_control)
                # in this case, the depth is free, so ROS does not have to do anything
                if (count_2%2) == 1:
                        print("Depth is regulated")
                        current_depth = rospy.wait_for_message("/sensors/depth", Float64)
                        depth_control = True
                        pub_depth_regulation.publish(depth_control)
                        pub_dep = rospy.Publisher('topic_for_depth_PID', Float64)
                        pub_dep.publish(current_depth)
                # in this case, we maintain the current depth using the PID
                time.sleep(1)