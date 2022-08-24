import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from RPi import GPIO
from time import sleep

# defining encoder pins, for several encoders, we need several pins
clk = 17
dt = 18
# repeat the previous block for other encoders
# clk_2 = pin_number
# dt_2 = pin_number

# setting GPIO mode and set up
GPIO.setmode(GPIO.BCM)
GPIO.setup(clk, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(dt, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# initializing the variables
counter = 0
clkLastState = GPIO.input(clk)


# initializing ROS node
rospy.init_node("encoder")
depth_status = rospy.Subscriber('depth_regulation', Bool)

# defining the publishers
publ = rospy.Publisher('light_control_topic', Float64)
pubd = rospy.Publisher('topic_for_depth_PID', Float64)
pubt = rospy.Publisher('topic_for_ESC', Float64)

# defining a range for the operator
min_range = 0
max_range = 360

def range_lights(value_lights):
        if value_lights < min_range:
                value_lights = 0
        if min_range in range(0, 90):
                value_lights = 0,25
        if min_range in range(90, 180):
                value_lights = 0,5
        if min_range in range(180, 270):
                value_lights = 0,75
        if min_range in range(270, 360):
                value_lights = 1
        return value_lights

"""

def range_depth(value_depth):
        if value_depth < min_range:
                value_depth = 0
        if value_depth in range(0, max_range/2):
                value_depth = 0
        if value_depth in range(max_range/2, max_range):
                value_depth = 5
        return value_depth

def range_thrusters(value_thrusters):
        if value_thrusters < min_range:
                value_thrusters = 0
        if value_thrusters in range(0, max_range/10):
                value_thrusters = 0,1
        if value_thrusters in range(max_range/10, max_range/5):
                value_thrusters = 0,2
        if value_thrusters in range(max_range/5, 3*max_range/10):
                value_thrusters = 0,3
        if value_thrusters in range(3*max_range/10, 2*max_range/5):
                value_thrusters = 0,4
        if value_thrusters in range(4*max_range/10, max_range/2):
                value_thrusters = 0,5
        if value_thrusters in range(max_range/2, 3*max_range/5):
                value_thrusters = 0,6
        if value_thrusters in range(3*max_range/5, 7*max_range/10):
                value_thrusters = 0,7
        if value_thrusters in range(7*max_range/10, 4*max_range/5):
                value_thrusters = 0,8
        if value_thrusters in range(4*max_range/5, 9*max_range/10):
                value_thrusters = 0,9
        if value_thrusters in range(9*max_range/10, max_range):
                value_thrusters = 1
        return value_thrusters
"""

# infinite loop that checks the last and the current position of the encoder 
# to check if it has moved
while True:
        clkState = GPIO.input(clk)
        dtState = GPIO.input(dt)
        if clkState != clkLastState:
                if dtState != clkState:
                        counter += 1
                else:
                        counter -= 1
                        print(counter)
                        # print(counter_2)
                        # print(counter_3)
        # counter is the output value of the encoder
        clkLastState = clkState
        sleep(0.01)
        # repeat the previous block for each of the encoders

        # making sure the operator stays within the range
        # these values are used as a coefficient that is multiplies by the power input of the controlled plant
        value_lights = counter % 360
        value_depth = counter % 360     # counter_2
        value_thrusters = counter % 360 # counter_3

        # calling the ad-hoc function
        range_lights(value_lights)
        #range_depth(value_depth)
        #range_thrusters(value_thrusters)

        # publishing values to ROS
        publ.publish(value_lights)
        #pubt.publish(value_thrusters)
        # only send depth commands if the depth is not regulated
        #if depth_status == False:
        #       pubd.publish(value_depth)



