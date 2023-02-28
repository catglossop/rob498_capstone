import rospy
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import PoseStamped

# Callback handlers
def handle_launch():
    print('Launch Requested. Your drone should take off.')

def handle_test():
    print('Test Requested. Your drone should perform the required tasks. Recording starts now.')

def handle_land():
    print('Land Requested. Your drone should land.')

def handle_abort():
    print('Abort Requested. Your drone should land immediately due to safety considerations')

# Service callbacks
def callback_launch(request):
    handle_launch()
    return EmptyResponse()

def callback_test(request):
    handle_test()
    return EmptyResponse()

def callback_land(request):
    handle_land()
    return EmptyResponse()

def callback_abort(request):
    handle_abort()
    return EmptyResponse()

def callback_vicon(vicon_msg):
    pass

def vicon_running(topic_name='vicon/ROB498_Drone/ROB498_Drone'):
    # Get a list of tuples containing the names and data types of all the topics that are currently published
    published_topics = rospy.get_published_topics()

    # Check if the topic exists by searching for its name in the list of published topics
    if any(topic_name in topic for topic in published_topics):
        print(f"The topic '{topic_name}' exists, using vicon.")
        return True
    else:
        print(f"The topic '{topic_name}' does not exist, not using vicon.")
        return False

# Main communication node for ground control
def comm_node():
    print('This is a dummy drone node to test communication with the ground control')
    print('Do not change the node name and service topics! The TAs will test these service calls prior to flight')
    print('Your own code should be integrated into this node')
    
    rospy.init_node('rob498_drone') 
    srv_launch = rospy.Service('comm/launch', Empty, callback_launch)
    srv_test = rospy.Service('comm/test', Empty, callback_test)
    srv_land = rospy.Service('comm/land', Empty, callback_land)
    srv_abort = rospy.Service('comm/abort', Empty, callback_abort)

    if vicon_running():
        # Subscribe to the vicon topic /vicon/ROB498_Drone/ROB498_Drone
        rospy.Subscriber('vicon/ROB498_Drone/ROB498_Drone', PoseStamped, callback_vicon)
    else:
        # Use data from the RealSense camera
        pass

    rospy.spin()

if __name__ == "__main__":
    comm_node()