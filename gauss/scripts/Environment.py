import numpy as np
import rospy, time
from std_msgs.msg import String

class Environment(object):
    def __init__(self,ID):
        rospy.init_node('environment', anonymous=True)
        self.listener()
    
    def listener(self):
        rospy.Subscriber('/Enviroment/ADSB/raw', String, self.ADSB_msg_damager)

    def ADSB_msg_damager(self,data):
        msg = data.data
        self.ADSBsender(msg)

    def ADSBsender(self,msg):
        pub = rospy.Publisher('/Enviroment/ADSB/damaged', String, queue_size=10)
        pub.publish(msg)
        time.sleep(0.1)
