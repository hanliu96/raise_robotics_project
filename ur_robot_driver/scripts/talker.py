#!/usr/bin/env python3

from operator import le
import socket
import rospy
from ur_robot_driver.msg import leicaMeas
from std_msgs.msg import String

HOST = '192.168.1.103'  # The server's hostname or IP address
PORT = 1212        # The port used by the server


def talker():
    pub = rospy.Publisher('chatter', leicaMeas, queue_size=10)
    rospy.init_node('talker', anonymous=True)

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        while True:
            data = s.recv(1024)
            if not data:
                break
            # s.sendall(data)
            print('Received', repr(data))
            pub.publish(convertFormat(repr(data)))


def convertFormat(inputStr):
    """
    Convenience method for converting the Leica input data string into ROS message
    @param: inputStr       A string that holds a printable representation of an leica measurement object
    @returns: ROS message
    """
    leica_meas = leicaMeas()
    info = inputStr.split(",")

    leica_meas.pointId = int(info[0][2:])
    leica_meas.northing = float(info[1])
    leica_meas.easting = float(info[2])
    leica_meas.height = float(info[3])
    leica_meas.hz_angle = float(info[4])
    leica_meas.v_angle = float(info[5])
    leica_meas.sd = float(info[6])
    leica_meas.EDM_kind = int(info[7])
    leica_meas.EDM_mode = int(info[8])

    return leica_meas


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass