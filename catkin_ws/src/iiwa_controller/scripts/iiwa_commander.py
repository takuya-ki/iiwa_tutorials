#!/usr/bin/env python

from __future__ import print_function
import re
import socket
import threading
from datetime import datetime

import tf
import rospy
import tf2_ros
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from iiwa_controller.srv import (
    SetTargetPose, 
    SetTargetPoseResponse
)
from std_srvs.srv import (
    Trigger, 
    TriggerResponse
)

HOST_IP = "127.0.0.1" # ip address of localhost
PORT = 12345  # port used
CLIENTNUM = 3  # max number of the clients to be connected
DATESIZE = 1024  # bytes of the data handled


def euler_to_quaternion(euler):
    """Convert Euler Angles to Quaternion

    euler: geometry_msgs/Vector3
    quaternion: geometry_msgs/Quaternion
    """
    q = tf.transformations.quaternion_from_euler(
        euler.x, euler.y, euler.z)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


class SocketServer:
    def __init__(self, host, port):
        self.host = host
        self.port = port

    def conn_client(self, client_socket, address):
        while True:
            rcv_data = client_socket.recv(DATESIZE)  # receiving data from a client
            if rcv_data:
                rospy.loginfo('[{0}] recv date : {1}'.format(
                    datetime.now().strftime('%Y-%m-%d %H:%M:%S'), 
                    rcv_data.decode('utf-8')))
                if rcv_data.decode('utf-8') == 'initialize':
                    # initial pose alignment with a visual marker
                    try:
                        initial_pose_align = rospy.ServiceProxy('initial_pose_align', Trigger)
                        _success = False
                        while not _success:
                            rospy.loginfo("waiting for initial_pose_align service")
                            rospy.sleep(rospy.Duration(1.0)) # sec
                            res = initial_pose_align()
                            _success = res.success
                    except rospy.ServiceException, e:
                        rospy.loginfo("initial_pose_align call failed.")
                    rospy.sleep(rospy.Duration(5.0)) # sec
                    client_socket.send(('done_initialization').encode('utf-8'))

                elif rcv_data.decode('utf-8') == 'pp':
                    rospy.loginfo("move to pp")

                    # motions to pusg objects on a conveyor
                    try:
                        move_to_pp = rospy.ServiceProxy('move_to_pp', Trigger)
                        _success = False
                        while not _success:
                            rospy.loginfo("waiting for move_to_pp service")
                            rospy.sleep(rospy.Duration(1.0)) # sec
                            res = move_to_pp()
                            _success = res.success
                    except rospy.ServiceException, e:
                        rospy.loginfo("move_to_pp call failed.")
                    rospy.sleep(rospy.Duration(5.0)) # sec
                    client_socket.send(('done_move_to_pp').encode('utf-8'))

                else:
                    try:
                        # extract the target position data and store them into Vector3
                        x = float(re.findall('x=(.*),y=', rcv_data)[0])
                        y = float(re.findall('y=(.*),z=', rcv_data)[0])
                        z = float(re.findall('z=(.*)', rcv_data)[0])
                        rospy.loginfo("Received. "+'x='+str(x)+',y='+str(y)+',z='+str(z))
                    except:
                        print('Failed to parse command :',rcv_data)
                        break
                    try:
                        pose = Pose()
                        pose.position = Vector3(x, y, z)
                        pose.orientation = euler_to_quaternion(Vector3(0.0, 0.0, 0.0))
                        tcp_move_to_manipulate = rospy.ServiceProxy(
                            'iiwa/iiwa_tcp_move_to_manipulate/tcp_move_to_manipulate', SetTargetPose)
                        _success = False
                        while not _success:
                            rospy.loginfo("waiting for tcp_move_to_manipulate service")
                            rospy.sleep(rospy.Duration(1.0)) # sec
                            res = tcp_move_to_manipulate(target_pose = pose)
                            _success = res.success
                    except rospy.ServiceException, e:
                        rospy.loginfo("tcp_move_to_manipulate call failed")

                    send_msg = 'Finished to set ' + rcv_data.decode('utf-8')
                    send_msg = send_msg.encode('utf-8')
                    client_socket.send(send_msg)
            else:
                break
        client_socket.close()
        
        rospy.loginfo('[{0}] disconnect client -> address : {1}'.format(
            datetime.now().strftime('%Y-%m-%d %H:%M:%S'), address))
    
    def run_server(self):
        # bring up the server
        try:
            server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # Activate the server; this will keep running until you
            # interrupt the program with Ctrl-C
            server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server_socket.bind((self.host, self.port))
            server_socket.listen(CLIENTNUM)
            # server_socket.settimeout(None)
            rospy.loginfo('[{}] run server'.format(datetime.now().strftime('%Y-%m-%d %H:%M:%S')))
            
            while True:
                client_socket, address = server_socket.accept() # accept the connection request from a client
                rospy.loginfo('[{0}] connect client -> address : {1}'.format(
                    datetime.now().strftime('%Y-%m-%d %H:%M:%S'), address))
                client_socket.settimeout(60)
                t = threading.Thread(target=self.conn_client, args=(client_socket, address))
                t.setDaemon(True)
                t.start()

            server_socket.serve_forever()
        except:
            pass
        finally:
            server_socket.close()


if __name__ == '__main__':
    rospy.init_node('iiwa_commandar',anonymous=True)
    rospy.wait_for_service('initial_pose_align')
    rospy.wait_for_service('iiwa/iiwa_tcp_move_to_manipulate/tcp_move_to_manipulate')
    SocketServer(HOST_IP, PORT).run_server()
    rospy.spin()
