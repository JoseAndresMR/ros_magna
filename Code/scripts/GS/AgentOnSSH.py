#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from Various import *
import rospkg
import argparse



class AgentOnSSH(object):

    def __init__(self, ID, hostname, username, password, fcu_url, gcs_url):

        print(ID, hostname, username, password, fcu_url, gcs_url)
        port = 22
        ssh_connection = SSHConnection(hostname, username, password, port)

        home_path = rospkg.RosPack().get_path('magna')[:-5]
        yaml_local_file = home_path + "/Code/yaml/all_rosparams.yaml"
        yaml_remote_file = 'ros/magna_ws/src/magna/Code/yaml/all_rosparams.yaml'
        ssh_connection.transferFile(yaml_local_file, yaml_remote_file)

        command1 = "cd ros; source /home/safedrone2/.bashrc; source /opt/ros/kinetic/setup.bash; source ~/ros/magna_ws/devel/setup.bash"
        command2 = " roslaunch magna Aerial_Segment.launch id:={0} mode:=custom fcu_url:={1} gcs_url:={2}".format(ID, fcu_url, gcs_url, password)
        ssh_connection.executeCommand(command1 + "; " + command2)

        ssh_connection.closeConnection()


def main():
    parser = argparse.ArgumentParser(description='Spawn robot in Gazebo for SITL')
    parser.add_argument('-ID', type=str, default="1", help='')
    parser.add_argument('-hostname', type=str, default="1", help='')
    parser.add_argument('-username', type=str, default="1", help='')
    parser.add_argument('-password', type=str, default="1", help='')
    parser.add_argument('-fcu_url', type=str, default="1", help='')
    parser.add_argument('-gcs_url', type=str, default="1", help='')

    args, unknown = parser.parse_known_args()
    AgentOnSSH(args.ID, args.hostname, args.username, args.password, args.fcu_url, args.gcs_url)

    return

if __name__ == '__main__':
    main()