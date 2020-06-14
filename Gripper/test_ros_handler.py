"""
Author: Konstantinos Angelopoulos
Date: 04/02/2020
All rights reserved.
Feel free to use and modify and if you like it give it a star.

Same as the ros_handler.py but does not initialize ROS node and main node.
Only for debugging and testing the communication without even connecting the gripper.
See the ros_handler.py for more details on functionality.
"""

#!/usr/bin/env python
import os

# Import ROS libraries only on Linux
if os.name == 'nt':
    from Gripper import encryption
else:
    import rospy
    from std_srvs.srv import Empty
    from reflex_one_msgs.msg import Command
    from reflex_one_msgs.msg import PoseCommand
    from reflex_one_msgs.msg import VelocityCommand
    from reflex_one_msgs.msg import Hand
    from reflex_one_msgs.msg import Motor
    import encryption

import socket
import time
import threading


# Create Server to listen for Clients
class Server(object):

    def __init__(self, ip, port, encrypt_flag=False):
        self.IP = ip
        self.Port = port
        self.bufferSize = 8192
        self.msg = str.encode(" Hello UDP Client ")
        self.msg_exit = str.encode(" Closing server...\n Exit with code 0. ")
        # Create a datagram socket
        self._socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        # Bind to address and ip
        self._socket.bind((self.IP, self.Port))
        self.clients = []
        self.client_list_key = 'root'
        self._done = False
        self._match = False
        self._ros_started = False
        self._encryption = encrypt_flag
        self._motor_load = Motor()
        self.start()  # Initialize ROS
        # rospy.init_node('GripperServer')
        self._calibrate_service = rospy.ServiceProxy('/reflex_one/calibrate_fingers', Empty)
        self._command_publisher = rospy.Publisher('/reflex_one/command', Command, queue_size=1)
        self._pose_publisher = rospy.Publisher('/reflex_one/command_position', PoseCommand, queue_size=1)
        self._vel_publisher = rospy.Publisher('/reflex_one/command_velocity', VelocityCommand, queue_size=1)
        self._hand_state = Hand()
        # Constantly capture the current hand state
        rospy.Subscriber('/reflex_one/hand_state', Hand, self.hand_state_update)
        rospy.Subscriber('/reflex_one/hand_state/motor', Motor, self.motor_load_update)

    def motor_load_update(self, data):
        self._motor_load = data

    def hand_state_update(self, data):
        self._hand_state = data

    def approach_fabric(self):
        motor_loads = [float(x.load) for x in self._hand_state.motor[:]]
        step, position = 0.05, 0.0
        for i in range(40):
            if abs(motor_loads[0]) >= 0.049 and abs(motor_loads[1]) >= 0.049:
                print('Fabric Reached')
                break
            motor_loads = [float(x.load) for x in self._hand_state.motor[:]]
            position += step
            self._pose_publisher.publish(PoseCommand(f1=position, f2=position))
            rospy.sleep(.75)
            print("%.3f" % abs(motor_loads[0]), "%.3f" % abs(motor_loads[1]))
        self._pose_publisher.publish(PoseCommand(f1=position, f2=position, f3=1.85))
        rospy.sleep(1.25)
        self._pose_publisher.publish(PoseCommand(f1=position, f2=position, f3=1.85, preshape1=.8, preshape2=.8))
        rospy.sleep(1.25)
        self._pose_publisher.publish(PoseCommand(f1=2.0, f2=2.0, f3=1.9, preshape1=.8, preshape2=.8))

    def start_thread(self, cmd):
        thread_command = "gnome-terminal --geometry 60x10+50+50 -e 'bash -c \"{} bash\" '".format(cmd)
        # thread_command = "gnome-terminal -e 'bash -c \"" + cmd + " bash\" '"
        os.system(thread_command)

    def send(self, msg, address):
        if self._encryption:
            msg = encryption.encrypt(msg)
        self._socket.sendto(msg, address)

    def receive(self):
        bytes_received = self._socket.recvfrom(self.bufferSize)
        if self._encryption:
            return [encryption.decrypt(bytes_received[0]), bytes_received[1]]
        return bytes_received

    # Listen for clients
    def listen(self):
        print("[GRIPPER SERVER]: UDP server up and listening ")
        while not self._done:
            bytes_received = self.receive()
            message = bytes_received[0]
            address = bytes_received[1]
            client_msg = "[GRIPPER SERVER]: Message from Client {}: {} ".format(address, message.decode('utf-8'))
            temp_message = message.decode('utf-8')
            temp_message = temp_message.lower()
            if temp_message != 'hello':
                self.send(str.encode('Please start with Hello as the first command...'), address)
                continue
            if len(self.clients) >= 16:
                self.clients = []
            self._match = False
            if len(self.clients) > 0:
                for client in range(len(self.clients)):
                    if self.clients[client][0] == address[0] and self.clients[client][1] == address[1]:
                        self._match = True
                        break
                if not self._match:
                    self.clients.append([address[0], address[1]])
            else:
                self.clients.append([address[0], address[1]])
            print(client_msg)
            # Sending a reply to client
            self.send(self.msg, address)
            # Waiting for command
            command = self.receive()
            message = command[0]
            address = command[1]
            # make message lowercase to handle type errors
            message = message.lower()
            if message.decode('utf-8') == 'exit' or message.decode('utf-8') == 'quit' or message.decode('utf-8') == 'close' or message.decode('utf-8') == 'terminate' or message.decode('utf-8') == 'kill':
                try:
                    self.send(self.msg_exit, address)
                    exit_ros = threading.Thread(target=self.start_thread, args=('killall gnome-terminal',))
                    exit_ros.start()
                    print('[GRIPPER SERVER]: {}'.format(self.msg_exit.decode('utf-8')))
                    self.close()
                except Exception as e:
                    self.send(str.encode(str(e)), address)
            elif message.decode('utf-8') == 'ros_status' or message.decode('utf-8') == 'status_ros' or message.decode('utf-8') == 'ros' or message.decode('utf-8') == 'active' or message.decode('utf-8') == 'status':
                try:
                    """ Server Logic should start here """
                    # Send the status of the ROS if the start command was send
                    if self._ros_started:
                        self.send(str.encode("ACTIVE"), address)
                    else:
                        self.send(str.encode('INACTIVE'), address)
                except Exception as e:
                    self.send(str.encode(str(e)), address)
            elif message.decode('utf-8') == 'start' or message.decode('utf-8') == 'begin':
                try:
                    """ Server Logic should start here """
                    # Initialize ROS in a different terminal with a thread so as the server can continue to communicate
                    start_ros = threading.Thread(target=self.start_thread, args=('./start.sh',))
                    start_ros.start()  # We start the thread that will run in loop
                    time.sleep(20)
                    self._ros_started = True
                    self.send(str.encode(" Started ROS... You can publish... "), address)
                except Exception as e:
                    self.send(str.encode(str(e)), address)
            elif message.decode('utf-8') == 'help' or message.decode('utf-8') == 'info' or message.decode('utf-8') == 'commands' or message.decode('utf-8') == 'command':
                try:
                    # Write help message like this to appear better
                    help_message = """
 =======================================================================================
 ------------------------------- Available commands ------------------------------------
 =======================================================================================
 Always start with Hello and after you receive the answer from server type the command.
 All commands are transformed to lower letters so they can be written in any form as long
 as the grammar is correct.
 Commands:
     help/info/commands/command:  -Returns the available options for the server.
     start/begin: -Starts the ROS reflex_one.launch file to initialize the gripper communication.
     status_ros/ros_status/ros/active/status: -Returns ACTIVE if the roslaunch has started or STOPPED INACTIVE if it hasn't.
     visualizer/visualize/visual/simulate/sim/simulation: -Start the ROS rviz simulation of the gripper.
     exit/quit/close/terminate/kill: -Closes all terminal and terminates the UDP server listening.
     clients_list/clients/client_list/list: -Returns the list of clients (ip and port) that have communicated with the server and their. After this command the server waits
                                             for the key to be send in order to send you the client list.
     last_client/last: -Returns the last client (IP and Port) that interacted with the Server. After this command the server waits for the key to be send in order to send you the client list.

     publish/post/pub: -After sending the command post or publish the server waits for message that must be posted to the rostopic that listens for the gripper positions.
                        Available commands after post/publish are:
                            open: Opens the Gripper to the zero position as specified in the calibration process see: https://www.labs.righthandrobotics.com/calibrate-fingers.
                            close: Closes the Gripper like a fist.
                            position/pos: Publishes a specific position for the gripper to move to. After sending the position command the server waits for the position values.
                                          The values must be in the following specific format including the brackets:
                                              {f1: 0.0, f2: 0.0, f3: 0.0, preshape1: 0.0, preshape2: 0.0}
                                              the f1/f2/f3 corresponds to the finger positions and the preshape1/2 to the finger rotations. All values are in radians!!!
                            velocity/vel: Publishes the new velocities for the robotic finger motors.After sending the velocity command the server waits for the velocity values.
                                          The values must be in the following specific format including the brackets:
                                              {f1: 0.0, f2: 0.0, f3: 0.0, preshape1: 0.0, preshape2: 0.0}. All values are in radians!!!
                            calibrate/calib/cal: Starts the calibration for the gripper fingers home position.
                            grasp/catch: Initializes a grasping method.
                            hand/state: Returns the state of the 3 robotic fingers.
                            motor/load: Returns the motor loads of the 3 robotic fingers.
                            clients_list/clients/client_list/list: -Returns the list of clients (ip and port) that have communicated with the server and their. After this command the server waits
                                             for the key to be send in order to send you the client list.
                            last_client/last: -Returns the last client (IP and Port) that interacted with the Server. After this command the server waits for the key to be send in order to send you the client list.

     Example to communicate with the server using the Client class:
         from gripper import Client
         client = Client('192.168.56.2', 20000)  # Initialize the client using the IP and port of the server
         client.send('Hello')  # Always send hello
         client.receive()  # Receive respond
         # Start reflex_one.launch file with ROS to communicate with the gripper (Assuming that the gripper is connected to the server.
         # See: https://www.labs.righthandrobotics.com/reflex-quickstart on how to install the Gripper to the Server
         client.send('start')
         # Always listen for responds after sending commands
         a.receive()
         time.sleep(15)  # Wait for the ROS to initialize
         a.send('Hello')
         a.receive()
         a.send('post')
         a.send('open')
         a.receive()
         a.send('Hello')
         a.receive()
         a.send('exit')
         a.receive()
         """
                    self.send(str.encode(help_message), address)
                except Exception as e:
                    self.send(str.encode(str(e)), address)
            elif message.decode('utf-8') == 'visualize' or message.decode('utf-8') == 'visualizer' or message.decode('utf-8') == 'visual' or message.decode('utf-8') == 'simulate' or message.decode('utf-8') == 'sim' or message.decode('utf-8') == 'simulation':
                try:
                    publish_ros = threading.Thread(target=self.start_thread, args=('./visualize.sh',))
                    publish_ros.start()
                    publish_ros.join()  # Wait for it to stop
                    self.send(str.encode(' Started RVIZ visualizer... '), address)
                except Exception as e:
                    self.send(str.encode(str(e)), address)
            elif message.decode('utf-8') == 'publish' or message.decode('utf-8') == 'post' or message.decode('utf-8') == 'pub':
                bytes_received = self.receive()
                message = bytes_received[0]
                message = message.lower()
                if message.decode('utf-8') == 'calibrate' or message.decode('utf-8') == 'calib' or message.decode('utf-8') == 'cal':
                    try:
                        """
                        Calibration needs to execute in a different terminal on a different thread
                        self.send(str.encode(' Started Calibration... Switch to ROS machine to calibrate. '), address)
                        self._calibrate_service()
                        """
                        calib_ros = threading.Thread(target=self.start_thread, args=('./calibrate.sh',))
                        calib_ros.start()
                        self.send(str.encode(' Started Calibration... Switch to ROS machine to calibrate. '), address)
                    except Exception as e:
                        self.send(str.encode(str(e)), address)
                elif message.decode('utf-8') == 'grasp' or message.decode('utf-8') == 'catch':
                    try:
                        start_approach_fabric = threading.Thread(target=self.approach_fabric)
                        start_approach_fabric.start()
                        start_approach_fabric.join()
                        self.send(str.encode(' Grasped Cloth '), address)
                    except Exception as e:
                        self.send(str.encode(str(e)), address)
                elif message.decode('utf-8') == 'motor' or message.decode('utf-8') == 'load':
                    try:
                        data_load = ','.join(str(x.load) for x in self._hand_state.motor[:])
                        self.send(str.encode('{}'.format(data_load)), address)
                    except Exception as e:
                        self.send(str.encode(str(e)), address)
                elif message.decode('utf-8') == 'hand' or message.decode('utf-8') == 'state':
                    try:
                        self.send(str.encode('{}'.format(self._hand_state)), address)
                    except Exception as e:
                        self.send(str.encode(str(e)), address)
                elif message.decode('utf-8') == 'open':
                    try:
                        self._pose_publisher.publish(PoseCommand(f1=0.0, f2=0.0, f3=0.0, preshape1=0.0, preshape2=0.0))
                        rospy.sleep(0.25)
                        self.send(str.encode(' Opened Gripper '), address)
                    except Exception as e:
                        self.send(str.encode(str(e)), address)
                elif message.decode('utf-8') == 'close':
                    try:
                        self._pose_publisher.publish(PoseCommand(f1=3.4, f2=3.4, f3=3.4, preshape1=0.0, preshape2=0.0))
                        rospy.sleep(0.25)
                        self.send(str.encode(' Closed Gripper '), address)
                    except Exception as e:
                        self.send(str.encode(str(e)), address)
                elif message.decode('utf-8') == 'velocity' or message.decode('utf-8') == 'vel':
                    try:
                        bytes_received = self.receive()
                        message = bytes_received[0]
                        message = message.decode('utf-8')
                        pub = [float(x) for x in message.split(',')]
                        self._vel_publisher.publish(VelocityCommand(f1=pub[0], f2=pub[1], f3=pub[2], preshape1=pub[3], preshape2=pub[4]))
                        rospy.sleep(0.25)
                        self.send(str.encode(' Moved Gripper '), address)
                    except Exception as e:
                        self.send(str.encode(str(e)), address)
                elif message.decode('utf-8') == 'position' or message.decode('utf-8') == 'pos':
                    try:
                        bytes_received = self.receive()
                        message = bytes_received[0]
                        message = message.decode('utf-8')
                        pub = [float(x) for x in message.split(',')]
                        self._pose_publisher.publish(PoseCommand(f1=pub[0], f2=pub[1], f3=pub[2], preshape1=pub[3], preshape2=pub[4]))
                        rospy.sleep(0.25)
                        self.send(str.encode(' Moved Gripper '), address)
                    except Exception as e:
                        self.send(str.encode(str(e)), address)
            elif message.decode('utf-8') == 'clients_list' or message.decode('utf-8') == 'clients' or message.decode('utf-8') == 'client_list' or message.decode('utf-8') == 'list':
                try:
                    # Wait for the correct key
                    bytes_received = self.receive()
                    message = bytes_received[0]
                    message = message.decode('utf-8')
                    message = message.lower()
                    if message != self.client_list_key:
                        self.send(str.encode('Wrong Key'), address)
                        continue
                    print('[GRIPPER SERVER]: Sending Client List..... ')
                    cl_msg = ' \n------------- Client List -------------\n '
                    for id_number, client in enumerate(self.clients):
                        cl_msg += ' Client: ID = {}, IP = {}, Port = {}\n '.format(id_number, client[0], client[1])
                    self.send(str.encode(cl_msg), address)
                except Exception as e:
                    self.send(str.encode(str(e)), address)
            elif message.decode('utf-8') == 'last_client' or message.decode('utf-8') == 'last':
                try:
                    # Wait for the correct key
                    bytes_received = self.receive()
                    message = bytes_received[0]
                    address = bytes_received[1]
                    message = message.decode('utf-8')
                    message = message.lower()
                    if message != self.client_list_key:
                        self.send(str.encode('Wrong Key'), address)
                        continue
                    print('[GRIPPER SERVER]: Sending Last Client to: {} {} '.format(address[0], address[1]))
                    last_client = ' Last Client: ID = {}, IP = {}, Port = {}\n '.format(len(self.clients) - 1, self.clients[-1][0], self.clients[-1][1])
                    self.send(str.encode(last_client), address)
                except Exception as e:
                    self.send(str.encode(str(e)), address)
            else:
                try:
                    self.send(str.encode('Send the Hello message and then send help or info or command or commands to learn how to communicate with the server'), address)
                except Exception as e:
                    self.send(str.encode(str(e)), address)

    def start(self):
        # Initialize ROS in a different terminal with a thread so as the server can continue to communicate
        start_ros = threading.Thread(target=self.start_thread, args=('./start.sh',))
        start_ros.start()  # We start the thread that will run in loop
        time.sleep(20)
        self._ros_started = True

    # Close server and exit
    def close(self):
        self._done = True
        self._socket.close()


# Create Client to communicate with Server
class Client(object):

    def __init__(self, ip, port, encrypt_flag=False):

        self.bufferSize = 8192
        # Create a UDP socket at client side
        self._socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self._done = False
        self.server_ip = ip
        self.server_port = port
        self._encryption = encrypt_flag

    def send(self, msg):
        # Send to server using created UDP socket
        if self._encryption:
            msg = encryption.encrypt(str.encode(msg))
            self._socket.sendto(msg, (self.server_ip, self.server_port))
        else:
            self._socket.sendto(str.encode(msg), (self.server_ip, self.server_port))

    def receive(self, ret=False):
        msg_received = self._socket.recvfrom(self.bufferSize)
        if self._encryption:
            if not ret:
                msg_received = encryption.decrypt(msg_received[0])
                print("[GRIPPER CLIENT]: Message from Server: {} ".format(msg_received.decode('utf-8')))
            else:
                msg_received = encryption.decrypt(msg_received[0])
                return msg_received.decode('utf-8')
        else:
            if not ret:
                print("[GRIPPER CLIENT]: Message from Server: {} ".format(msg_received[0].decode('utf-8')))
            else:
                return msg_received[0].decode('utf-8')

    def start_grasping(self, RDK):
        from robodk.robodk import Pose_2_KUKA, KUKA_2_Pose, tr
        import numpy as np
        robot = RDK.Item('KUKA')
        pose = RDK.Item('Target')
        motor_loads, step, position = [0, 0, 0, 0, 0], 0.1, 0.0
        ErrorHandled = False
        while not ErrorHandled:
            # Move fingers until they reach the fabric
            while (abs(motor_loads[0]) + abs(motor_loads[1])) <= 0.16:
                position += step
                self.send('Hello')
                _ = self.receive(ret=True)
                self.send('pub')
                self.send('position')
                self.send('{}, {}, 0.0, 0.0, 0.0'.format(position, position))
                # self.send('1.65, 1.55, 0.0, 0.0, 0.0')
                # self.send('1.33, 1.29, 0.0, 0.0, 0.0')  # 90 tool distance
                # self.send('1.44, 1.39, 0.0, 0.0, 0.0')  # 95 tool distance
                self.receive()
                self.send('Hello')
                _ = self.receive(ret=True)
                self.send('pub')
                self.send('load')
                message = self.receive(ret=True)
                motor_loads = [float(x) for x in message.split(',')]
                print(motor_loads[:2])
            measured_height = np.arccos(90 - position) * 105
            if 11 <= measured_height <= 10:
                """
                height = Pose_2_KUKA(robot.Pose())[2] - 200
                joints = tr(robot.Joints())
                robot.setJoints([joints[0, 0], joints[0, 1], joints[0, 6], joints[0, 2], joints[0, 3], joints[0, 4], joints[0, 5]])
                """
                target = Pose_2_KUKA(pose.Pose())
                pose.setPose(KUKA_2_Pose((target[0], target[1], target[2] - height, target[3], target[4], target[5])))
                robot.MoveJ(pose)

        self.send('Hello')
        _ = self.receive(ret=True)
        self.send('pub')
        self.send('position')
        # self.send('{}, {}, 0.0, 0.0, 0.0'.format(position, position))
        self.send('1.65, 1.55, 0.0, 0.0, 0.0')
        # self.send('1.33, 1.29, 0.0, 0.0, 0.0')  # 90 tool distance
        # self.send('1.44, 1.39, 0.0, 0.0, 0.0')  # 95 tool distance
        time.sleep(1.0)
        self.send('Hello')
        _ = self.receive(ret=True)
        self.send('pub')
        self.send('pos')
        # self.send('{}, {}, 1.85, 0.0, 0.0'.format(position, position))
        self.send('1.65, 1.55, 1.85, 0.0, 0.0')
        # self.send('1.65, 1.55, 1.29, 0.0, 0.0')
        self.receive()
        time.sleep(1)
        self.send('Hello')
        _ = self.receive(ret=True)
        self.send('pub')
        self.send('pos')
        # self.send('{}, {}, 1.85, 0.8, 0.8'.format(position, position))
        self.send('1.65, 1.55, 1.85, 1.0, 1.0')
        self.receive()
        time.sleep(1)
        self.send('Hello')
        _ = self.receive(ret=True)
        self.send('pub')
        self.send('pos')
        self.send('2.15, 2.15, 1.9, 0.8, 0.8')
        # self.send('2.0, 2.0, 1.85, 0.0, 1.0'.format(position, position))
        self.receive()
        time.sleep(1)

    # Only for when using the .sh files to execute the commands (SLower) see (Gripper.gripper for Ubuntu Indigo)
    def format_position_message(self, f1, f2, f3, p1, p2, preshape_only=False):
        if preshape_only:
            return '{}preshape1: {}, preshape2: {}{}'.format('"{', float(p1), float(p2), '}"')
        """ Position Message must be in this format {f1: 0.0, f2: 0.0, f3: 0.0, preshape1: 0.0, preshape2: 0.0} """
        return '{}f1: {}, f2: {}, f3: {}, preshape1: {}, preshape2: {}{}'.format('"{', float(f1), float(f2), float(f3), float(p1), float(p2), '}"')

    # Close client and exit
    def close(self):
        self._done = True
        self._socket.close()


def graspModel(RDK, daq, gripper_client):
    """
    First Grasping technique for correcting the Gripper's height
    :param RDK: Connection to the RoboDK instance
    :param daq: ATI FT Sensor Client instance
    :param gripper_client: Gripper Client instance
    :return Boolean
    """
    import time
    import numpy as np
    from robodk.robodk import Pose_2_KUKA, KUKA_2_Pose, tr
    ORIGINAL_LOWER_LIMITS = [-170, -30, -170, -120, -170, -120, -170]
    ORIGINAL_UPPER_LIMITS = [170, 210, 170, 120, 170, 120, 170]
    MODIFIED_LOWER_LIMITS = [-170, -30, -170, -170, -170, -170, -170]
    MODIFIED_UPPER_LIMITS = [170, 210, 170, 170, 170, 170, 170]
    robot = RDK.Item('KUKA')
    pose = RDK.Item('Target')
    ErrorHandled = False
    desired_height = 118
    last_position = None
    while not ErrorHandled:
        forces_history = []
        motor_loads, step, position = [0, 0, 0, 0, 0], 0.08, 0.0
        if last_position is not None:
            position = last_position
        gripper_client.send('Hello')
        _ = gripper_client.receive(ret=True)
        gripper_client.send('pub')
        gripper_client.send('position')
        gripper_client.send('{}, {}, 0.0, 0.0, 0.0'.format(position, position))
        time.sleep(2)
        # Zero the sensor values
        daq.send('hello')
        daq.receive()
        daq.send('bias')
        daq.receive()
        # Move fingers until they reach the fabric
        while True:
            gripper_client.send('Hello')
            _ = gripper_client.receive(ret=True)
            gripper_client.send('pub')
            gripper_client.send('position')
            gripper_client.send('{}, {}, 0.0, 0.0, 0.0'.format(position, position))
            time.sleep(.6)
            daq.send('hello')
            m = daq.receive(ret=True)
            daq.send('forces')
            forces = daq.receive(ret=True)
            try:
                forces = [float(x) for x in forces.split(',')]
            except Exception as e:
                """
                Catch exception where server has not initialized and fixed the correct
                sequence for the ATI FT Sensor and fixing sequence is transmitted.
                """
                print(f'[ATI FT CLIENT]: SERVER NOT INITIALISED \n {e}')
                forces = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            position += step
            if (90 * np.pi / 180) <= position <= 95 * np.pi / 180:
                ErrorHandled = True
                break
            if len(forces_history) >= 2:
                if abs(forces_history[-1] - forces_history[-2]) >= .6:
                    break
            print(forces[0], forces[1], forces[2])
            print(position)
            forces_history.append(forces[2])
            # time.sleep(.8)
        gripper_client.send('Hello')
        _ = gripper_client.receive(ret=True)
        gripper_client.send('pub')
        gripper_client.send('position')
        gripper_client.send('{}, {}, 0.0, 0.0, 0.0'.format(0, 0))
        measured_height = np.cos((np.pi / 2) - position - 3 * step) * 110
        last_position = position - 3 * step
        if measured_height >= desired_height and not ErrorHandled:
            target = Pose_2_KUKA(pose.PoseAbs())
            pose.setPoseAbs(KUKA_2_Pose((target[0], target[1], target[2] - (measured_height - desired_height), target[3], target[4], target[5])))
            # change robot's joint limits to the original ones to avoid joint limitations
            robot.setJointLimits(MODIFIED_LOWER_LIMITS, MODIFIED_UPPER_LIMITS)
            tar = tr(pose.Joints())
            """ 
                Change the position of the Joints because KUKA LWR IV+ has declared the third joint as an External Joint.
                KUKA LWR IV+ Joints = [A1, A2, E1, A3, A4, A5, A6] and RoboDK sends the joints as [A1, A2, A3, A4, A5, A6, E1]
            """
            robot.MoveJ([tar[0, 0], tar[0, 1], tar[0, 3], tar[0, 4], tar[0, 5], tar[0, 6], tar[0, 2]])
            # change robot's joint limits to the modified ones to compute the real joint angles
            robot.setJointLimits(ORIGINAL_LOWER_LIMITS, ORIGINAL_UPPER_LIMITS)
        elif measured_height <= desired_height and not ErrorHandled:
            target = Pose_2_KUKA(pose.PoseAbs())
            pose.setPoseAbs(KUKA_2_Pose((target[0], target[1], target[2] + (desired_height - measured_height), target[3], target[4], target[5])))
            # change robot's joint limits to the original ones to avoid joint limitations
            robot.setJointLimits(MODIFIED_LOWER_LIMITS, MODIFIED_UPPER_LIMITS)
            tar = tr(pose.Joints())
            """ 
                Change the position of the Joints because KUKA LWR IV+ has declared the third joint as an External Joint.
                KUKA LWR IV+ Joints = [A1, A2, E1, A3, A4, A5, A6] and RoboDK sends the joints as [A1, A2, A3, A4, A5, A6, E1]
            """
            robot.MoveJ([tar[0, 0], tar[0, 1], tar[0, 3], tar[0, 4], tar[0, 5], tar[0, 6], tar[0, 2]])
            # change robot's joint limits to the modified ones to compute the real joint angles
            robot.setJointLimits(ORIGINAL_LOWER_LIMITS, ORIGINAL_UPPER_LIMITS)

    target = Pose_2_KUKA(pose.PoseAbs())
    pose.setPoseAbs(KUKA_2_Pose((target[0], target[1], target[2] + 6.5, target[3], target[4], target[5])))
    # change robot's joint limits to the original ones to avoid joint limitations
    robot.setJointLimits(MODIFIED_LOWER_LIMITS, MODIFIED_UPPER_LIMITS)
    tar = tr(pose.Joints())
    """ 
        Change the position of the Joints because KUKA LWR IV+ has declared the third joint as an External Joint.
        KUKA LWR IV+ Joints = [A1, A2, E1, A3, A4, A5, A6] and RoboDK sends the joints as [A1, A2, A3, A4, A5, A6, E1]
    """
    robot.MoveJ([tar[0, 0], tar[0, 1], tar[0, 3], tar[0, 4], tar[0, 5], tar[0, 6], tar[0, 2]])
    # change robot's joint limits to the modified ones to compute the real joint angles
    robot.setJointLimits(ORIGINAL_LOWER_LIMITS, ORIGINAL_UPPER_LIMITS)
    gripper_client.send('Hello')
    _ = gripper_client.receive(ret=True)
    gripper_client.send('pub')
    gripper_client.send('pos')
    gripper_client.send('{}, {}, 0.0, 0.0, 0.0'.format(position, position))
    gripper_client.receive()
    time.sleep(2)
    gripper_client.send('Hello')
    _ = gripper_client.receive(ret=True)
    gripper_client.send('pub')
    gripper_client.send('pos')
    gripper_client.send('{}, {}, 1.85, 0.0, 0.0'.format(position, position))
    gripper_client.receive()
    time.sleep(2)
    gripper_client.send('Hello')
    _ = gripper_client.receive(ret=True)
    gripper_client.send('pub')
    gripper_client.send('pos')
    gripper_client.send('{}, {}, 1.85, 0.9, 0.9'.format(position, position))
    gripper_client.receive()
    time.sleep(2)
    gripper_client.send('Hello')
    _ = gripper_client.receive(ret=True)
    gripper_client.send('pub')
    gripper_client.send('pos')
    gripper_client.send('2.25, 2.25, 2.1, 0.9, 0.9')
    gripper_client.receive()
    time.sleep(2)
    target = Pose_2_KUKA(pose.PoseAbs())
    pose.setPoseAbs(KUKA_2_Pose((target[0], target[1], target[2] + 30, target[3], target[4], target[5])))
    # change robot's joint limits to the original ones to avoid joint limitations
    robot.setJointLimits(MODIFIED_LOWER_LIMITS, MODIFIED_UPPER_LIMITS)
    tar = tr(pose.Joints())
    """ 
        Change the position of the Joints because KUKA LWR IV+ has declared the third joint as an External Joint.
        KUKA LWR IV+ Joints = [A1, A2, E1, A3, A4, A5, A6] and RoboDK sends the joints as [A1, A2, A3, A4, A5, A6, E1]
    """
    robot.MoveJ([tar[0, 0], tar[0, 1], tar[0, 3], tar[0, 4], tar[0, 5], tar[0, 6], tar[0, 2]])
    # change robot's joint limits to the modified ones to compute the real joint angles
    robot.setJointLimits(ORIGINAL_LOWER_LIMITS, ORIGINAL_UPPER_LIMITS)
    return True


def touch(RDK, daq, gripper_client):
    """
    Second Grasping technique for correcting the Gripper's height
    :param RDK: Connection to the RoboDK instance
    :param daq: ATI FT Sensor Client instance
    :param gripper_client: Gripper Client instance
    :return Boolean
    """
    from robodk.robodk import Pose_2_KUKA, KUKA_2_Pose, tr
    import numpy as np
    import time

    ORIGINAL_LOWER_LIMITS = [-170, -30, -170, -120, -170, -120, -170]
    ORIGINAL_UPPER_LIMITS = [170, 210, 170, 120, 170, 120, 170]
    MODIFIED_LOWER_LIMITS = [-170, -30, -170, -170, -170, -170, -170]
    MODIFIED_UPPER_LIMITS = [170, 210, 170, 170, 170, 170, 170]

    robot = RDK.Item('KUKA')
    pose = RDK.Item('Target')
    target = Pose_2_KUKA(pose.PoseAbs())
    pose.setPoseAbs(KUKA_2_Pose((target[0], target[1], target[2] + 80, target[3], target[4], target[5])))
    # change robot's joint limits to the original ones to avoid joint limitations
    robot.setJointLimits(MODIFIED_LOWER_LIMITS, MODIFIED_UPPER_LIMITS)
    tar = tr(pose.Joints())
    """ 
        Change the position of the Joints because KUKA LWR IV+ has declared the third joint as an External Joint.
        KUKA LWR IV+ Joints = [A1, A2, E1, A3, A4, A5, A6] and RoboDK sends the joints as [A1, A2, A3, A4, A5, A6, E1]
    """
    robot.MoveJ([tar[0, 0], tar[0, 1], tar[0, 3], tar[0, 4], tar[0, 5], tar[0, 6], tar[0, 2]])
    # change robot's joint limits to the modified ones to compute the real joint angles
    robot.setJointLimits(ORIGINAL_LOWER_LIMITS, ORIGINAL_UPPER_LIMITS)
    ErrorHandled = False
    while not ErrorHandled:
        forces_history = []
        step, position = 3, np.pi / 2
        gripper_client.send('Hello')
        _ = gripper_client.receive(ret=True)
        gripper_client.send('pub')
        gripper_client.send('position')
        gripper_client.send('{}, 0.0, 0.0, 0.0, 0.0'.format(position))
        time.sleep(2)
        # Zero the sensor values
        daq.send('hello')
        daq.receive()
        daq.send('bias')
        daq.receive()
        # Move fingers until they reach the fabric
        while True:
            target = Pose_2_KUKA(pose.PoseAbs())
            pose.setPoseAbs(KUKA_2_Pose((target[0], target[1], target[2] - step, target[3], target[4], target[5])))
            # change robot's joint limits to the original ones to avoid joint limitations
            robot.setJointLimits(MODIFIED_LOWER_LIMITS, MODIFIED_UPPER_LIMITS)
            tar = tr(pose.Joints())
            """ 
                Change the position of the Joints because KUKA LWR IV+ has declared the third joint as an External Joint.
                KUKA LWR IV+ Joints = [A1, A2, E1, A3, A4, A5, A6] and RoboDK sends the joints as [A1, A2, A3, A4, A5, A6, E1]
            """
            robot.MoveJ([tar[0, 0], tar[0, 1], tar[0, 3], tar[0, 4], tar[0, 5], tar[0, 6], tar[0, 2]])
            # change robot's joint limits to the modified ones to compute the real joint angles
            robot.setJointLimits(ORIGINAL_LOWER_LIMITS, ORIGINAL_UPPER_LIMITS)
            time.sleep(1)

            daq.send('hello')
            m = daq.receive(ret=True)
            daq.send('forces')
            forces = daq.receive(ret=True)
            try:
                forces = [float(x) for x in forces.split(',')]
            except Exception as e:
                """
                Catch exception where server has not initialized and fixed the correct
                sequence for the ATI FT Sensor and fixing sequence is transmitted.
                """
                print(f'[ATI FT CLIENT]: SERVER NOT INITIALISED \n {e}')
                forces = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            if len(forces_history) >= 2:
                if abs(forces_history[-1] - forces_history[-2]) >= .6:
                    ErrorHandled = True
                    break
            print(forces[0], forces[1], forces[2])
            print(target[2] - step)
            forces_history.append(forces[2])
            # time.sleep(.8)

    target = Pose_2_KUKA(pose.PoseAbs())
    pose.setPoseAbs(KUKA_2_Pose((target[0], target[1], target[2] + 6.5, target[3], target[4], target[5])))
    # change robot's joint limits to the original ones to avoid joint limitations
    robot.setJointLimits(MODIFIED_LOWER_LIMITS, MODIFIED_UPPER_LIMITS)
    tar = tr(pose.Joints())
    """ 
        Change the position of the Joints because KUKA LWR IV+ has declared the third joint as an External Joint.
        KUKA LWR IV+ Joints = [A1, A2, E1, A3, A4, A5, A6] and RoboDK sends the joints as [A1, A2, A3, A4, A5, A6, E1]
    """
    robot.MoveJ([tar[0, 0], tar[0, 1], tar[0, 3], tar[0, 4], tar[0, 5], tar[0, 6], tar[0, 2]])
    # change robot's joint limits to the modified ones to compute the real joint angles
    robot.setJointLimits(ORIGINAL_LOWER_LIMITS, ORIGINAL_UPPER_LIMITS)
    gripper_client.send('Hello')
    _ = gripper_client.receive(ret=True)
    gripper_client.send('pub')
    gripper_client.send('position')
    gripper_client.send('{}, {}, 0.0, 0.0, 0.0'.format(0, 0))
    gripper_client.send('Hello')
    target = Pose_2_KUKA(pose.PoseAbs())
    pose.setPoseAbs(KUKA_2_Pose((target[0], target[1], target[2] - 5.0, target[3], target[4], target[5])))
    # change robot's joint limits to the original ones to avoid joint limitations
    robot.setJointLimits(MODIFIED_LOWER_LIMITS, MODIFIED_UPPER_LIMITS)
    tar = tr(pose.Joints())
    """ 
        Change the position of the Joints because KUKA LWR IV+ has declared the third joint as an External Joint.
        KUKA LWR IV+ Joints = [A1, A2, E1, A3, A4, A5, A6] and RoboDK sends the joints as [A1, A2, A3, A4, A5, A6, E1]
    """
    robot.MoveJ([tar[0, 0], tar[0, 1], tar[0, 3], tar[0, 4], tar[0, 5], tar[0, 6], tar[0, 2]])
    # change robot's joint limits to the modified ones to compute the real joint angles
    robot.setJointLimits(ORIGINAL_LOWER_LIMITS, ORIGINAL_UPPER_LIMITS)
    _ = gripper_client.receive(ret=True)
    gripper_client.send('pub')
    gripper_client.send('pos')
    gripper_client.send('{}, {}, 0.0, 0.0, 0.0'.format(np.pi / 2, np.pi / 2))
    gripper_client.receive()
    time.sleep(2)
    _ = gripper_client.receive(ret=True)
    gripper_client.send('pub')
    gripper_client.send('pos')
    gripper_client.send('{}, {}, 1.85, 0.0, 0.0'.format(np.pi / 2, np.pi / 2))
    gripper_client.receive()
    time.sleep(2)
    gripper_client.send('Hello')
    _ = gripper_client.receive(ret=True)
    gripper_client.send('pub')
    gripper_client.send('pos')
    gripper_client.send('{}, {}, 1.85, 0.9, 0.9'.format(np.pi / 2, np.pi / 2))
    gripper_client.receive()
    time.sleep(2)
    gripper_client.send('Hello')
    _ = gripper_client.receive(ret=True)
    gripper_client.send('pub')
    gripper_client.send('pos')
    gripper_client.send('2.25, 2.25, 2.1, 0.9, 0.9')
    gripper_client.receive()
    time.sleep(2)
    target = Pose_2_KUKA(pose.PoseAbs())
    pose.setPoseAbs(KUKA_2_Pose((target[0], target[1], target[2] + 30, target[3], target[4], target[5])))
    # change robot's joint limits to the original ones to avoid joint limitations
    robot.setJointLimits(MODIFIED_LOWER_LIMITS, MODIFIED_UPPER_LIMITS)
    tar = tr(pose.Joints())
    """ 
        Change the position of the Joints because KUKA LWR IV+ has declared the third joint as an External Joint.
        KUKA LWR IV+ Joints = [A1, A2, E1, A3, A4, A5, A6] and RoboDK sends the joints as [A1, A2, A3, A4, A5, A6, E1]
    """
    robot.MoveJ([tar[0, 0], tar[0, 1], tar[0, 3], tar[0, 4], tar[0, 5], tar[0, 6], tar[0, 2]])
    # change robot's joint limits to the modified ones to compute the real joint angles
    robot.setJointLimits(ORIGINAL_LOWER_LIMITS, ORIGINAL_UPPER_LIMITS)
    return True


if __name__ == '__main__':
    """
    a = Client('192.168.56.2', 20000, encrypt_flag=True)
    a.send('Hello')
    a.receive()
    a.send('status')
    b = a.receive(ret=True)
    if b == 'INACTIVE':
        a.send('Hello')
        a.receive()
        a.send('start')
        a.receive()

    a.start_grasping()
    """
    """
    a = Client('192.168.56.2', 20000, encrypt_flag=True)
    a.send('Hello')
    a.receive()
    a.send('status')
    b = a.receive(ret=True)
    if b == 'INACTIVE':
        a.send('Hello')
        a.receive()
        a.send('start')
        a.receive()

    a.send('hello')
    a.receive()
    a.send('pub')
    a.send('open')
    a.receive()
    time.sleep(3)
    a.send('Hello')
    a.receive()
    a.send('pub')
    a.send('position')
    a.send(a.format_position_message(1.65, 1.55, 0.0, 0.0, 0.0))
    a.receive()
    time.sleep(3)
    a.send('Hello')
    a.receive()
    a.send('pub')
    a.send('pos')
    a.send(a.format_position_message(1.65, 1.55, 1.85, 0.0, 0.0))
    a.receive()
    time.sleep(3)
    a.send('Hello')
    a.receive()
    a.send('pub')
    a.send('pos')
    a.send(a.format_position_message(1.65, 1.55, 1.85, 0.8, 1.0))
    a.receive()
    time.sleep(3)
    time.sleep(3)
    a.send('Hello')
    a.receive()
    a.send('pub')
    a.send('pos')
    a.send(a.format_position_message(2.0, 2.0, 1.85, 0.8, 1.0))
    a.receive()
    time.sleep(3)
    a.send('Hello')
    a.receive()
    a.send('pub')
    a.send('open')
    a.receive()
    """
    """
    a.send('start')
    a.receive()
    time.sleep(20)
    a.send('hello')
    a.receive()
    a.send('pub')
    a.send('open')
    a.send('Hello')
    a.receive()
    a.send('pub')
    a.send('position')
    a.send(a.format_position_message(1.9, 1.9, 1.9, 0.5, 0.5))
    # a.send('grasp')
    """
    """
    a = Server('localhost', 20000)
    a.listen()
    """
    """
    def client():
        a = Server('localhost', 20000)
        a.listen()
    o = threading.Thread(target=client, args=())
    o.start()

    b = Client('localhost', 20000)
    b.send('Hello')
    b.receive()
    """
    """
    for i in range(15):
        a = Client('192.168.56.2', 20000, encrypt_flag=True)
        a.send('Hello')
        a.receive()
        a.send('clients')
        a.send('root')
        a.receive()
    """
    """
    a = Client('192.168.56.2', 20000, encrypt_flag=True)
    a.send('Hello')
    a.receive()
    a.send('status')
    b = a.receive(ret=True)
    if b == 'INACTIVE':
        a.send('Hello')
        a.receive()
        a.send('start')
        a.receive()

    a.start_grasping()
    """
    a = Server('192.168.56.2', 20000, encrypt_flag=True)
    a.listen()
