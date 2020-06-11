"""
This file is used only if the Gripper is run on ROS Indigo and works by executing bash files
in order to publish to the ROS nodes. It is very slow and not used. It is left only if it is
necessary to have ROS Indigo, otherwise use the ros_handler.py that runs with ROS Jade in near real time.
"""
import socket
import os
import time
import threading
import Gripper.encryption as encryption


# Create Server to listen for Clients
class Server(object):
    """
    Class to handle the Server backend communication with the Windows API.
    """

    def __init__(self, ip, port, encrypt_flag=False):
        """
        Initialization parameters for the Server
        :param ip: IP that the Server is listening on
        :param port: Port that the server is listening on
        :param encrypt_flag: Flag to encrypt messages or not (must be the same value as the Client value)
        :return None
        """
        self.IP = ip  # server ip
        self.Port = port  # server port
        self.bufferSize = 10240  # message buffer 1 KB
        self.msg = str.encode(" Hello UDP Client ")  # server hello message
        self.msg_exit = str.encode(" Closing server...\n Exit with code 0. ")  # server exit message
        # Create a datagram socket
        self._socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        # Bind to address and ip
        self._socket.bind((self.IP, self.Port))
        self.clients = []  # client list
        self.client_list_key = 'root'  # client list key (default root)
        self._done = False  # flag to exit
        self._match = False  # flag to check if client is already on list
        self._ros_started = False  # flag if ROS started
        self._encryption = encrypt_flag  # encryption flag

    def start_thread(self, cmd):
        """
        Executes cmd/terminal commands in new threads in a new terminal on a specific location
        :return None
        """
        thread_command = "gnome-terminal --geometry 60x10+50+50 -e 'bash -c \"{} bash\" '".format(cmd)
        # thread_command = "gnome-terminal -e 'bash -c \"" + cmd + " bash\" '"
        os.system(thread_command)

    def send(self, msg, address):
        """
        Sends an encrypted/unencrypted string message
        :param msg a string of message
        :param address IP address and port of the connected client
        :return None
        """
        if self._encryption:
            msg = encryption.encrypt(msg)
        self._socket.sendto(msg, address)

    def receive(self):
        """
        Receives a string message from a client and decrypts it.
        :return string of message received from a client
        """
        bytes_received = self._socket.recvfrom(self.bufferSize)
        if self._encryption:
            return [encryption.decrypt(bytes_received[0]), bytes_received[1]]
        return bytes_received

    # Listen for clients
    def listen(self):
        """
        Main loop that listens for connections, executes commands and handles the backend processes.
        :return None
        """
        print("[GRIPPER SERVER]: UDP server up and listening ")
        while not self._done:
            bytes_received = self.receive()
            # receive message
            message = bytes_received[0]
            address = bytes_received[1]
            client_msg = "[GRIPPER SERVER]: Message from Client {}: {} ".format(address, message.decode('utf-8'))
            temp_message = message.decode('utf-8')
            temp_message = temp_message.lower()
            # check if it is hello
            if temp_message != 'hello':
                self.send(str.encode('Please start with Hello as the first command...'), address)
                continue
            # clean client list after 16 saved clients
            if len(self.clients) >= 16:
                self.clients = []
            self._match = False
            # store clients
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
                    # exit server and quit
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
     help:  -Returns the available options for the server.
     start/begin: -Starts the ROS reflex_one.launch file to initialize the gripper communication.
     visualize/visualizer/visual/simulation/sim/simulate: -Starts the ROS rviz simulation.
     status_ros/ros_status/ros/active/status: -Returns ACTIVE if the roslaunch has started or STOPPED INACTIVE if it hasn't.
     exit/quit/close/terminate/kill: -Closes all terminal and terminates the UDP server listening.
     publish/post/pub: -After sending the command post or publish the server waits for message that must be posted to the rostopic that listens for the gripper positions.
                        Available commands after post/publish are:
                            open: Opens the Gripper to the zero position as specified in the calibration process see: https://www.labs.righthandrobotics.com/calibrate-fingers.
                            close: Closes the Gripper like a fist.
                            position/pos: Publishes a specific position for the gripper to move to. After sending the position command the server waits for the position values.
                                          The values must be in the following specific format including the brackets:
                                              {f1: 0.0, f2: 0.0, f3: 0.0, preshape1: 0.0, preshape2: 0.0}
                                              the f1/f2/f3 corresponds to the finger positions and the preshape1/2 to the finger rotations. All values are in radians!!!
                            clients_list/clients/client_list/list: -Returns the list of clients (ip and port) that have communicated with the server and their. After this command the server waits
                                             for the key to be send in order to send you the client list.
                            last_client/last: -Returns the last client (IP and Port) that interacted with the Server. 
                                               After this command the server waits for the key to be send in order to send you the client list.
                            calibrate/calib/cal: -Starts calibration for the reflex one grippers.
                            velocity/vel: -Changes the velocity of the grippers hands.
                                           The values must be in the following specific format including the brackets:
                                              {f1: 0.0, f2: 0.0, f3: 0.0, preshape1: 0.0, preshape2: 0.0}
                                              the f1/f2/f3 corresponds to the finger positions and the preshape1/2 to the finger rotations. All values are in radians!!!
                                       
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
                    # start rviz visualization
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
                        # start calibration
                        publish_ros = threading.Thread(target=self.start_thread, args=('./calibrate.sh',))
                        publish_ros.start()
                        publish_ros.join()  # Wait for it to stop
                        self.send(str.encode(' Started Calibration... Switch to ROS machine to calibrate. '), address)
                    except Exception as e:
                        self.send(str.encode(str(e)), address)
                elif message.decode('utf-8') == 'grasp' or message.decode('utf-8') == 'catch':
                    try:
                        # grasp an object
                        publish_ros = threading.Thread(target=self.start_thread, args=('./grasp.sh',))
                        publish_ros.start()
                        publish_ros.join()  # Wait for it to stop
                        self.send(str.encode(' Grasped Cloth '), address)
                    except Exception as e:
                        self.send(str.encode(str(e)), address)
                elif message.decode('utf-8') == 'open':
                    try:
                        # open gripper
                        publish_ros = threading.Thread(target=self.start_thread, args=('./open.sh',))
                        publish_ros.start()
                        publish_ros.join()  # Wait for it to stop
                        self.send(str.encode(' Opened Gripper '), address)
                    except Exception as e:
                        self.send(str.encode(str(e)), address)
                elif message.decode('utf-8') == 'close':
                    try:
                        # close gripper
                        publish_ros = threading.Thread(target=self.start_thread, args=('./close.sh',))
                        publish_ros.start()
                        publish_ros.join()  # Wait for it to stop
                        self.send(str.encode(' Closed Gripper '), address)
                    except Exception as e:
                        self.send(str.encode(str(e)), address)
                elif message.decode('utf-8') == 'velocity' or message.decode('utf-8') == 'vel':
                    try:
                        # change the  gripper velocity
                        bytes_received = self.receive()
                        message = bytes_received[0]
                        message = message.decode('utf-8')
                        # Remove the file if it exists
                        if os.path.exists('velocity.sh'):
                            os.remove('velocity.sh')
                        # Write new file with new position
                        with open('velocity.sh', 'w') as pub_file:
                            # Write bash file like this to appear better
                            pub_file.write("""
#! /bin/bash
# Always run setup.bash to initialize environment parameters that ROS requires
# Otherwise roslaunch will not find the files
. /home/kostas/catkin_ws/devel/setup.bash
# Assuming that the start.sh file has been called and ROS started correctly we can publish
# to the hand node
rostopic pub -1 /reflex_one/command_velocity reflex_one_msgs/VelocityCommand {}
                                           """.format(message))
                        # make bash file executable by sudo
                        executable_bash = threading.Thread(target=self.start_thread, args=('chmod +x velocity.sh',))
                        executable_bash.start()
                        executable_bash.join()  # Wait for it to stop
                        # Execute bash file
                        publish_ros = threading.Thread(target=self.start_thread, args=('./velocity.sh',))
                        publish_ros.start()
                        publish_ros.join()  # Wait for it to stop
                        self.send(str.encode(' Moved Gripper '), address)
                    except Exception as e:
                        self.send(str.encode(str(e)), address)
                elif message.decode('utf-8') == 'position' or message.decode('utf-8') == 'pos':
                    try:
                        # Move gripper to a specific position
                        bytes_received = self.receive()
                        message = bytes_received[0]
                        message = message.decode('utf-8')
                        # Remove the file if it exists
                        if os.path.exists('position.sh'):
                            os.remove('position.sh')
                        # Write new file with new position
                        with open('position.sh', 'w') as pub_file:
                            # Write bash file like this to appear better
                            pub_file.write("""
#! /bin/bash
# Always run setup.bash to initialize environment parameters that ROS requires
# Otherwise roslaunch will not find the files
. /home/kostas/catkin_ws/devel/setup.bash
# Assuming that the start.sh file has been called and ROS started correctly we can publish
# to the hand node
rostopic pub -1 /reflex_one/command_position reflex_one_msgs/PoseCommand {}
                                           """.format(message))
                        # make bash file executable by sudo
                        executable_bash = threading.Thread(target=self.start_thread, args=('chmod +x position.sh',))
                        executable_bash.start()
                        executable_bash.join()  # Wait for it to stop
                        # Execute bash file
                        publish_ros = threading.Thread(target=self.start_thread, args=('./position.sh',))
                        publish_ros.start()
                        publish_ros.join()  # Wait for it to stop
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
                    # check if key is correct
                    if message != self.client_list_key:
                        self.send(str.encode('Wrong Key'), address)
                        continue
                    # send client list
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
                    # check if key is correct
                    if message != self.client_list_key:
                        self.send(str.encode('Wrong Key'), address)
                        continue
                    # Send last client connected
                    print('[GRIPPER SERVER]: Sending Last Client to: {} {} '.format(address[0], address[1]))
                    last_client = ' Last Client: ID = {}, IP = {}, Port = {}\n '.format(len(self.clients)-1, self.clients[-1][0], self.clients[-1][1])
                    self.send(str.encode(last_client), address)
                except Exception as e:
                    self.send(str.encode(str(e)), address)
            else:
                try:
                    self.send(str.encode('Send the Hello message and then send help or info or command or commands to learn how to communicate with the server'), address)
                except Exception as e:
                    self.send(str.encode(str(e)), address)

    # Close server and exit
    def close(self):
        """
        Close the socket and quits
        :return None
        """
        self._done = True
        self._socket.close()


# Create Client to communicate with Server
class Client(object):
    """
    Class Client to communicate with the Server API
    """

    def __init__(self, ip, port, encrypt_flag=False):
        """
        :param ip: IP that the server is listening on
        :param port: port that the server is listening on
        :param encrypt_flag: Flag to encrypt messages or not (must be the same value as the Server value)
        :return None
        """
        self.bufferSize = 10240
        # Create a UDP socket at client side
        self._socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self._done = False
        self.server_ip = ip
        self.server_port = port
        self._encryption = encrypt_flag

    def send(self, msg):
        """
        Sends an encrypted/unencrypted string message
        :param msg a string of message
        :return None
        """
        if self._encryption:
            msg = encryption.encrypt(str.encode(msg))
            self._socket.sendto(msg, (self.server_ip, self.server_port))
        else:
            self._socket.sendto(str.encode(msg), (self.server_ip, self.server_port))

    def receive(self, ret=False):
        """
        Receives a string message from a client and decrypts it.
        :param ret: Flag to return the message or print it on the screen
        :return string of message received from a client
        """
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

    def start_grasping(self):
        """
        !!!!! Depreciated and not used
        Initializes the grasping technique
        :return None
        """
        self.send('hello')
        self.receive()
        self.send('pub')
        self.send('open')
        self.receive()
        time.sleep(3)
        self.send('Hello')
        self.receive()
        self.send('pub')
        self.send('position')
        self.send(self.format_position_message(1.65, 1.55, 0.0, 0.0, 0.0))
        self.receive()
        time.sleep(3)
        self.send('Hello')
        self.receive()
        self.send('pub')
        self.send('pos')
        self.send(self.format_position_message(1.65, 1.55, 1.85, 0.0, 0.0))
        self.receive()
        time.sleep(3)
        self.send('Hello')
        self.receive()
        self.send('pub')
        self.send('pos')
        self.send(self.format_position_message(1.65, 1.55, 1.85, 0.8, 1.0))
        self.receive()
        time.sleep(3)
        self.send('Hello')
        self.receive()
        self.send('pub')
        self.send('pos')
        self.send(self.format_position_message(2.0, 2.0, 1.85, 0.8, 1.0))
        self.receive()
        time.sleep(3)
        """
        self.send('Hello')
        self.receive()
        self.send('pub')
        self.send('open')
        self.receive()
        time.sleep(3)
        """

    def format_position_message(self, f1, f2, f3, p1, p2, preshape_only=False):
        """
        Formats the position of the Gripper in a PoseCommand format
        :param f1: position of the first finger in rad
        :param f2: position of the second finger in rad
        :param f3: position of the third finger in rad
        :param p1: rotation of the first finger in rad
        :param p2: rotation of the second finger in rad
        :param preshape_only: to indicate that only position is changed and not rotation
        """
        if preshape_only:
            return '{}preshape1: {}, preshape2: {}{}'.format('"{', float(p1), float(p2), '}"')
        """ Position Message must be in this format {f1: 0.0, f2: 0.0, f3: 0.0, preshape1: 0.0, preshape2: 0.0} """
        return '{}f1: {}, f2: {}, f3: {}, preshape1: {}, preshape2: {}{}'.format('"{', float(f1), float(f2), float(f3), float(p1), float(p2), '}"')

    # Close client and exit
    def close(self):
        """
        Close the socket and quits
        :return None
        """
        self._done = True
        self._socket.close()


if __name__ == '__main__':
    # Test API functionality
    """
    a = Client('localhost', 20000, encrypt_flag=False)
    a.send('Hello')
    a.receive()
    a.send('help')
    a.receive()
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
    a = Server('192.168.56.2', 20000, encrypt_flag=True)
    a.listen()
    """
