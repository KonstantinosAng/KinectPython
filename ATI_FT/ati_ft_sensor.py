"""
Author: Konstantinos Angelopoulos
Date: 04/02/2020
All rights reserved.
Feel free to use and modify and if you like it give it a star.
"""
"""
Handles the communication between the controller, the FT Sensor and the laptops
"""
import socket
import os
import time
import threading
import sys


# Create Server to listen for Clients
class AtiFtServer(object):

    def __init__(self, ip, port, serial_port='COM1', mode='ascii', encrypt_flag=False):
        """
        :param ip: IP that the ATI FT server listens to
        :param port: Port that the ATI FT server listens to
        :param serial_port: Serial port that the ATI FT sensor is connected to (see FTsensor3.py or FTsensor.py)
        :param mode: communication mode between the ATI and the Controller (see FTsensor3.py or FTsensor.py)
        :param encrypt_flag: Flag to encrypt the communication with SHA-256
        :return None
        """
        self.IP = ip  # IP to communicate using UDP
        self.Port = port  # Port to bind Server
        self._serial_port = serial_port  # Port that the ATI FT is connected
        self.bufferSize = 5120  # buffer to pad message to
        self.msg = str.encode(" Hello UDP Client ")  # Hello message to greet clients
        self.msg_exit = str.encode(" Closing server...\n Exit with code 0. ")  # Exit message at closing
        # Create a datagram socket at given IP and Port
        self._socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        # Bind to address and ip
        self._socket.bind((self.IP, self.Port))
        self.clients = []  # Store clients IP and Port
        self.client_list_key = 'root'  # Key to access client list, same key used at encryption.py
        self._forces = []  # Store forces and Torques
        self._done = False  # Flag to stop and exit server
        self._match = False  # Flag for not storing same clients
        self._status = False  # Flag for server status
        self._encryption = encrypt_flag  # Flag to encrypt messages
        self._frequency = 0.0  # Sampling Frequency
        self._stop_reading = False  # Flag to stop reading from sensor
        self._sensor_bias = False  # Flag to bias the sensor values
        self._sensor_unbias = False  # Flag to unbias the sensor values
        self._mode = mode  # FTsensor communication mode, either ascii or binary. Ascii works better, binary give fault values
        self._status = False  # Server Status
        self._limit = ''  # Check what value exceeded
        self._exceed = False  # Check if any values exceeded

    def start_thread(self, cmd):
        """
        Executes cmd/terminal commands in new threads in a new terminal on a specific location with specific width and height
        :return None
        """
        thread_command = "gnome-terminal -e 'bash -c \"{} bash\" '".format(cmd)
        os.system(thread_command)

    def start_reading(self):
        """
        Main loop that runs in a separate thread and constantly
        pulls and saves data from the  ATI FT sensor.
        :return: None
        """
        # check for python version
        if sys.version_info[0] < 3:
            from FTsensor import Sensor
        else:
            from FTsensor3 import Sensor
        # connect to the ATI controller
        daq = Sensor(self._serial_port, mode=self._mode)
        self._status = True
        # start main loop
        while not self._stop_reading and not self._exceed:
            try:
                t = time.time()  # frequency time
                _msg = daq.read()  # read counts values
                forces = daq.counts_2_force_torque(_msg)  # convert counts to ft
                # Bias the sensor values
                if self._sensor_bias:
                    forces = daq.counts_2_force_torque(_msg, unbiased=True)
                    daq.sensor_bias(forces)
                    forces = daq.counts_2_force_torque(_msg)
                    self._sensor_bias = False
                # Unbias the sensor values
                if self._sensor_unbias:
                    daq.sensor_unbias()
                    self._sensor_unbias = False
                self._forces = forces  # store forces
                # calculate frequency
                if (time.time() - t) > 0:
                    self._frequency = 1.0 / (time.time() - t)
                # check if a force/torque exceeds a limit
                for f, limit, flag in zip([forces[0], forces[1], forces[2], forces[3], forces[4], forces[5]], [65.0, 65.0, 65.0, 5.0, 5.0, 5.0], ['FX Axis', 'FY Axis', 'FZ Axis', 'TX Axis', 'TY Axis', 'TZ Axis']):
                    if abs(f) > limit:
                        self._limit = flag
                        self._exceed = True
            except Exception as e:
                print('[ATI FT SERVER]: {}'.format(e))
        # quit and stop loop
        daq.stop()
        self._status = False

    def send(self, msg, address):
        """
        Sends an encrypted/unencrypted string message
        :param msg a string of message
        :param address IP address and port of the connected client
        :return None
        """
        import encryption
        if self._encryption:
            msg = encryption.encrypt(msg)
        self._socket.sendto(msg, address)

    def receive(self):
        """
        Receives a string message from a client and decrypts it.
        :return string of message received from a client
        """
        import encryption
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
        print("[ATI FT SERVER]: UDP server up and listening ")
        while not self._done:
            # receive message
            bytes_received = self.receive()
            message = bytes_received[0]
            address = bytes_received[1]
            client_msg = "[ATI FT SERVER]: Message from Client {}: {} ".format(address, message.decode('utf-8'))
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
            # print(client_msg)
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
                    print("[ATI FT SERVER]: {}".format(self.msg_exit.decode('utf-8')))
                    self._stop_reading = True  # Stop reading
                    self.close()  # Close server and quit
                except Exception as e:
                    self.send(str.encode(str(e)), address)
            elif message.decode('utf-8') == 'status' or message.decode('utf-8') == 'active':
                try:
                    # Send the status of the server if the start command was send
                    if self._status:
                        self.send(str.encode("ACTIVE"), address)
                    else:
                        self.send(str.encode('INACTIVE'), address)
                except Exception as e:
                    self.send(str.encode(str(e)), address)
            elif message.decode('utf-8') == 'frequency':
                try:
                    """ Return the frequency sampling rate """
                    if self._frequency > 0:
                        self.send(str.encode("{} Hz".format(self._frequency)), address)
                    else:
                        self.send(str.encode("Server status is INACTIVE. Send help for more information"), address)
                except Exception as e:
                    self.send(str.encode(str(e)), address)
            elif message.decode('utf-8') == 'sensor_bias' or message.decode('utf-8') == 'bias' or message.decode('utf-8') == 'sb' or message.decode('utf-8') == 'zero':
                try:
                    """ Bias the sensor values """
                    self._sensor_bias = True
                    print('[ATI FT SERVER]: Biased the Sensor Values')
                    self.send(str.encode('Sensor was Biased'), address)
                except Exception as e:
                    self.send(str.encode(str(e)), address)
            elif message.decode('utf-8') == 'sensor_unbias' or message.decode('utf-8') == 'unbias' or message.decode('utf-8') == 'sub' or message.decode('utf-8') == 'unzero':
                try:
                    """ unbias the sensor values """
                    self._sensor_unbias = True
                    print('[ATI FT SERVER]: Unbiased the Sensor Values')
                    self.send(str.encode('Sensor was unbiased'), address)
                except Exception as e:
                    self.send(str.encode(str(e)), address)
            elif message.decode('utf-8') == 'stop' or message.decode('utf-8') == 'finish':
                try:
                    """ Stops pulling data from the ati ft server """
                    self._stop_reading = True
                    self.send(str.encode(" Stopped reading from ATI FT... "), address)
                    print('[ATI FT SERVER]: Stopped reading from ATI FT... ')
                except Exception as e:
                    self.send(str.encode(str(e)), address)
            elif message.decode('utf-8') == 'start' or message.decode('utf-8') == 'begin':
                try:
                    if not self._status:
                        """ starts pulling and reading data from the ati ft server """
                        start_ati_sensor = threading.Thread(target=self.start_reading, args=())
                        start_ati_sensor.start()
                        self.send(str.encode(" Started ATI FT... You can grab... "), address)
                        print('[ATI FT SERVER]: Started ATI FT... You can grab... ')
                    else:
                        self.send(str.encode(" ATI is already started and connected. You can grab values. "), address)
                except Exception as e:
                    self.send(str.encode(str(e)), address)
            elif message.decode('utf-8') == 'read' or message.decode('utf-8') == 'forces' or message.decode('utf-8') == 'torques' or message.decode('utf-8') == 'return' or message.decode('utf-8') == 'values' or message.decode('utf-8') == 'ft':
                try:
                    if self._exceed:
                        """ Return the axis that exceeded the limit """
                        self.send(str.encode("Force Exceeded in {}. Sensor stopped. Restart the sensor".format(self._limit)), address)
                        print("[ATI FT SERVER]: Force Exceeded in {}. Sensor stopped.".format(self._limit))
                        self._exceed = False
                        self._forces = []
                        self._limit = ''
                        self._status = False
                    if len(self._forces) > 0:
                        """ Return the last force torque values as Fx, Fy, Fz, Tx, Ty, Tz (Forces are in N and Torques are in Nm) """
                        self.send(str.encode("{}, {}, {}, {}, {}, {}".format('%.3f' % self._forces[0], '%.3f' % self._forces[1], '%.3f' % self._forces[2], '%.3f' % self._forces[3], '%.3f' % self._forces[4], '%.3f' % self._forces[5])), address)
                    else:
                        self.send(str.encode('No value is grabbed, make sure the server status is ACTIVE. Send help for more information.'), address)
                except Exception as e:
                    self.send(str.encode(str(e)), address)
            elif message.decode('utf-8') == 'cal' or message.decode('utf-8') == 'calibration' or message.decode('utf-8') == 'matrix':
                try:
                    """ return the calibration data from the ati ft sensor """
                    import json
                    with open('Calibration/FT3585-Calibration_matrix.json') as calibration_file:
                        data = json.load(calibration_file)
                        calibration = data['Calibration Matrix']
                    self.send(str.encode(" Calibration Matrix = {} ".format(calibration)), address)
                    del json
                except Exception as e:
                    self.send(str.encode(str(e)), address)
            elif message.decode('utf-8') == 'help' or message.decode('utf-8') == 'info' or message.decode('utf-8') == 'commands' or message.decode('utf-8') == 'command':
                try:
                    """ Write help message like this to appear better """
                    help_message = """
 =======================================================================================
 ------------------------------- Available commands ------------------------------------
 =======================================================================================
 Always start with Hello and after you receive the answer from server type the command.
 All commands are transformed to lower letters so they can be written in any form as long
 as the grammar is correct.
 Commands:
     help:  -Returns the available options for the server.
     start/begin: -Starts reading the Fx, Fy, Fz, Tx, Ty, Tz values in N and Nm respectively.
     stop/finish: -Stops reading values from force torque sensor.
     status/active: -Returns ACTIVE if the server has started reading or STOPPED INACTIVE if it hasn't.
     exit/quit/close/terminate/kill: -Closes all terminal and terminates the UDP server listening.
     clients_list/clients/client_list/list: -Returns the list of clients (ip and port) that have communicated with the server and their. After this command the server waits
                                             for the key to be send in order to send you the client list.
     last_client/last: -Returns the last client (IP and Port) that interacted with the Server. After this command the server waits for the key to be send in order to send you the client list.
     read/forces/torques/return/values/ft: -Returns the last force values grabbed from the sensor given that the status is ACTIVE. The Force Torque values are in the format of: 
                                            'fx: 0.0, fy: 0.0, fz: 0.0, tx: 0.0, ty: 0.0, tz: 0.0' in Newton and Newton/meter respectively.
     calibration/matrix/cal: -Returns the calibration matrix stored in the Force Torque Sensor.
     frequency: -Returns the sampling frequency in Hz.
     bias/sensor_bias/sb/zero: -Bias the sensor values.
     unbias/sensor_unbias/sub/unzero: -Unbias the sensor values.
     Example to communicate with the server using the Client class:
        Server code:
        s = AtiFtServer('192.168.56.2', 20000, serial_port='COM1', mode='ascii', encrypt_flag=True)
        s.listen()
        Client code:
        c = AtiFtClient('192.168.56.2', 20000, encrypt_flag=True)
        c.send('Hello')  # or a.send('hello') server handles lower and upper case letters
        c.receive()  # always receive
        c.send('status')
        msg = c.receive(ret=True)
        if msg == 'INACTIVE':
            c.send('hello')
            c.receive()
            c.send('start') 
            c.receive()
        while True:
            c.send('Hello')
            msg = c.receive(ret=True)
            c.send('forces')
            c.receive()
         """
                    self.send(str.encode(help_message), address)
                except Exception as e:
                    self.send(str.encode(str(e)), address)
            elif message.decode('utf-8') == 'clients_list' or message.decode('utf-8') == 'clients' or message.decode('utf-8') == 'client_list' or message.decode('utf-8') == 'list':
                try:
                    """ return the client list """
                    # Wait for the correct key
                    bytes_received = self.receive()
                    message = bytes_received[0]
                    message = message.decode('utf-8')
                    message = message.lower()
                    if message != self.client_list_key:
                        self.send(str.encode('Wrong Key'), address)
                        continue
                    print('[ATI FT SERVER]: Sending Client List..... ')
                    cl_msg = ' \n------------- Client List -------------\n '
                    for id_number, client in enumerate(self.clients):
                        cl_msg += ' Client: ID = {}, IP = {}, Port = {}\n '.format(id_number, client[0], client[1])
                    self.send(str.encode(cl_msg), address)
                except Exception as e:
                    self.send(str.encode(str(e)), address)
            elif message.decode('utf-8') == 'last_client' or message.decode('utf-8') == 'last':
                try:
                    """ return the last connected client """
                    # Wait for the correct key
                    bytes_received = self.receive()
                    message = bytes_received[0]
                    address = bytes_received[1]
                    message = message.decode('utf-8')
                    message = message.lower()
                    if message != self.client_list_key:
                        self.send(str.encode('Wrong Key'), address)
                        continue
                    print('[ATI FT SERVER]: Sending Last Client to: {} {} '.format(address[0], address[1]))
                    last_client = '\n Last Client: ID = {}, IP = {}, Port = {}\n '.format(len(self.clients) - 1, self.clients[-1][0], self.clients[-1][1])
                    self.send(str.encode(last_client), address)
                except Exception as e:
                    self.send(str.encode(str(e)), address)
            else:
                try:
                    """ return a help message if not a valid command is send """
                    self.send(str.encode('Send the Hello message and then send help or info or command or commands to learn how to communicate with the server'), address)
                except Exception as e:
                    self.send(str.encode(str(e)), address)

    # Close server and exit
    def close(self):
        """
        Close the socket and quit
        :return None
        """
        self._done = True
        self._socket.close()


# Create Client to communicate with Server
class AtiFtClient(object):

    def __init__(self, ip, port, encrypt_flag=False):
        """
        :param ip: IP that the server is listening on
        :param port: port that the server is listening on
        :param encrypt_flag: Flag to encrypt messages or not (must be the same value as the Server value)
        :return None
        """
        self.bufferSize = 5120
        # Create a UDP socket at client side (AF_INET6 for IPv6 IP (WAN IP) and AF_INET for IPv4 (LAN/WLAN IP))
        self._socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self._done = False
        self.server_ip = ip
        self.server_port = port
        self._encryption = encrypt_flag

    def send(self, msg):
        """
        Send to server using created UDP socket
        :param msg: a string of characters
        :return None
        """
        from ATI_FT import encryption
        # Send to server using created UDP socket
        if self._encryption:
            msg = encryption.encrypt(str.encode(msg))
            self._socket.sendto(msg, (self.server_ip, self.server_port))
        else:
            self._socket.sendto(str.encode(msg), (self.server_ip, self.server_port))

    def receive(self, ret=False):
        """
        Receive messages
        :param ret: flag to print the received message or return the message
        :return a string of characters
        """
        from ATI_FT import encryption
        msg_received = self._socket.recvfrom(self.bufferSize)
        if self._encryption:
            if not ret:
                msg_received = encryption.decrypt(msg_received[0])
                print("[ATI FT CLIENT]: Message from Server: {} ".format(msg_received.decode('utf-8')))
            else:
                msg_received = encryption.decrypt(msg_received[0])
                return msg_received.decode('utf-8')
        else:
            if not ret:
                print("[ATI FT CLIENT]: Message from Server: {} ".format(msg_received[0].decode('utf-8')))
            else:
                return msg_received[0].decode('utf-8')

    # Close client and exit
    def close(self):
        """
        Close the socket and quit
        :return None
        """
        self._done = True
        self._socket.close()


if __name__ == '__main__':
    # Test functionality
    a = AtiFtServer('localhost', 10000, serial_port='COM1', mode='ascii', encrypt_flag=True)
    a.listen()
