import pyzed.sl as sl
import socket
import threading

class SocketServer(threading.Thread):
    def __init__(self, host, host_port, server, server_port):
        threading.Thread.__init__(self)
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.bind((host, host_port))
        self.server = (server, server_port) 
        self.data = '' 
        self._stop = threading.Event()
        self.lock = threading.Lock()

    def run(self):
        while True:
            self.data = self.s.recvfrom(1024)


    def receivedMessage(self):
        return self.data.decode('utf-8')

        
    def sendMessage(self, message):
        self.s.sendto(message.encode('utf-8'), self.server)
        



def main():
    
    zed = sl.Camera()
    socketServer = SocketServer("10.56.87.106", 27002, "10.56.87.2", 27001)
    socketServer.start()


    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720

    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    init_params.coordinate_units = sl.UNIT.METER

    err = zed.open(init_params)
    
    if err != sl.ERROR_CODE.SUCCESS:
        exit(1)

    py_transform = sl.Transform()
    tracking_parameters = sl.PositionalTrackingParameters(_init_pos=py_transform)
    err = zed.enable_positional_tracking(tracking_parameters)
    if err != sl.ERROR_CODE.SUCCESS:
        exit(1)

    i = 0
    zed_pose = sl.Pose()

    zed_sensors = sl.SensorsData()
    runtime_parameters = sl.RuntimeParameters()

    while i < 10000:
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:

            zed.get_position(zed_pose, sl.REFERENCE_FRAME.WORLD)
            zed.get_sensors_data(zed_sensors, sl.TIME_REFERENCE.IMAGE)
            zed_imu = zed_sensors.get_imu_data()

            py_translation = sl.Translation()
            x = round(zed_pose.get_translation(py_translation).get()[0], 3)
            y = round(zed_pose.get_translation(py_translation).get()[1], 3)
            z = round(zed_pose.get_translation(py_translation).get()[2], 3)
            timestamp = zed_pose.timestamp.get_milliseconds()
            data = (str(x)+";"+str(y)+";"+str(timestamp))
            print(data)
            socketServer.sendMessage(data)

            i = i + 1

    zed.close()


if __name__ == "__main__":
    main()

