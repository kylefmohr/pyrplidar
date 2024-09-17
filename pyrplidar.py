from pyrplidar_serial import PyRPlidarSerial
from pyrplidar_protocol import *
import pyrplidar_protocol
import struct



class PyRPlidar:

    def __init__(self):
        self.lidar_serial = None
        self.measurements = None
    
    def __del__(self):
        self.disconnect()

    

    def connect(self, port="/dev/ttyUSB0", baudrate=115200, timeout=3):
        self.lidar_serial = PyRPlidarSerial()
        self.lidar_serial.open(port, baudrate, timeout)
        print("PyRPlidar Info : device is connected")


    def disconnect(self):
        if self.lidar_serial is not None:
            self.lidar_serial.close()
            self.lidar_serial = None
            print("PyRPlidar Info : device is disconnected")



    def send_command(self, cmd, payload=None):
        if self.lidar_serial == None:
            raise PyRPlidarConnectionError("PyRPlidar Error : device is not connected")

        self.lidar_serial.send_data(PyRPlidarCommand(cmd, payload).raw_bytes)

    def receive_descriptor(self):
        if self.lidar_serial == None:
            raise PyRPlidarConnectionError("PyRPlidar Error : device is not connected")
        
        descriptor = PyRPlidarResponse(self.lidar_serial.receive_data(RPLIDAR_DESCRIPTOR_LEN))
        
        if descriptor.sync_byte1 != RPLIDAR_SYNC_BYTE1[0] or descriptor.sync_byte2 != RPLIDAR_SYNC_BYTE2[0]:
            raise PyRPlidarProtocolError("PyRPlidar Error : sync bytes are mismatched", hex(descriptor.sync_byte1), hex(descriptor.sync_byte2), hex(RPLIDAR_SYNC_BYTE1[0]), hex(RPLIDAR_SYNC_BYTE2[0]))
        return descriptor

    def receive_data(self, descriptor):
        if self.lidar_serial == None:
            raise PyRPlidarConnectionError("PyRPlidar Error : received data length is mismatched")
        
        data = self.lidar_serial.receive_data(descriptor.data_length)
        if len(data) != descriptor.data_length:
            raise PyRPlidarProtocolError("PyRPlidar Error : received data length is mismatched")
        return data



    def stop(self):
        self.send_command(RPLIDAR_CMD_STOP)

    def reset(self):
        self.send_command(RPLIDAR_CMD_RESET)

    def set_motor_pwm(self, pwm):
        self.lidar_serial.set_dtr(False)
        self.send_command(RPLIDAR_CMD_SET_MOTOR_PWM, struct.pack("<H", pwm))
    
    

    def get_info(self):
        self.send_command(RPLIDAR_CMD_GET_INFO)
        descriptor = self.receive_descriptor()
        data = self.receive_data(descriptor)
        return PyRPlidarDeviceInfo(data)

    def get_health(self):
        self.send_command(RPLIDAR_CMD_GET_HEALTH)
        descriptor = self.receive_descriptor()
        data = self.receive_data(descriptor)
        return PyRPlidarHealth(data)

    def get_samplerate(self):
        self.send_command(RPLIDAR_CMD_GET_SAMPLERATE)
        descriptor = self.receive_descriptor()
        data = self.receive_data(descriptor)
        return PyRPlidarSamplerate(data)

    def get_lidar_conf(self, payload):
        self.send_command(RPLIDAR_CMD_GET_LIDAR_CONF, payload)
        descriptor = self.receive_descriptor()
        data = self.receive_data(descriptor)
        return data

    def get_scan_mode_count(self):
        data = self.get_lidar_conf(struct.pack("<I", RPLIDAR_CONF_SCAN_MODE_COUNT))
        count = struct.unpack("<H", data[4:6])[0]
        return count

    def get_scan_mode_typical(self):
        data = self.get_lidar_conf(struct.pack("<I", RPLIDAR_CONF_SCAN_MODE_TYPICAL))
        typical_mode = struct.unpack("<H", data[4:6])[0]
        return typical_mode

    def get_scan_modes(self):
        
        scan_modes = []
        scan_mode_count = self.get_scan_mode_count()
        
        for mode in range(scan_mode_count):
            scan_mode = PyRPlidarScanMode(
                            self.get_lidar_conf(struct.pack("<IH", RPLIDAR_CONF_SCAN_MODE_NAME, mode)),
                            self.get_lidar_conf(struct.pack("<IH", RPLIDAR_CONF_SCAN_MODE_MAX_DISTANCE, mode)),
                            self.get_lidar_conf(struct.pack("<IH", RPLIDAR_CONF_SCAN_MODE_US_PER_SAMPLE, mode)),
                            self.get_lidar_conf(struct.pack("<IH", RPLIDAR_CONF_SCAN_MODE_ANSWER_TYPE, mode)))
            scan_modes.append(scan_mode)
        
        return scan_modes

    

    def start_scan(self):
        self.send_command(RPLIDAR_CMD_SCAN)
        descriptor = self.receive_descriptor()
    
        def scan_generator():
            while True:
                data = self.receive_data(descriptor)
                yield PyRPlidarMeasurement(data)
    
        return scan_generator

    
    
    def start_scan_express(self, mode):
        
        self.send_command(RPLIDAR_CMD_EXPRESS_SCAN, struct.pack("<BI", mode, 0x00000000))
        
        descriptor = self.receive_descriptor()

        if descriptor.data_type == 0x82:
            capsule_type = PyRPlidarScanCapsule
        elif descriptor.data_type == 0x84:
            capsule_type = PyRPlidarScanUltraCapsule
        elif descriptor.data_type == 0x85:
            capsule_type = PyRPlidarScanDenseCapsule
        else:
            raise PyRPlidarProtocolError("RPlidar Error : scan data type is not supported")
        
        def scan_generator():
            
            data = self.receive_data(descriptor)
            capsule_prev = capsule_type(data)
            capsule_current = None
            
            while True:
                data = self.receive_data(descriptor)
                capsule_current = capsule_type(data)
                
                nodes = capsule_type._parse_capsule(capsule_prev, capsule_current)
                for index, node in enumerate(nodes):
                     yield PyRPlidarMeasurement(raw_bytes=None, measurement_hq=node)
    
                capsule_prev = capsule_current

        return scan_generator

    
    def force_scan(self):
        self.send_command(RPLIDAR_CMD_FORCE_SCAN)
        descriptor = self.receive_descriptor()
        
        def scan_generator():
            while True:
                data = self.receive_data(descriptor)
                yield PyRPlidarMeasurement(data)
        
        return scan_generator
    