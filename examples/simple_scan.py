from pyrplidar import PyRPlidar
import time

SERIAL_PORT = 'com16'
BAUDRATE = 1000000
# Update the serial port and baudrate to match your device
# Linux   : "/dev/ttyUSB0"
# MacOS   : "/dev/tty.usbserial-0001"
# Windows : "COM5"

def simple_scan():

    lidar = PyRPlidar()
    lidar.connect(port=SERIAL_PORT, baudrate=BAUDRATE, timeout=3)
    # Linux   : "/dev/ttyUSB0"
    # MacOS   : "/dev/cu.SLAB_USBtoUART"
    # Windows : "COM5"

                  
    lidar.set_motor_pwm(500)
    time.sleep(2)
    
    scan_generator = lidar.force_scan()
    
    for count, scan in enumerate(scan_generator()):
        print(count, scan)
        if count == 20: break

    lidar.stop()
    lidar.set_motor_pwm(0)

    
    lidar.disconnect()


if __name__ == "__main__":
    simple_scan()
