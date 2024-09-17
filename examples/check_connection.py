from pyrplidar import PyRPlidar
import time

SERIAL_PORT = 'com16'
BAUDRATE = 1000000
# Update the serial port and baudrate to match your device
# Linux   : "/dev/ttyUSB0"
# MacOS   : "/dev/tty.usbserial-0001"
# Windows : "COM5"

def check_connection():

    lidar = PyRPlidar()
    lidar.connect(port=SERIAL_PORT, baudrate=BAUDRATE, timeout=10)
    # Linux   : "/dev/ttyUSB0"
    # MacOS   : "/dev/cu.SLAB_USBtoUART"
    # Windows : "COM5"
    # lidar.stop()
    # time.sleep(2)
    # lidar.reset()   
    # time.sleep(2)           
    info = lidar.get_info()
    print("info :", info)
    
    health = lidar.get_health()
    print("health :", health)
    
    samplerate = lidar.get_samplerate()
    print("samplerate :", samplerate)
    
    
    scan_modes = lidar.get_scan_modes()
    print("scan modes :")
    for scan_mode in scan_modes:
        print(scan_mode)

    scan_generator = lidar.start_scan()
    with open("scan.pcd", "w") as f:
        f.write("# .PCD v.7 - Point Cloud Data file format\n")
        f.write("VERSION .7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("WIDTH 20000\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write("POINTS 20000\n")
        f.write("DATA ascii\n")
        height = 0
        while True:
            
            for count, scan in enumerate(scan_generator()):
                print(count, scan)
                if count == 200: break
                # f.write("\n".join(["{} {} {}".format(x, y, height) for x, y, quality in scan]))
                # f.write("\n")
            input("Move the lidar up and press enter")
            height += 1
            
    
        
    for count, scan in enumerate(scan_generator()):
        print(count, scan)
        if count == 20000: break

    lidar.stop()
    lidar.disconnect()



if __name__ == "__main__":
    check_connection()
