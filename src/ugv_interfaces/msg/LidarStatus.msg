# Lidar connection status (/ugv/lidar, lidar_manager_node).

string source     # usb | gpio
string port       # serial port in use; empty if none was found
bool running      # the sllidar driver is running
float32 rate_hz   # /scan rate over the last few seconds
string message    # last error or info
