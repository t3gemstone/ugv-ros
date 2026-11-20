# Current state of the line sensor array (published by ugv_line/line_sensor_node).

std_msgs/Header header

# Sensor readings from left to right; true = the sensor sees the line.
bool[] sensors

# Raw readings, left to right. QTR-RC: discharge time [µs] (dark = long);
# digital sensor: pin level (0/1). For threshold tuning.
uint16[] raw

# Does at least one sensor see the line?
bool detected

# Lateral position of the line relative to the robot axis (metres, ROS convention: left is +y).
# Meaningless while detected=false.
float32 position

# All sensors see the line at the same time (intersection / finish line).
bool intersection
