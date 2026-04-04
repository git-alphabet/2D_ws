# Livox publish pointcloud msg format.

std_msgs/Header header    # ROS standard message header
uint64 timebase           # The time of first point (ns)
uint32 point_num          # Total number of points
uint8  lidar_id           # Lidar device id number
uint8[3]  rsvd            # Reserved use
CustomPoint[] points      # Pointcloud data
