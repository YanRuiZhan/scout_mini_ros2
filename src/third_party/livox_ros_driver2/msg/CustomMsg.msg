std_msgs/Header header
uint64 timebase          # timestamp in nanosecond
uint32 point_num         # Total point number
uint8 lidar_id           # Lidar device id number
uint8[3] rsvd            # Reserved usage (Not defined)
CustomPoint[] points     # Point data array
