This package can be used to fuse GPS measurements to estimate the pose of the rosbot 
1. The gps_to_utm.py will convert gps coordinates to utm that can be used 
2. The ekf_node_v2.py will run the ekf.
3. The rosbot_ekf_bag.launch will play the bag file and start the gps_to_utm and ekf_node and record the outputs into a new bag file. 

