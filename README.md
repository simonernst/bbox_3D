# bbox_3D


Description : 

Receive the BoudinngBoxes message from darknet_ros
Calculate the center coordinates of the bbox and associate it with the PointCloud to create a 3D position in the camera link
Convert the 3D position into the map frame
Save it as an interest point in the MapManager


Usage : roslaunch coordinates_point_cloud tf_bbox.launch


