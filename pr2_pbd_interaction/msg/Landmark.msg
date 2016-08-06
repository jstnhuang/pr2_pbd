uint8 TABLE_TOP=1
uint8 SURFACE=2
uint8 CLOUD_BOX=3
uint8 type
string name # Name of this landmark
geometry_msgs/Pose pose # Pose of this landmark, in the robot's base frame.
geometry_msgs/Vector3 dimensions # Dimensions of the bounding box around the object.
string db_id # MongoDB ID of this object, if type is CLOUD_BOX
