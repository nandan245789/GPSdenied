# Compressed obstacle map representation
builtin_interfaces/Time stamp
string frame_id
float32 resolution       # meters per voxel
float32 map_size_x       # meters
float32 map_size_y
float32 map_size_z
uint8[] data             # Serialized occupancy data
uint32 num_occupied      # Count of occupied voxels
uint32 num_free          # Count of free voxels
