# Current navigation state with pose, velocity, and confidence
builtin_interfaces/Time stamp
geometry_msgs/PoseStamped pose       # 6-DoF pose in odom frame
geometry_msgs/TwistStamped twist     # Linear + angular velocity
float64[36] pose_covariance          # 6x6 covariance matrix (row-major)
float32 confidence                   # 0.0 (lost) to 1.0 (nominal)
bool vio_tracking                    # true if VIO is tracking
uint32 feature_count                 # Number of tracked visual features
