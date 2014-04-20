Header header

# Camera Matrix
sensor_msgs/CameraInfo camInfo

# Id
int16 frameId

# Image from matlab
sensor_msgs/Image img

# 2d points in the image plane with Z=id=descriptor
geometry_msgs/Point[] pts2d

# corresponding 3d ground truth points in world coordinates
geometry_msgs/Point[] pts3d

# Pose of camera in world coordinates
geometry_msgs/Transform camPose

# Pose of IMU in world coordinates
geometry_msgs/Transform imuPose

# Pose of CAM in IMU coordinates
geometry_msgs/Transform imu2cam

# Rotation of IMU in world frame
geometry_msgs/Transform imu

# NOISE TODO
# 2d points in the image plane + NOISE with Z=id=descriptor + OUTLIERS
geometry_msgs/Point[] pts2d_noise
