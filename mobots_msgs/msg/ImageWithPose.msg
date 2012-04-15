# This represents an image together with the pose at which it was taken.

uint8 mobotID		                          # ID of the Mobot which sent the image
sensor_msgs/Image image
geometry_msgs/PoseWithCovarianceStamped pose  # Maybe use Pose2DWithCovarianceStamped...
