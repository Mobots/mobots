# prio == 0: pose vor allen anderen einfügen, rest verwerfen (default)
# prio == -1: pose vor allen anderen einfügen
# prio == -2: pose am ende der liste einfügen
# prio == sonst: pose an der prio-position einfügen

geometry_msgs/Pose2D pose
int8 prio
