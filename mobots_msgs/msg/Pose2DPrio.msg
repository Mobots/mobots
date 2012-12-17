# prio == 0: pose vor allen anderen einfuegen, rest verwerfen (default)
# prio == -1: pose vor allen anderen einfuegen
# prio == -2: pose am ende der liste einfuegen
# prio == sonst: pose an der prio-position einfügen

geometry_msgs/Pose2D pose
int8 prio
