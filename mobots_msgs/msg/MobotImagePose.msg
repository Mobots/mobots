# This message contains a compressed image, pose, and mobotID
# It is an extension ImageWithPose.msg

Header header		# Header timestamp should be acquisition time of image
					# Header frame_id should be optical frame of camera
					# origin of frame should be optical center of cameara
					# +x should point to the right in the image
					# +y should point down in the image
					# +z should point into to plane of the image

ImageWithPose image # Contains the image and pose information

int8 mobotID		# ID of the Mobot which sent the image