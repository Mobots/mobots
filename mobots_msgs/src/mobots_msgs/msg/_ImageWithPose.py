"""autogenerated by genmsg_py from ImageWithPose.msg. Do not edit."""
import roslib.message
import struct

import geometry_msgs.msg
import std_msgs.msg
import sensor_msgs.msg

class ImageWithPose(roslib.message.Message):
  _md5sum = "785d4424ae261db4966f618b95922857"
  _type = "mobots_msgs/ImageWithPose"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# This represents an image together with the pose at which it was taken.

uint8 mobotID		                          # ID of the Mobot which sent the image
sensor_msgs/CompressedImage image
geometry_msgs/PoseWithCovarianceStamped pose  # Maybe use Pose2DWithCovarianceStamped...

================================================================================
MSG: sensor_msgs/CompressedImage
# This message contains a compressed image

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of cameara
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image

string format        # Specifies the format of the data
                     #   Acceptable values:
                     #     jpeg, png
uint8[] data         # Compressed image buffer

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/PoseWithCovarianceStamped
# This expresses an estimated pose with a reference coordinate frame and timestamp

Header header
PoseWithCovariance pose

================================================================================
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of postion and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

"""
  __slots__ = ['mobotID','image','pose']
  _slot_types = ['uint8','sensor_msgs/CompressedImage','geometry_msgs/PoseWithCovarianceStamped']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       mobotID,image,pose
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(ImageWithPose, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.mobotID is None:
        self.mobotID = 0
      if self.image is None:
        self.image = sensor_msgs.msg.CompressedImage()
      if self.pose is None:
        self.pose = geometry_msgs.msg.PoseWithCovarianceStamped()
    else:
      self.mobotID = 0
      self.image = sensor_msgs.msg.CompressedImage()
      self.pose = geometry_msgs.msg.PoseWithCovarianceStamped()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      _x = self
      buff.write(_struct_B3I.pack(_x.mobotID, _x.image.header.seq, _x.image.header.stamp.secs, _x.image.header.stamp.nsecs))
      _x = self.image.header.frame_id
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.image.format
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.image.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_3I.pack(_x.pose.header.seq, _x.pose.header.stamp.secs, _x.pose.header.stamp.nsecs))
      _x = self.pose.header.frame_id
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_7d.pack(_x.pose.pose.pose.position.x, _x.pose.pose.pose.position.y, _x.pose.pose.pose.position.z, _x.pose.pose.pose.orientation.x, _x.pose.pose.pose.orientation.y, _x.pose.pose.pose.orientation.z, _x.pose.pose.pose.orientation.w))
      buff.write(_struct_36d.pack(*self.pose.pose.covariance))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      if self.image is None:
        self.image = sensor_msgs.msg.CompressedImage()
      if self.pose is None:
        self.pose = geometry_msgs.msg.PoseWithCovarianceStamped()
      end = 0
      _x = self
      start = end
      end += 13
      (_x.mobotID, _x.image.header.seq, _x.image.header.stamp.secs, _x.image.header.stamp.nsecs,) = _struct_B3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.image.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.image.format = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.image.data = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.pose.header.seq, _x.pose.header.stamp.secs, _x.pose.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.pose.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 56
      (_x.pose.pose.pose.position.x, _x.pose.pose.pose.position.y, _x.pose.pose.pose.position.z, _x.pose.pose.pose.orientation.x, _x.pose.pose.pose.orientation.y, _x.pose.pose.pose.orientation.z, _x.pose.pose.pose.orientation.w,) = _struct_7d.unpack(str[start:end])
      start = end
      end += 288
      self.pose.pose.covariance = _struct_36d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      _x = self
      buff.write(_struct_B3I.pack(_x.mobotID, _x.image.header.seq, _x.image.header.stamp.secs, _x.image.header.stamp.nsecs))
      _x = self.image.header.frame_id
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.image.format
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.image.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_3I.pack(_x.pose.header.seq, _x.pose.header.stamp.secs, _x.pose.header.stamp.nsecs))
      _x = self.pose.header.frame_id
      length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_7d.pack(_x.pose.pose.pose.position.x, _x.pose.pose.pose.position.y, _x.pose.pose.pose.position.z, _x.pose.pose.pose.orientation.x, _x.pose.pose.pose.orientation.y, _x.pose.pose.pose.orientation.z, _x.pose.pose.pose.orientation.w))
      buff.write(self.pose.pose.covariance.tostring())
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      if self.image is None:
        self.image = sensor_msgs.msg.CompressedImage()
      if self.pose is None:
        self.pose = geometry_msgs.msg.PoseWithCovarianceStamped()
      end = 0
      _x = self
      start = end
      end += 13
      (_x.mobotID, _x.image.header.seq, _x.image.header.stamp.secs, _x.image.header.stamp.nsecs,) = _struct_B3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.image.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.image.format = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.image.data = str[start:end]
      _x = self
      start = end
      end += 12
      (_x.pose.header.seq, _x.pose.header.stamp.secs, _x.pose.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      self.pose.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 56
      (_x.pose.pose.pose.position.x, _x.pose.pose.pose.position.y, _x.pose.pose.pose.position.z, _x.pose.pose.pose.orientation.x, _x.pose.pose.pose.orientation.y, _x.pose.pose.pose.orientation.z, _x.pose.pose.pose.orientation.w,) = _struct_7d.unpack(str[start:end])
      start = end
      end += 288
      self.pose.pose.covariance = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=36)
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_36d = struct.Struct("<36d")
_struct_3I = struct.Struct("<3I")
_struct_7d = struct.Struct("<7d")
_struct_B3I = struct.Struct("<B3I")