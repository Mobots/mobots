"""autogenerated by genpy from mobots_msgs/CvMat.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class CvMat(genpy.Message):
  _md5sum = "faf7fa27dead12d88d281e8b4e661e76"
  _type = "mobots_msgs/CvMat"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """uint32 rows
uint32 cols
int32 type
uint32 elemSize
uint8[] data

"""
  __slots__ = ['rows','cols','type','elemSize','data']
  _slot_types = ['uint32','uint32','int32','uint32','uint8[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       rows,cols,type,elemSize,data

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(CvMat, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.rows is None:
        self.rows = 0
      if self.cols is None:
        self.cols = 0
      if self.type is None:
        self.type = 0
      if self.elemSize is None:
        self.elemSize = 0
      if self.data is None:
        self.data = ''
    else:
      self.rows = 0
      self.cols = 0
      self.type = 0
      self.elemSize = 0
      self.data = ''

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_struct_2IiI.pack(_x.rows, _x.cols, _x.type, _x.elemSize))
      _x = self.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 16
      (_x.rows, _x.cols, _x.type, _x.elemSize,) = _struct_2IiI.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.data = str[start:end].decode('utf-8')
      else:
        self.data = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_2IiI.pack(_x.rows, _x.cols, _x.type, _x.elemSize))
      _x = self.data
      length = len(_x)
      # - if encoded as a list instead, serialize as bytes instead of string
      if type(_x) in [list, tuple]:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 16
      (_x.rows, _x.cols, _x.type, _x.elemSize,) = _struct_2IiI.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.data = str[start:end].decode('utf-8')
      else:
        self.data = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_2IiI = struct.Struct("<2IiI")
