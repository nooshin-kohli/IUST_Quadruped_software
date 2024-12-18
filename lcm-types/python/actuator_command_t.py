"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class actuator_command_t(object):
    __slots__ = ["id", "kp", "kd", "position", "velocity", "tau"]

    __typenames__ = ["int64_t", "double", "double", "double", "double", "double"]

    __dimensions__ = [None, None, None, None, None, None]

    def __init__(self):
        self.id = 0
        self.kp = 0.0
        self.kd = 0.0
        self.position = 0.0
        self.velocity = 0.0
        self.tau = 0.0

    def encode(self):
        buf = BytesIO()
        buf.write(actuator_command_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qddddd", self.id, self.kp, self.kd, self.position, self.velocity, self.tau))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != actuator_command_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return actuator_command_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = actuator_command_t()
        self.id, self.kp, self.kd, self.position, self.velocity, self.tau = struct.unpack(">qddddd", buf.read(48))
        return self
    _decode_one = staticmethod(_decode_one)

    def _get_hash_recursive(parents):
        if actuator_command_t in parents: return 0
        tmphash = (0xd60841f5931a9d84) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if actuator_command_t._packed_fingerprint is None:
            actuator_command_t._packed_fingerprint = struct.pack(">Q", actuator_command_t._get_hash_recursive([]))
        return actuator_command_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

    def get_hash(self):
        """Get the LCM hash of the struct"""
        return struct.unpack(">Q", actuator_command_t._get_packed_fingerprint())[0]

