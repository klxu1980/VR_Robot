import struct
import numpy as np


def read_float(buffer, pos):
    return struct.unpack("!f", [buffer[pos], buffer[pos + 1], buffer[pos + 2], buffer[pos + 3]])[0]


def write_float(value, buffer, pos):
    # 高位在前，低位在后
    bytes = struct.pack("f", value)
    buffer[pos] = bytes[0]
    buffer[pos + 1] = bytes[1]
    buffer[pos + 2] = bytes[2]
    buffer[pos + 3] = bytes[3]


if __name__ == '__main__':
    # buffer = np.zeros(4, dtype=np.uint8)
    # write_float(1.23, buffer, 0)
    # print(buffer)
    # v = read_float(buffer, 0)
    # print(v)

    f = np.zeros(2, dtype=np.float32)
    f[0] = 1.2
    f[1] = 2.4

    x = np.frombuffer(f.tobytes(), dtype=np.float32)
    print(x)