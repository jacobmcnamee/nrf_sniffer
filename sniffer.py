import sys
import time
import binascii
import struct
import pylink

SYNC0 =                     0x53
SYNC1 =                     0x46

SYNC0_OFFSET =              0
SYNC1_OFFSET =              1
LENGTH_OFFSET =             2
CRC_OFFSET =                4
DATA_OFFSET =               8

HEADER_SIZE =               8

LENGTH_MAX =                512

def output_fields(fields):
    print(fields)
    sys.stdout.flush()

def payload_decode(payload):
    fields = { }

    fields["timestamp"] = struct.unpack_from("<I", payload, 0)[0]
    fields["rx_count"] = struct.unpack_from("<I", payload, 4)[0]
    fields["fifo_overflow_count"] = struct.unpack_from("<I", payload, 8)[0]
    fields["fifo_depth_max"] = struct.unpack_from("<B", payload, 12)[0]
    fields["rssi"] = struct.unpack_from("<B", payload, 13)[0]
    fields["data"] = payload[14:]

    output_fields(fields)

def packet_decode(data):
    if len(data) < HEADER_SIZE:
        return len(data) - HEADER_SIZE

    if data[SYNC0_OFFSET] != SYNC0 or data[SYNC1_OFFSET] != SYNC1:
        return 1

    length = struct.unpack_from("<H", data, LENGTH_OFFSET)[0]
    if length > LENGTH_MAX:
        return 1

    if len(data) < HEADER_SIZE + length:
        return len(data) - (HEADER_SIZE + length)

    frame = data[:HEADER_SIZE + length]

    packet_crc = struct.unpack_from("<I", data, CRC_OFFSET)[0]
    computed_crc = 0xffffffff
    computed_crc = binascii.crc32(frame[:CRC_OFFSET], computed_crc)
    computed_crc = binascii.crc32(frame[DATA_OFFSET:], computed_crc)
    if packet_crc != computed_crc:
        return 1

    payload = frame[DATA_OFFSET:]
    payload_decode(payload)
    return HEADER_SIZE + length

def decoder_process(decoder, data):
    decoder["buffer"] += data

    while True:
        ret = packet_decode(decoder["buffer"])
        if ret > 0:
            decoder["buffer"] = decoder["buffer"][ret:]
        else:
            return

def main(argv):
    if len(argv) >= 1:
        jlink_serial = argv[0]
    if len(argv) >= 2:
        device = argv[1]

    jlink = pylink.JLink()
    jlink.open(serial_no=jlink_serial)
    jlink.set_tif(pylink.enums.JLinkInterfaces.SWD)
    jlink.connect(device, verbose=True)
    jlink.rtt_start()

    decoder = { }
    decoder["buffer"] = bytearray()

    try:
        while jlink.connected():
            data = jlink.rtt_read(0, 1024)
            if data:
                decoder_process(decoder, bytes(data))
            else:
                time.sleep(0.001)
    except Exception:
        raise

if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
