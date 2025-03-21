RsHeader header           # Header timestamp should be acquisition time of image

uint32 height             # image height, that is, number of rows
uint32 width              # image width, that is, number of columns

char[8] encoding          # Encoding of pixels -- channel meaning, ordering, size
                          # taken from the list of strings in include/sensor_msgs/image_encodings.hpp

uint8 is_bigendian        # is this data bigendian?
uint32 step               # Full row length in bytes
uint8[6220800] data       # 1920*1080*3