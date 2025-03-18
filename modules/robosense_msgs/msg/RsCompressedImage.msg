# rscompressedimage.msg 
uint8 JPEG = 0
uint8 H265_I = 1
uint8 H265_P = 2

uint8 CLIP_NOT_BELONG = 0
uint8 CLIP_BELONG     = 1

std_msgs/Header  header
uint8            type 
byte[]           data 
uint8            attach_type
int32            target_expose_time
int32            real_expose_time 
