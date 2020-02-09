#ifndef IOCTL1_H
#define IOCTL1_H

#include <linux/ioctl.h>

#define MAGIC_NO 'k'
#define SET_CHANNEL_SEQ_NO 1
#define SET_ALIGNMENT_SEQ_NO 2

//IOCTL CALL
#define SET_CHANNEL _IOW(MAGIC_NO,SET_CHANNEL_SEQ_NO ,int)
#define SET_ALIGNMENT _IOW(MAGIC_NO,SET_ALIGNMENT_SEQ_NO,int)
#define DEVICE_FILE_NAME "/dev/adc8"
#endif
