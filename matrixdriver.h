#ifndef matrixdriver_matrixdriver_H
#define matrixdriver_matrixdriver_H

/**
 * Sets the bitbang delay, e.g. the time to wait between state-changes of
 * the gpio-line in microseconds.
 */
#define matrixdriver_IOCTL_BITBANG_DELAY _IO(0xaa, 0x01)

/**
 * Sets the drivers assumed size of the the devices in bytes. E.g. one single
 * matrixdriver is one byte long but they can be daisy-chained so two tpics
 * will be two bytes long etc.
 */
#define matrixdriver_IOCTL_SIZE _IO(0xaa, 0x03)

/**
 * Sets the nth bit of the device. All other bits remain untouched.
 */
#define matrixdriver_BIT_SET _IOW(0xaa, 0x04)

/**
 * Unsets the nth bit of the device. All other bits remain untouched.
 */
#define matrixdriver_BIT_UNSET _IOW(0xaa, 0x05)

#endif //matrixdriver_matrixdriver_H
