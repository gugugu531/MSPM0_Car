#ifndef APP_DEVICE_CHECK_H
#define APP_DEVICE_CHECK_H

#include <stdint.h>

void AppDeviceCheck_Run(void);
void AppDeviceCheck_ProcessImuByte(uint8_t byte);

#endif /* APP_DEVICE_CHECK_H */
