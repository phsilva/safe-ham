#ifndef _INCLUDE_USB_H_
#define _INCLUDE_USB_H_ 1

#include <libopencm3/usb/usbd.h>

#ifdef __cplusplus
extern "C" {
#endif

void setup_usb(usbd_device **usbd_dev);

extern usbd_device *globalUSB;
extern uint8_t usb_connected;
extern uint8_t dw_run;

#ifdef __cplusplus
}
#endif

#endif // !_INCLUDE_USB_H_
