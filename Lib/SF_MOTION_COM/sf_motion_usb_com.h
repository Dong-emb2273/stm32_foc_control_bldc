#ifndef SF_MOTION_USB_COM_H
#define SF_MOTION_USB_COM_H

#include <stdint.h>

#define SFM_USB_COM_FRAME_HEADER 0xA5A5
#define SFM_USB_COM_DATA_TX_MAX  32

typedef struct {
    uint8_t *data_rx;
    uint16_t data_rx_len;
    uint8_t data_tx[SFM_USB_COM_DATA_TX_MAX];
    uint16_t data_tx_len;
    int (*send_data)(uint8_t*, uint16_t);
} sfm_usb_com_t;

void sfm_usb_com_init(sfm_usb_com_t *usb, int (*send_data)(uint8_t*, uint16_t));
int8_t sfm_usb_com_start_send_data(sfm_usb_com_t *usb, uint8_t *data, uint16_t len);
int8_t sfm_usb_com_receive_data(sfm_usb_com_t *usb, uint8_t *data_out, uint16_t *data_out_len);

#endif /* SF_MOTION_USB_COM_H */