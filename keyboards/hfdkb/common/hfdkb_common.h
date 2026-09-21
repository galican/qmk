#ifndef _HFDKB_COMMON_H_
#define _HFDKB_COMMON_H_

#include "quantum.h"

void wireless_post_init(void);
void wireless_wakeup_init(void);
bool wireless_process_record(uint16_t keycode, keyrecord_t *record);
bool wireless_indicators_advanced(uint8_t led_min, uint8_t led_max);

#endif /* _HFDKB_COMMON_H_ */
