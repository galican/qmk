/* Copyright (C) 2023 Westberry Technology (ChangZhou) Corp., Ltd
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifdef NO_USB_STARTUP_CHECK

#    include <ch.h>
#    include <hal.h>

#    include "usb_main.h"

/* TMK includes */
#    include "action_util.h"
#    include "mousekey.h"
#    include "print.h"

#    include "suspend.h"
#    include "wait.h"

#    include "quantum.h"
#    include "common/bt_task.h"

void restart_usb_driver(USBDriver *usbp) {}

void housekeeping_task_bt(void) {
    if (dev_info.devs == DEVS_USB) {
        if ((USB_DRIVER.state == USB_SUSPENDED) && (USB_DRIVER.saved_state == USB_ACTIVE)) {
            print("[s]");
            while (USB_DRIVER.state == USB_SUSPENDED) {
                if (gpio_read_pin(MM_CABLE_PIN)) break;
                /* Do this in the suspended state */
                suspend_power_down(); // on AVR this deep sleeps for 15ms
                /* Remote wakeup */
                if (suspend_wakeup_condition()) {
                    usbWakeupHost(&USB_DRIVER);
                    restart_usb_driver(&USB_DRIVER);
                }
            }
            /* Woken up */
            // variables has been already cleared by the wakeup hook
            send_keyboard_report();
#    ifdef MOUSEKEY_ENABLE
            mousekey_send();
#    endif /* MOUSEKEY_ENABLE */
        }
    }
}
#endif
