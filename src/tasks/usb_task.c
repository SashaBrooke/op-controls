#include "tusb.h"
#include "tasks/usb_task.h"

void usb_task(void *pvParameters) {
    (void)pvParameters;

    for (;;) {
        tud_task();
    }
}
