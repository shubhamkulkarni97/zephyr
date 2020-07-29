#include <zephyr.h>
#include "esp_event.h"
#include "esp_wifi_internal.h"
#include "esp_networking_priv.h"
#include "rom/ets_sys.h"

K_THREAD_STACK_DEFINE(event_task_stack, CONFIG_EVENT_TASK_STACK_SIZE);
static struct k_thread event_task_handle;
static struct k_msgq event_queue;
static void *event_msgq_buffer;

esp_err_t esp_event_send(system_event_t *event)
{
    k_msgq_put(&event_queue, (void *)event, K_FOREVER);
	return ESP_OK;
}

void event_task(void *arg)
{
    bool connected_flag = false;
    system_event_t event;
    while (1) {
        k_msgq_get(&event_queue, &event, K_FOREVER);
        switch (event.event_id) {
            case SYSTEM_EVENT_STA_START:
            {
                ets_printf("SYSTEM_EVENT_STA_START\n");
                break;
            }

            case SYSTEM_EVENT_STA_CONNECTED:
            {
                ets_printf("SYSTEM_EVENT_STA_CONNECTED\n");
                connected_flag = true;
                esp_wifi_register_rx_callback();
                break;
            }

            case SYSTEM_EVENT_STA_DISCONNECTED:
            {
                ets_printf("SYSTEM_EVENT_STA_DISCONNECTED\n");
                if (!connected_flag) {
                    esp_wifi_connect();
                }
                connected_flag = false;
                break;
            }
            default:
                    break;
        }
    }
}

esp_err_t esp_event_init(void)
{
    event_msgq_buffer = k_malloc(sizeof(system_event_t) * 10);
    if (event_msgq_buffer == NULL) {
        return ESP_ERR_NO_MEM;
    }
    k_msgq_init(&event_queue, event_msgq_buffer, sizeof(system_event_t), 10);
    k_thread_create(&event_task_handle, event_task_stack, CONFIG_EVENT_TASK_STACK_SIZE,
		(k_thread_entry_t)event_task, NULL, NULL, NULL,
		CONFIG_EVENT_TASK_PRIO, K_INHERIT_PERMS, K_NO_WAIT);

    return ESP_OK;
}
