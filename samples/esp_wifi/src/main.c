/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>

#define CONFIG_POSIX_FS

#include "esp_wifi.h"
#include "stdlib.h"
#include "string.h"
#include "esp_wifi_internal.h"
#include "esp_dport_access.h"
#include "rom/ets_sys.h"
#include "esp_attr.h"
#include "esp_wifi_os_adapter.h"
#include "posix/pthread.h"
#include "esp_phy.h"
#include "wifi_system.h"
#include "esp_timer.h"

K_THREAD_STACK_DEFINE(wifi_stack, 4096);

#define portTICK_PERIOD_MS (1000 / 100)

#define CONFIG_ESP32_DPORT_DIS_INTERRUPT_LVL 5

static void *wifi_msgq_buffer;

typedef enum {
    ESP_LOG_NONE,       /*!< No log output */
    ESP_LOG_ERROR,      /*!< Critical errors, software module can not recover on its own */
    ESP_LOG_WARN,       /*!< Error conditions from which recovery measures have been taken */
    ESP_LOG_INFO,       /*!< Information messages which describe normal flow of events */
    ESP_LOG_DEBUG,      /*!< Extra information which is not necessary for normal use (values, pointers, sizes, etc). */
    ESP_LOG_VERBOSE     /*!< Bigger chunks of debugging information, or frequent messages which can potentially flood the output. */
} esp_log_level_t;

// static unsigned int intr_key;

const wpa_crypto_funcs_t g_wifi_default_wpa_crypto_funcs = {
    .size = sizeof(wpa_crypto_funcs_t),
    .version = ESP_WIFI_CRYPTO_VERSION};

uint32_t IRAM_ATTR esp_log_timestamp()
{
    return k_uptime_ticks();
}

uint32_t IRAM_ATTR esp_dport_access_reg_read(uint32_t reg)
{
    uint32_t apb;
    unsigned int intLvl;
    __asm__ __volatile__ (\
                  "rsil %[LVL], "XTSTR(CONFIG_ESP32_DPORT_DIS_INTERRUPT_LVL)"\n"\
                  "movi %[APB], "XTSTR(0x3ff40078)"\n"\
                  "l32i %[APB], %[APB], 0\n"\
                  "l32i %[REG], %[REG], 0\n"\
                  "wsr  %[LVL], "XTSTR(PS)"\n"\
                  "rsync\n"\
                  : [APB]"=a"(apb), [REG]"+a"(reg), [LVL]"=a"(intLvl)\
                  : \
                  : "memory" \
                  );
    return reg;
}

void IRAM_ATTR esp_log_writev(esp_log_level_t level,
        const char* tag,
        const char* format,
        va_list args)
{
    vprintf(format, args);
}

void esp_log_write(esp_log_level_t level,
                   const char *tag,
                   const char *format, ...)
{
    va_list list;
    va_start(list, format);
    esp_log_writev(level, tag, format, list);
    va_end(list);
}

static void IRAM_ATTR s_esp_dport_access_stall_other_cpu_start(void)
{

}

static void IRAM_ATTR s_esp_dport_access_stall_other_cpu_end(void)
{

}

/*
 If CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP is enabled. Prefer to allocate a chunk of memory in SPIRAM firstly.
 If failed, try to allocate it in internal memory then.
 */
IRAM_ATTR void *wifi_malloc( size_t size )
{
    void *ptr = k_malloc(size);
    if (ptr == NULL) {
        ets_printf("Malloc failed\n\n\n\n");
    }
    return ptr;
}

/*
 If CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP is enabled. Prefer to allocate a chunk of memory in SPIRAM firstly.
 If failed, try to allocate it in internal memory then.
 */
IRAM_ATTR void *wifi_realloc( void *ptr, size_t size )
{
    return realloc(ptr, size);
}

/*
 If CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP is enabled. Prefer to allocate a chunk of memory in SPIRAM firstly.
 If failed, try to allocate it in internal memory then.
 */
IRAM_ATTR void *wifi_calloc( size_t n, size_t size )
{
    return k_calloc(n, size);
}

static void * IRAM_ATTR wifi_zalloc_wrapper(size_t size)
{
    void *ptr = wifi_calloc(1, size);
    if (ptr) {
        memset(ptr, 0, size);
    }
    return ptr;
}

wifi_static_queue_t* wifi_create_queue( int queue_len, int item_size)
{
    wifi_static_queue_t *queue = NULL;

    queue = (wifi_static_queue_t*)k_malloc(sizeof(wifi_static_queue_t));
    if (!queue) {
        return NULL;
    }

#if CONFIG_SPIRAM_USE_MALLOC

    queue->storage = heap_caps_calloc(1, sizeof(StaticQueue_t) + (queue_len*item_size), MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
    if (!queue->storage) {
        goto _error;
    }

    queue->handle = xQueueCreateStatic( queue_len, item_size, ((uint8_t*)(queue->storage)) + sizeof(StaticQueue_t), (StaticQueue_t*)(queue->storage));

    if (!queue->handle) {
        goto _error;
    }

    return queue;

_error:
    if (queue) {
        if (queue->storage) {
            free(queue->storage);
        }

        free(queue);
    }

    return NULL;
#else

    wifi_msgq_buffer = k_malloc(queue_len * item_size);
    if (wifi_msgq_buffer == NULL) {
        ets_printf("Msg buffer allocation failed\n"); 
        return NULL;
    }
    queue->handle = k_malloc(sizeof(struct k_msgq));
    if (queue->handle == NULL) {
        k_free(wifi_msgq_buffer);
        ets_printf("Queue handle allocation failed\n");
        return NULL;
    }
	k_msgq_init((struct k_msgq *)queue->handle, wifi_msgq_buffer, item_size, queue_len);
    ets_printf("%p\n", queue->handle);
    return queue;
#endif
    return NULL;
}

void wifi_delete_queue(wifi_static_queue_t *queue)
{
    if (queue) {
        k_free(queue->handle);

#if CONFIG_SPIRAM_USE_MALLOC
        if (queue->storage) {
            free(queue->storage);
        }
#endif

        k_free(queue);
    }
}

static void * wifi_create_queue_wrapper(int queue_len, int item_size)
{
    return wifi_create_queue(queue_len, item_size);
}

static void wifi_delete_queue_wrapper(void *queue)
{
    wifi_delete_queue(queue);
}

static void * spin_lock_create_wrapper(void)
{
	struct k_spinlock *wifi_spin_lock = (struct k_spinlock *) k_malloc(sizeof(struct k_spinlock));
    return (void *)wifi_spin_lock;
}

static uint32_t IRAM_ATTR wifi_int_disable_wrapper(void *wifi_int_mux)
{
    unsigned int *int_mux = (unsigned int *) wifi_int_mux;
    *int_mux = irq_lock();
    return 0;
}

static void IRAM_ATTR wifi_int_restore_wrapper(void *wifi_int_mux, uint32_t tmp)
{
    unsigned int *key = (unsigned int *) wifi_int_mux;
    irq_unlock(*key);
}

static void IRAM_ATTR task_yield_from_isr_wrapper(void)
{
    k_yield();
}

static void * semphr_create_wrapper(uint32_t max, uint32_t init)
{
	struct k_sem *sem = (struct k_sem *) k_malloc(sizeof(struct k_sem));
	k_sem_init(sem, init, max);
	return (void *) sem;
}

static void semphr_delete_wrapper(void *semphr)
{
    k_free(semphr);
}

static void wifi_thread_semphr_free(void* data)
{
    if (data) {
        semphr_delete_wrapper(data);
    }
}

static void * wifi_thread_semphr_get_wrapper(void)
{
    static bool s_wifi_thread_sem_key_init = false;
    static pthread_key_t s_wifi_thread_sem_key;
    struct k_sem *sem = NULL;

    if (s_wifi_thread_sem_key_init == false) {
        if (0 != pthread_key_create(&s_wifi_thread_sem_key, wifi_thread_semphr_free)) {
            return NULL;
        }
        s_wifi_thread_sem_key_init = true;
    } else {
        sem = pthread_getspecific(s_wifi_thread_sem_key);
    }
    if (!sem) {
        sem = (struct k_sem *) k_malloc(sizeof(struct k_sem));
        k_sem_init(sem, 0, 1);
        if (sem) {
            pthread_setspecific(s_wifi_thread_sem_key, sem);
            // ESP_LOGV(TAG, "thread sem create: sem=%p", sem);
        }
    }
    return (void*)sem;
}

// static int32_t IRAM_ATTR semphr_take_from_isr_wrapper(void *semphr, void *hptw)
// {
//     return 0;
//     // return (int32_t)k_sem_take((struct k_sem *)semphr, K_MSEC(0));
// }

// static int32_t IRAM_ATTR semphr_give_from_isr_wrapper(void *semphr, void *hptw)
// {
//     // k_sem_give((struct k_sem *)semphr);
// 	return 0;
// }

static int32_t semphr_take_wrapper(void *semphr, uint32_t block_time_tick)
{
    if (block_time_tick == OSI_FUNCS_TIME_BLOCKING) {
        int ret = k_sem_take((struct k_sem *)semphr, K_FOREVER);
        if (ret == 0) {
            return 1;
        }
    } else {
        int ms = block_time_tick * portTICK_PERIOD_MS;
        int ret = k_sem_take((struct k_sem *)semphr, K_MSEC(ms));
        if (ret == 0) {
            return 1;
        }
    }
    return 0;
}

static int32_t semphr_give_wrapper(void *semphr)
{
	k_sem_give((struct k_sem *) semphr);
    return 1;
}

static void * recursive_mutex_create_wrapper(void)
{
    struct k_mutex *my_mutex = (struct k_mutex *) k_malloc(sizeof(struct k_mutex));
	k_mutex_init(my_mutex);
    ets_printf("Created %p mutex recursive\n", my_mutex);
    return (void *)my_mutex;
}

static void * mutex_create_wrapper(void)
{
    struct k_mutex *my_mutex = (struct k_mutex *) k_malloc(sizeof(struct k_mutex));
	k_mutex_init(my_mutex);
    ets_printf("Created %p mutex\n", my_mutex);
    return (void *)my_mutex;
}

static void mutex_delete_wrapper(void *mutex)
{
    // vSemaphoreDelete(mutex);
}

static int32_t IRAM_ATTR mutex_lock_wrapper(void *mutex)
{
    struct k_mutex *my_mutex = (struct k_mutex *) mutex;
	k_mutex_lock(my_mutex, K_FOREVER);
    return 0;
}

static int32_t IRAM_ATTR mutex_unlock_wrapper(void *mutex)
{
    struct k_mutex *my_mutex = (struct k_mutex *) mutex;
	k_mutex_unlock(my_mutex);
    return 0;
}

static void * queue_create_wrapper(uint32_t queue_len, uint32_t item_size)
{
	struct k_queue *queue = (struct k_queue *)k_malloc(sizeof(struct k_queue));
    k_msgq_init((struct k_msgq *)queue, wifi_msgq_buffer, item_size, queue_len);
    return (void *)queue;
}

static void delete_wrapper(void *handle)
{
    k_free(handle);
}

static int32_t queue_send_wrapper(void *queue, void *item, uint32_t block_time_tick)
{
    if (block_time_tick == OSI_FUNCS_TIME_BLOCKING) {
        k_msgq_put((struct k_msgq *)queue, item, K_FOREVER);
    } else {
        int ms = block_time_tick * portTICK_PERIOD_MS;
        k_msgq_put((struct k_msgq *)queue, item, K_MSEC(ms));
    }
    return 1;
}

static int32_t IRAM_ATTR queue_send_from_isr_wrapper(void *queue, void *item, void *hptw)
{
    int *hpt = (int *) hptw;
    k_msgq_put((struct k_msgq *)queue, item, K_NO_WAIT);
    *hpt = 0;
    return 1;
}

int32_t queue_send_to_back_wrapper(void *queue, void *item, uint32_t block_time_tick)
{
    return 0;
}

int32_t queue_send_to_front_wrapper(void *queue, void *item, uint32_t block_time_tick)
{
    return 0;
}

static int32_t queue_recv_wrapper(void *queue, void *item, uint32_t block_time_tick)
{
    if (block_time_tick == OSI_FUNCS_TIME_BLOCKING) {
        k_msgq_get((struct k_msgq *)queue, item, K_FOREVER);
    } else {
        int ms = block_time_tick * portTICK_PERIOD_MS;
        k_msgq_get((struct k_msgq *)queue, item, K_MSEC(ms));
    }
    return 1;
}

static uint32_t event_group_wait_bits_wrapper(void *event, uint32_t bits_to_wait_for, int clear_on_exit, int wait_for_all_bits, uint32_t block_time_tick)
{
    return 0;
}

static int32_t task_create_pinned_to_core_wrapper(void *task_func, const char *name, uint32_t stack_depth, void *param, uint32_t prio, void *task_handle, uint32_t core_id)
{
    task_handle = k_malloc(sizeof(struct k_thread));
	k_thread_create((struct k_thread *)task_handle, wifi_stack, stack_depth,
		(k_thread_entry_t)task_func, param, NULL, NULL,
		prio, K_INHERIT_PERMS, K_NO_WAIT);
	return 1;
}

static int32_t task_create_wrapper(void *task_func, const char *name, uint32_t stack_depth, void *param, uint32_t prio, void *task_handle)
{
	k_thread_create((struct k_thread *)task_handle, wifi_stack, stack_depth,
		(k_thread_entry_t)task_func, param, NULL, NULL,
		prio, K_INHERIT_PERMS, K_NO_WAIT);
	return 1;
}

static int32_t IRAM_ATTR task_ms_to_tick_wrapper(uint32_t ms)
{
    return (int32_t)(ms / portTICK_PERIOD_MS);
}

static int32_t task_get_max_priority_wrapper(void)
{
    return (int32_t)(0);
}

// static int32_t esp_event_post_wrapper(const char* event_base, int32_t event_id, void* event_data, size_t event_data_size, uint32_t ticks_to_wait)
// {
//     if (ticks_to_wait == OSI_FUNCS_TIME_BLOCKING) {
//         return (int32_t)esp_event_post(event_base, event_id, event_data, event_data_size, 0xffffffff);
//     } else {
//         return (int32_t)esp_event_post(event_base, event_id, event_data, event_data_size, ticks_to_wait);
//     }
// }

static void IRAM_ATTR timer_arm_wrapper(void *timer, uint32_t tmout, bool repeat)
{
    ets_timer_arm(timer, tmout, repeat);
}

static void IRAM_ATTR timer_disarm_wrapper(void *timer)
{
    ets_timer_disarm(timer);
}

static void timer_done_wrapper(void *ptimer)
{
    ets_timer_done(ptimer);
}

static void timer_setfn_wrapper(void *ptimer, void *pfunction, void *parg)
{
    ets_timer_setfn(ptimer, pfunction, parg);
}

static void IRAM_ATTR timer_arm_us_wrapper(void *ptimer, uint32_t us, bool repeat)
{
    ets_timer_arm_us(ptimer, us, repeat);
}

static int get_time_wrapper(void *t)
{
    return 0;
}

static void * IRAM_ATTR malloc_internal_wrapper(size_t size)
{
    void *ptr = k_malloc(size);
    if (ptr == NULL) {
        ets_printf("Malloc failed\n");
    }
    return ptr;
}

static void * IRAM_ATTR realloc_internal_wrapper(void *ptr, size_t size)
{
    return realloc(ptr, size);
}

static void * IRAM_ATTR calloc_internal_wrapper(size_t n, size_t size)
{
    return k_calloc(n, size);
}

static void * IRAM_ATTR zalloc_internal_wrapper(size_t size)
{
    void *ptr = calloc_internal_wrapper(1, size);
    if (ptr) {
        memset(ptr, 0, size);
    }
    return ptr;
}

// void vQueueDelete(void *ptr)
// {

// }

uint32_t uxQueueMessagesWaiting(void *queue)
{
    return 0;
}

void * xEventGroupCreate(void)
{
    return NULL;
}

void vEventGroupDelete(void *grp)
{
}

uint32_t xEventGroupSetBits(void *ptr,uint32_t data)
{
    return 0;
}

uint32_t xEventGroupClearBits(void *ptr,uint32_t data)
{
    return 0;
}

// void vTaskDelete(void *ptr)
// {

// }

void *xTaskGetCurrentTaskHandle(void)
{
    return (void *)k_current_get();
}

void vTaskDelay(uint32_t ticks)
{
    int ms = ticks * portTICK_PERIOD_MS;
    k_sleep(K_MSEC(ms));
}

static int coex_wifi_request_wrapper(uint32_t event, uint32_t latency, uint32_t duration)
{
    return 0;
}

static int coex_wifi_release_wrapper(uint32_t event)
{
    return 0;
}

static void set_isr_wrapper(int32_t n, void *f, void *arg)
{
    // IRQ_DIRECT_CONNECT(27, n,(const void *) f, 0);
    irq_disable(0);
    irq_connect_dynamic(0, n, f, arg, 0);
}

static void intr_on(unsigned int mask)
{
    irq_enable(0);
}

static void intr_off(unsigned int mask)
{
    irq_disable(0);
}

static uint32_t coex_status_get_wrapper(void)
{
#if CONFIG_SW_COEXIST_ENABLE
    return coex_status_get();
#else
    return 0;
#endif
}

static void coex_condition_set_wrapper(uint32_t type, bool dissatisfy)
{
#if CONFIG_SW_COEXIST_ENABLE
    coex_condition_set(type, dissatisfy);
#endif
}

uint32_t esp_get_free_heap_size(void)
{
    return 10000;
}

uint32_t zephyr_random(void)
{
    return 0x2848;
}

int32_t os_random(uint8_t *buf, unsigned int len)
{
    return 0x2848;
}

unsigned long random(void)
{
    return 0x2848;
}

int32_t esp_modem_sleep_deregister(uint32_t module)
{
    return 0;
}

void sc_ack_send_wrapper(void *param)
{
}

void sc_ack_send_stop(void)
{
}

wifi_osi_funcs_t g_wifi_osi_funcs = {
    ._version = ESP_WIFI_OS_ADAPTER_VERSION,
    ._set_isr = set_isr_wrapper,
    ._ints_on = intr_on,
    ._ints_off = intr_off,
    ._spin_lock_create = spin_lock_create_wrapper,
    ._spin_lock_delete = k_free,
    ._wifi_int_disable = wifi_int_disable_wrapper,
    ._wifi_int_restore = wifi_int_restore_wrapper,
    ._task_yield_from_isr = task_yield_from_isr_wrapper,
    ._semphr_create = semphr_create_wrapper,
    ._semphr_delete = semphr_delete_wrapper,
    ._semphr_take = semphr_take_wrapper,
    ._semphr_give = semphr_give_wrapper,
    ._wifi_thread_semphr_get = wifi_thread_semphr_get_wrapper,
    ._mutex_create = mutex_create_wrapper,
    ._recursive_mutex_create = recursive_mutex_create_wrapper,
    ._mutex_delete = mutex_delete_wrapper,
    ._mutex_lock = mutex_lock_wrapper,
    ._mutex_unlock = mutex_unlock_wrapper,
    ._queue_create = queue_create_wrapper,
    ._queue_delete = delete_wrapper,
    ._queue_send = queue_send_wrapper,
    ._queue_send_from_isr = queue_send_from_isr_wrapper,
    ._queue_send_to_back = queue_send_to_back_wrapper,
    ._queue_send_to_front = queue_send_to_front_wrapper,
    ._queue_recv = queue_recv_wrapper,
    ._queue_msg_waiting = uxQueueMessagesWaiting,
    ._event_group_create = (void *(*)(void))xEventGroupCreate,
    ._event_group_delete = (void(*)(void *))vEventGroupDelete,
    ._event_group_set_bits = xEventGroupSetBits,
    ._event_group_clear_bits = xEventGroupClearBits,
    ._event_group_wait_bits = event_group_wait_bits_wrapper,
    ._task_create_pinned_to_core = task_create_pinned_to_core_wrapper,
    ._task_create = task_create_wrapper,
    ._task_delete = delete_wrapper,
    ._task_delay = vTaskDelay,
    ._task_ms_to_tick = task_ms_to_tick_wrapper,
    ._task_get_current_task = (void *(*)(void))xTaskGetCurrentTaskHandle,
    ._task_get_max_priority = task_get_max_priority_wrapper,
    ._malloc = k_malloc,
    ._free = k_free,
    ._get_free_heap_size = esp_get_free_heap_size,
    ._rand = zephyr_random,
    ._dport_access_stall_other_cpu_start_wrap = s_esp_dport_access_stall_other_cpu_start,
    ._dport_access_stall_other_cpu_end_wrap = s_esp_dport_access_stall_other_cpu_end,
    ._phy_rf_deinit = esp_phy_rf_deinit,
    ._phy_load_cal_and_init = esp_phy_load_cal_and_init,
    ._phy_common_clock_enable = esp_phy_common_clock_enable,
    ._phy_common_clock_disable = esp_phy_common_clock_disable,
    ._read_mac = esp_read_mac,
    ._timer_arm = timer_arm_wrapper,
    ._timer_disarm = timer_disarm_wrapper,
    ._timer_done = timer_done_wrapper,
    ._timer_setfn = timer_setfn_wrapper,
    ._timer_arm_us = timer_arm_us_wrapper,
    ._periph_module_enable = periph_module_enable,
    ._periph_module_disable = periph_module_disable,
    ._esp_timer_get_time = esp_timer_get_time,
    // ._nvs_set_i8 = nvs_set_i8,
    // ._nvs_get_i8 = nvs_get_i8,
    // ._nvs_set_u8 = nvs_set_u8,
    // ._nvs_get_u8 = nvs_get_u8,
    // ._nvs_set_u16 = nvs_set_u16,
    // ._nvs_get_u16 = nvs_get_u16,
    // ._nvs_open = nvs_open,
    // ._nvs_close = nvs_close,
    // ._nvs_commit = nvs_commit,
    // ._nvs_set_blob = nvs_set_blob,
    // ._nvs_get_blob = nvs_get_blob,
    // ._nvs_erase_key = nvs_erase_key,
    ._get_random = os_random,
    ._get_time = get_time_wrapper,
    ._random = random,
    ._log_write = esp_log_write,
    ._log_writev = esp_log_writev,
    ._log_timestamp = esp_log_timestamp,
    ._malloc_internal =  malloc_internal_wrapper,
    ._realloc_internal = realloc_internal_wrapper,
    ._calloc_internal = calloc_internal_wrapper,
    ._zalloc_internal = zalloc_internal_wrapper,
    ._wifi_malloc = wifi_malloc,
    ._wifi_realloc = wifi_realloc,
    ._wifi_calloc = wifi_calloc,
    ._wifi_zalloc = wifi_zalloc_wrapper,
    ._wifi_create_queue = wifi_create_queue_wrapper,
    ._wifi_delete_queue = wifi_delete_queue_wrapper,
    ._modem_sleep_enter = esp_modem_sleep_enter,
    ._modem_sleep_exit = esp_modem_sleep_exit,
    ._modem_sleep_register = esp_modem_sleep_register,
    ._modem_sleep_deregister = esp_modem_sleep_deregister,
    ._sc_ack_send = sc_ack_send_wrapper,
    ._sc_ack_send_stop = sc_ack_send_stop,
    ._coex_status_get = coex_status_get_wrapper,
    ._coex_condition_set = coex_condition_set_wrapper,
    ._coex_wifi_request = coex_wifi_request_wrapper,
    ._coex_wifi_release = coex_wifi_release_wrapper,
    ._magic = ESP_WIFI_OS_ADAPTER_MAGIC,
};


// coex_adapter_funcs_t g_coex_adapter_funcs = {
//     ._version = 1,
//     ._spin_lock_create = spin_lock_create_wrapper,
//     ._spin_lock_delete = free,
//     ._int_disable = wifi_int_disable_wrapper,
//     ._int_enable = wifi_int_restore_wrapper,
//     ._task_yield_from_isr = task_yield_from_isr_wrapper,
//     ._semphr_create = semphr_create_wrapper,
//     ._semphr_delete = semphr_delete_wrapper,
//     ._semphr_take_from_isr = semphr_take_from_isr_wrapper,
//     ._semphr_give_from_isr = semphr_give_from_isr_wrapper,
//     ._semphr_take = semphr_take_wrapper,
//     ._semphr_give = semphr_give_wrapper,
//     ._is_in_isr = coex_is_in_isr_wrapper,
//     ._malloc_internal =  malloc_internal_wrapper,
//     ._free = free,
//     ._timer_disarm = timer_disarm_wrapper,
//     ._timer_done = timer_done_wrapper,
//     ._timer_setfn = timer_setfn_wrapper,
//     ._timer_arm_us = timer_arm_us_wrapper,
//     ._esp_timer_get_time = esp_timer_get_time,
//     ._magic = -559038801,
// };

esp_err_t esp_event_send(system_event_t *event) {
    ets_printf("%d is event\n", event->event_id);
	return ESP_OK;
}

/* copy paste from wifi_init.c */
esp_err_t esp_wifi_init(const wifi_init_config_t *config)
{
    //esp_event_set_default_wifi_handlers();
    return esp_wifi_init_internal(config);
}


void main(void)
{
    esp_timer_init();
	wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT() ;
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "Oneplus6T",
            .password = "",
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
            .threshold.authmode = WIFI_AUTH_OPEN,
        },
    };
	esp_err_t ret = esp_wifi_init(&config);
	ret |= esp_wifi_set_mode(WIFI_MODE_STA);
    ret |= esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    ret |= esp_wifi_start();
    if (ret != 0) {
        printk("Wifi Init Failed\n");
    } else {
        printk("Wifi Init Success\n");
    }
    esp_err_t err = esp_wifi_connect();
    if (err != ESP_OK) {
        ets_printf("Failed to call connect, %x\n", err);
    } else {
        ets_printf("Success connect\n");
    }
}
