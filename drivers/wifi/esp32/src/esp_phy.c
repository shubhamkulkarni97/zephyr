#include <zephyr.h>
#include "esp_phy.h"
#include "rom/ets_sys.h"
#include "phy.h"
#include "esp_wifi_internal.h"

static volatile int32_t s_common_clock_enable_ref = 0;
static unsigned int intr_key;
static struct k_spinlock s_modem_sleep_lock;

/* Bit mask of modules needing to call phy_rf_init */
static uint32_t s_module_phy_rf_init = 0;

/* Whether modem sleep is turned on */
static volatile bool s_is_phy_rf_en = false;

/* time stamp updated when the PHY/RF is turned on */
static int64_t s_phy_rf_en_ts = 0;

/* Bit mask of modules which might use RF, system can enter modem
 * sleep mode only when all modules registered require to enter
 * modem sleep*/
static uint32_t s_modem_sleep_module_register = 0;

/* Bit mask of modules needing to enter modem sleep mode */
static uint32_t s_modem_sleep_module_enter = 0;

/* Whether modern sleep is turned on */
static volatile bool s_is_modem_sleep_en = false;

static bool is_wifi_clk_peripheral(periph_module_t periph)
{
    /* A small subset of peripherals use WIFI_CLK_EN_REG and
       CORE_RST_EN_REG for their clock & reset registers */
    switch(periph) {
    case PERIPH_SDMMC_MODULE:
    case PERIPH_SDIO_SLAVE_MODULE:
    case PERIPH_EMAC_MODULE:
    case PERIPH_RNG_MODULE:
    case PERIPH_WIFI_MODULE:
    case PERIPH_BT_MODULE:
    case PERIPH_WIFI_BT_COMMON_MODULE:
    case PERIPH_BT_BASEBAND_MODULE:
    case PERIPH_BT_LC_MODULE:
        return true;
    default:
        return false;
    }
}

static uint32_t get_clk_en_reg(periph_module_t periph)
{
    if (periph == PERIPH_AES_MODULE || periph == PERIPH_SHA_MODULE || periph == PERIPH_RSA_MODULE) {
        return DPORT_PERI_CLK_EN_REG;
    } else {
        return is_wifi_clk_peripheral(periph) ? DPORT_WIFI_CLK_EN_REG : DPORT_PERIP_CLK_EN_REG;
    }
}

static uint32_t get_clk_en_mask(periph_module_t periph)
{
    switch(periph) {
        case PERIPH_RMT_MODULE:
            return DPORT_RMT_CLK_EN;
        case PERIPH_LEDC_MODULE:
            return DPORT_LEDC_CLK_EN;
        case PERIPH_UART0_MODULE:
            return DPORT_UART_CLK_EN;
        case PERIPH_UART1_MODULE:
            return DPORT_UART1_CLK_EN;
        case PERIPH_UART2_MODULE:
            return DPORT_UART2_CLK_EN;
        case PERIPH_I2C0_MODULE:
            return DPORT_I2C_EXT0_CLK_EN;
        case PERIPH_I2C1_MODULE:
            return DPORT_I2C_EXT1_CLK_EN;
        case PERIPH_I2S0_MODULE:
            return DPORT_I2S0_CLK_EN;
        case PERIPH_I2S1_MODULE:
            return DPORT_I2S1_CLK_EN;
        case PERIPH_TIMG0_MODULE:
            return DPORT_TIMERGROUP_CLK_EN;
        case PERIPH_TIMG1_MODULE:
            return DPORT_TIMERGROUP1_CLK_EN;
        case PERIPH_PWM0_MODULE:
            return DPORT_PWM0_CLK_EN;
        case PERIPH_PWM1_MODULE:
            return DPORT_PWM1_CLK_EN;
        case PERIPH_PWM2_MODULE:
            return DPORT_PWM2_CLK_EN;
        case PERIPH_PWM3_MODULE:
            return DPORT_PWM3_CLK_EN;
        case PERIPH_UHCI0_MODULE:
            return DPORT_UHCI0_CLK_EN;
        case PERIPH_UHCI1_MODULE:
            return DPORT_UHCI1_CLK_EN;
        case PERIPH_PCNT_MODULE:
            return DPORT_PCNT_CLK_EN;
        case PERIPH_SPI_MODULE:
            return DPORT_SPI01_CLK_EN;
        case PERIPH_HSPI_MODULE:
            return DPORT_SPI2_CLK_EN;
        case PERIPH_VSPI_MODULE:
            return DPORT_SPI3_CLK_EN;
        case PERIPH_SPI_DMA_MODULE:
            return DPORT_SPI_DMA_CLK_EN;
        case PERIPH_SDMMC_MODULE:
            return DPORT_WIFI_CLK_SDIO_HOST_EN;
        case PERIPH_SDIO_SLAVE_MODULE:
            return DPORT_WIFI_CLK_SDIOSLAVE_EN;
        case PERIPH_CAN_MODULE:
            return DPORT_CAN_CLK_EN;
        case PERIPH_EMAC_MODULE:
            return DPORT_WIFI_CLK_EMAC_EN;
        case PERIPH_RNG_MODULE:
            return DPORT_WIFI_CLK_RNG_EN;
        case PERIPH_WIFI_MODULE:
            return DPORT_WIFI_CLK_WIFI_EN_M;
        case PERIPH_BT_MODULE:
            return DPORT_WIFI_CLK_BT_EN_M;
        case PERIPH_WIFI_BT_COMMON_MODULE:
            return DPORT_WIFI_CLK_WIFI_BT_COMMON_M;
        case PERIPH_BT_BASEBAND_MODULE:
            return DPORT_BT_BASEBAND_EN;
        case PERIPH_BT_LC_MODULE:
            return DPORT_BT_LC_EN;
        case PERIPH_AES_MODULE:
            return DPORT_PERI_EN_AES;
        case PERIPH_SHA_MODULE:
            return DPORT_PERI_EN_SHA;
        case PERIPH_RSA_MODULE:
            return DPORT_PERI_EN_RSA;
        default:
            return 0;
    }
}

static uint32_t get_rst_en_mask(periph_module_t periph, bool enable)
{
    switch(periph) {
        case PERIPH_RMT_MODULE:
            return DPORT_RMT_RST;
        case PERIPH_LEDC_MODULE:
            return DPORT_LEDC_RST;
        case PERIPH_UART0_MODULE:
            return DPORT_UART_RST;
        case PERIPH_UART1_MODULE:
            return DPORT_UART1_RST;
        case PERIPH_UART2_MODULE:
            return DPORT_UART2_RST;
        case PERIPH_I2C0_MODULE:
            return DPORT_I2C_EXT0_RST;
        case PERIPH_I2C1_MODULE:
            return DPORT_I2C_EXT1_RST;
        case PERIPH_I2S0_MODULE:
            return DPORT_I2S0_RST;
        case PERIPH_I2S1_MODULE:
            return DPORT_I2S1_RST;
        case PERIPH_TIMG0_MODULE:
            return DPORT_TIMERGROUP_RST;
        case PERIPH_TIMG1_MODULE:
            return DPORT_TIMERGROUP1_RST;
        case PERIPH_PWM0_MODULE:
            return DPORT_PWM0_RST;
        case PERIPH_PWM1_MODULE:
            return DPORT_PWM1_RST;
        case PERIPH_PWM2_MODULE:
            return DPORT_PWM2_RST;
        case PERIPH_PWM3_MODULE:
            return DPORT_PWM3_RST;
        case PERIPH_UHCI0_MODULE:
            return DPORT_UHCI0_RST;
        case PERIPH_UHCI1_MODULE:
            return DPORT_UHCI1_RST;
        case PERIPH_PCNT_MODULE:
            return DPORT_PCNT_RST;
        case PERIPH_SPI_MODULE:
            return DPORT_SPI01_RST;
        case PERIPH_HSPI_MODULE:
            return DPORT_SPI2_RST;
        case PERIPH_VSPI_MODULE:
            return DPORT_SPI3_RST;
        case PERIPH_SPI_DMA_MODULE:
            return DPORT_SPI_DMA_RST;
        case PERIPH_SDMMC_MODULE:
            return DPORT_SDIO_HOST_RST;
        case PERIPH_SDIO_SLAVE_MODULE:
            return DPORT_SDIO_RST;
        case PERIPH_CAN_MODULE:
            return DPORT_CAN_RST;
        case PERIPH_EMAC_MODULE:
            return DPORT_EMAC_RST;
        case PERIPH_AES_MODULE:
            if (enable == true) {
                // Clear reset on digital signature & secure boot units, otherwise AES unit is held in reset also.
                return (DPORT_PERI_EN_AES | DPORT_PERI_EN_DIGITAL_SIGNATURE | DPORT_PERI_EN_SECUREBOOT);
            } else {
                //Don't return other units to reset, as this pulls reset on RSA & SHA units, respectively.
                return DPORT_PERI_EN_AES;
            }
        case PERIPH_SHA_MODULE:
            if (enable == true) {
                // Clear reset on secure boot, otherwise SHA is held in reset
                return (DPORT_PERI_EN_SHA | DPORT_PERI_EN_SECUREBOOT);
            } else {
                // Don't assert reset on secure boot, otherwise AES is held in reset
                return DPORT_PERI_EN_SHA;
            }
        case PERIPH_RSA_MODULE:
            if (enable == true) {
                // Also clear reset on digital signature, otherwise RSA is held in reset
                return (DPORT_PERI_EN_RSA | DPORT_PERI_EN_DIGITAL_SIGNATURE);
            } else {
                // Don't reset digital signature unit, as this resets AES also
                return DPORT_PERI_EN_RSA;
            }
        case PERIPH_WIFI_MODULE:
        case PERIPH_BT_MODULE:
        case PERIPH_WIFI_BT_COMMON_MODULE:
        case PERIPH_BT_BASEBAND_MODULE:
        case PERIPH_BT_LC_MODULE:
            return 0;
        default:
            return 0;
    }
}

static uint32_t get_rst_en_reg(periph_module_t periph)
{
    if (periph == PERIPH_AES_MODULE || periph == PERIPH_SHA_MODULE || periph == PERIPH_RSA_MODULE) {
        return DPORT_PERI_RST_EN_REG;
    } else {
        return is_wifi_clk_peripheral(periph) ? DPORT_CORE_RST_EN_REG : DPORT_PERIP_RST_EN_REG;
    }
}

void periph_module_enable(periph_module_t periph)
{
    intr_key = irq_lock();
    DPORT_SET_PERI_REG_MASK(get_clk_en_reg(periph), get_clk_en_mask(periph));
    DPORT_CLEAR_PERI_REG_MASK(get_rst_en_reg(periph), get_rst_en_mask(periph, true));
    irq_unlock(intr_key);
}

void periph_module_disable(periph_module_t periph)
{
    intr_key = irq_lock();
    DPORT_CLEAR_PERI_REG_MASK(get_clk_en_reg(periph), get_clk_en_mask(periph));
    DPORT_SET_PERI_REG_MASK(get_rst_en_reg(periph), get_rst_en_mask(periph, false));
    irq_unlock(intr_key);
}

void IRAM_ATTR esp_phy_common_clock_enable(void)
{
    if (s_common_clock_enable_ref == 0) {
        // Enable WiFi/BT common clock
        periph_module_enable(PERIPH_WIFI_BT_COMMON_MODULE);
    }
    s_common_clock_enable_ref++;
}

void IRAM_ATTR esp_phy_common_clock_disable(void)
{
    if (s_common_clock_enable_ref > 0) {
        s_common_clock_enable_ref --;

        if (s_common_clock_enable_ref == 0) {
             // Disable WiFi/BT common clock
             periph_module_disable(PERIPH_WIFI_BT_COMMON_MODULE);
        }
    } else {
        abort();
    }
}

static inline void phy_update_wifi_mac_time(bool en_clock_stopped, int64_t now)
{
    static uint32_t s_common_clock_disable_time = 0;

    if (en_clock_stopped) {
        s_common_clock_disable_time = (uint32_t)now;
    } else {
        if (s_common_clock_disable_time) {
            uint32_t diff = (uint64_t)now - s_common_clock_disable_time;

            // if (s_wifi_mac_time_update_cb) {
                esp_wifi_internal_update_mac_time(diff);
            // }
            s_common_clock_disable_time = 0;
            // ESP_LOGD(TAG, "wifi mac time delta: %u", diff);
        }
    }
}

// PHY init data handling functions
#if CONFIG_ESP32_PHY_INIT_DATA_IN_PARTITION
#include "esp_partition.h"

const esp_phy_init_data_t* esp_phy_get_init_data()
{
    const esp_partition_t* partition = esp_partition_find_first(
            ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_PHY, NULL);
    if (partition == NULL) {
        // ESP_LOGE(TAG, "PHY data partition not found");
        return NULL;
    }
    // ESP_LOGD(TAG, "loading PHY init data from partition at offset 0x%x", partition->address);
    size_t init_data_store_length = sizeof(phy_init_magic_pre) +
            sizeof(esp_phy_init_data_t) + sizeof(phy_init_magic_post);
    uint8_t* init_data_store = (uint8_t*) k_malloc(init_data_store_length);
    if (init_data_store == NULL) {
        // ESP_LOGE(TAG, "failed to allocate memory for PHY init data");
        return NULL;
    }
    esp_err_t err = esp_partition_read(partition, 0, init_data_store, init_data_store_length);
    if (err != ESP_OK) {
        // ESP_LOGE(TAG, "failed to read PHY data partition (0x%x)", err);
        return NULL;
    }
    if (memcmp(init_data_store, PHY_INIT_MAGIC, sizeof(phy_init_magic_pre)) != 0 ||
        memcmp(init_data_store + init_data_store_length - sizeof(phy_init_magic_post),
                PHY_INIT_MAGIC, sizeof(phy_init_magic_post)) != 0) {
        // ESP_LOGE(TAG, "failed to validate PHY data partition");
        return NULL;
    }
    // ESP_LOGD(TAG, "PHY data partition validated");
    return (const esp_phy_init_data_t*) (init_data_store + sizeof(phy_init_magic_pre));
}

void esp_phy_release_init_data(const esp_phy_init_data_t* init_data)
{
    k_free((uint8_t*) init_data - sizeof(phy_init_magic_pre));
}

#else // CONFIG_ESP32_PHY_INIT_DATA_IN_PARTITION

// phy_init_data.h will declare static 'phy_init_data' variable initialized with default init data

const esp_phy_init_data_t* esp_phy_get_init_data()
{
    // ESP_LOGD(TAG, "loading PHY init data from application binary");
    return &phy_init_data;
}

void esp_phy_release_init_data(const esp_phy_init_data_t* init_data)
{
    // no-op
}
#endif // CONFIG_ESP32_PHY_INIT_DATA_IN_PARTITION

esp_err_t esp_phy_rf_init(const esp_phy_init_data_t* init_data, esp_phy_calibration_mode_t mode, 
                          esp_phy_calibration_data_t* calibration_data, phy_rf_module_t module)
{
    /* 3 modules may call phy_init: Wi-Fi, BT, Modem Sleep */
    if (module >= PHY_MODULE_COUNT){
        // ESP_LOGE(TAG, "%s, invalid module parameter(%d), should be smaller than \
                //  module count(%d)", __func__, module, PHY_MODULE_COUNT);
        return ESP_ERR_INVALID_ARG;
    }

    // _lock_acquire(&s_phy_rf_init_lock);
    uint32_t s_module_phy_rf_init_old = s_module_phy_rf_init;
    bool is_wifi_or_bt_enabled = !!(s_module_phy_rf_init_old & (BIT(PHY_BT_MODULE) | BIT(PHY_WIFI_MODULE)));
    esp_err_t status = ESP_OK;
    s_module_phy_rf_init |= BIT(module);

    if ((is_wifi_or_bt_enabled == false) && (module == PHY_MODEM_MODULE)){
        status = ESP_FAIL;
    }
    else if (s_is_phy_rf_en == true) {
    }
    else {
        /* If Wi-Fi, BT all disabled, modem sleep should not take effect;
         * If either Wi-Fi or BT is enabled, should allow modem sleep requires 
         * to enter sleep;
         * If Wi-Fi, BT co-exist, it is disallowed that only one module 
         * support modem sleep, E,g. BT support modem sleep but Wi-Fi not
         * support modem sleep;
         */
        if (is_wifi_or_bt_enabled == false){
            if ((module == PHY_BT_MODULE) || (module == PHY_WIFI_MODULE)){
                s_is_phy_rf_en = true;
            }
        }
        else {
            if (module == PHY_MODEM_MODULE){
                s_is_phy_rf_en = true;
            }
            else if ((module == PHY_BT_MODULE) || (module == PHY_WIFI_MODULE)){
                /* New module (BT or Wi-Fi) can init RF according to modem_sleep_exit */
            }
        }
        if (s_is_phy_rf_en == true){
            // Update time stamp
            s_phy_rf_en_ts = k_ticks_to_us_near64(k_uptime_ticks());
            // Update WiFi MAC time before WiFi/BT common clock is enabled
            phy_update_wifi_mac_time(false, s_phy_rf_en_ts);
            // Enable WiFi/BT common peripheral clock
            //periph_module_enable(PERIPH_WIFI_BT_COMMON_MODULE);
            esp_phy_common_clock_enable();
            phy_set_wifi_mode_only(0);

            if (ESP_CAL_DATA_CHECK_FAIL == register_chipv7_phy(init_data, calibration_data, mode)) {
                // ESP_LOGW(TAG, "saving new calibration data because of checksum failure, mode(%d)", mode);
#ifdef CONFIG_ESP32_PHY_CALIBRATION_AND_DATA_STORAGE
                if (mode != PHY_RF_CAL_FULL) {
                    esp_phy_store_cal_data_to_nvs(calibration_data);
                }
#endif
            }

            // coex_bt_high_prio();
        }
    }

#if CONFIG_SW_COEXIST_ENABLE
    if ((module == PHY_BT_MODULE) || (module == PHY_WIFI_MODULE)){
        uint32_t phy_bt_wifi_mask = BIT(PHY_BT_MODULE) | BIT(PHY_WIFI_MODULE);
        if ((s_module_phy_rf_init & phy_bt_wifi_mask) == phy_bt_wifi_mask) { //both wifi & bt enabled
            coex_init();
            coex_resume();
        }
    }
#endif

    // _lock_release(&s_phy_rf_init_lock);
    return status;
}

esp_err_t esp_phy_rf_deinit(phy_rf_module_t module)
{
    ets_printf("%s\n", __func__);
    return ESP_OK;
}

void esp_phy_load_cal_and_init(phy_rf_module_t module)
{
    esp_phy_calibration_data_t* cal_data =
            (esp_phy_calibration_data_t*) k_calloc(sizeof(esp_phy_calibration_data_t), 1);
    if (cal_data == NULL) {
        // ESP_LOGE(TAG, "failed to allocate memory for RF calibration data");
        abort();
    }

#if CONFIG_REDUCE_PHY_TX_POWER
    const esp_phy_init_data_t* phy_init_data = esp_phy_get_init_data();
    if (phy_init_data == NULL) {
        // ESP_LOGE(TAG, "failed to obtain PHY init data");
        abort();
    }

    esp_phy_init_data_t* init_data = (esp_phy_init_data_t*) k_malloc(sizeof(esp_phy_init_data_t));
    if (init_data == NULL) {
        // ESP_LOGE(TAG, "failed to allocate memory for phy init data");
        abort();
    }

    memcpy(init_data, phy_init_data, sizeof(esp_phy_init_data_t));
    if (esp_reset_reason() == ESP_RST_BROWNOUT) {
        esp_phy_reduce_tx_power(init_data);
    }
#else
    const esp_phy_init_data_t* init_data = esp_phy_get_init_data();
    if (init_data == NULL) {
        // ESP_LOGE(TAG, "failed to obtain PHY init data");
        abort();
    }
#endif

#ifdef CONFIG_ESP32_PHY_CALIBRATION_AND_DATA_STORAGE
    esp_phy_calibration_mode_t calibration_mode = PHY_RF_CAL_PARTIAL;
    uint8_t sta_mac[6];
    if (rtc_get_reset_reason(0) == DEEPSLEEP_RESET) {
        calibration_mode = PHY_RF_CAL_NONE;
    }
    esp_err_t err = esp_phy_load_cal_data_from_nvs(cal_data);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "failed to load RF calibration data (0x%x), falling back to full calibration", err);
        calibration_mode = PHY_RF_CAL_FULL;
    }

    esp_efuse_mac_get_default(sta_mac);
    memcpy(cal_data->mac, sta_mac, 6);
    esp_phy_rf_init(init_data, calibration_mode, cal_data, module);

    if (calibration_mode != PHY_RF_CAL_NONE && err != ESP_OK) {
        err = esp_phy_store_cal_data_to_nvs(cal_data);
    } else {
        err = ESP_OK;
    }
#else
    esp_phy_rf_init(init_data, PHY_RF_CAL_FULL, cal_data, module);
#endif

#if CONFIG_REDUCE_PHY_TX_POWER
    esp_phy_release_init_data(phy_init_data);
    k_free(init_data);
#else
    esp_phy_release_init_data(init_data);
#endif

    k_free(cal_data); // PHY maintains a copy of calibration data, so we can free this
}


esp_err_t esp_modem_sleep_register(modem_sleep_module_t module)
{
    if (module >= MODEM_MODULE_COUNT){
        // ESP_LOGE(TAG, "%s, invalid module parameter(%d), should be smaller than \
        //          module count(%d)", __func__, module, MODEM_MODULE_COUNT);
        return ESP_ERR_INVALID_ARG;
    }
    else if (s_modem_sleep_module_register & BIT(module)){
        // ESP_LOGI(TAG, "%s, multiple registration of module (%d)", __func__, module);
        return ESP_OK;
    }
    else{
        k_spinlock_key_t key = k_spin_lock(&s_modem_sleep_lock);
        s_modem_sleep_module_register |= BIT(module);
        /* The module is set to enter modem sleep by default, otherwise will prevent
         * other modules from entering sleep mode if this module never call enter sleep function
         * in the future */
        s_modem_sleep_module_enter |= BIT(module);
        k_spin_unlock(&s_modem_sleep_lock, key);
        return ESP_OK;
    }
}

esp_err_t esp_modem_sleep_exit(modem_sleep_module_t module)
{
#if CONFIG_SW_COEXIST_ENABLE
    uint32_t phy_bt_wifi_mask = BIT(PHY_BT_MODULE) | BIT(PHY_WIFI_MODULE);
#endif

    if (module >= MODEM_MODULE_COUNT){
        // ESP_LOGE(TAG, "%s, invalid module parameter(%d), should be smaller than \
        //          module count(%d)", __func__, module, MODEM_MODULE_COUNT);
        return ESP_ERR_INVALID_ARG;
    }
    else if (!(s_modem_sleep_module_register & BIT(module))){
        // ESP_LOGW(TAG, "%s, module (%d) has not been registered", __func__, module);
        return ESP_ERR_INVALID_ARG;
    }
    else {
        k_spinlock_key_t key = k_spin_lock(&s_modem_sleep_lock);
        s_modem_sleep_module_enter &= ~BIT(module);
        if (s_is_modem_sleep_en){
            esp_err_t status = esp_phy_rf_init(NULL,PHY_RF_CAL_NONE,NULL, PHY_MODEM_MODULE);
            if (status == ESP_OK){
                s_is_modem_sleep_en = false;
            }
        }
#if CONFIG_SW_COEXIST_ENABLE
        _lock_acquire(&s_phy_rf_init_lock);
        if (((s_module_phy_rf_init & phy_bt_wifi_mask) == phy_bt_wifi_mask)  //both wifi & bt enabled
                && (s_modem_sleep_module_enter & (MODEM_BT_MASK | MODEM_WIFI_MASK)) == 0){
            coex_resume();
        }
        _lock_release(&s_phy_rf_init_lock);
#endif
        k_spin_unlock(&s_modem_sleep_lock, key);
        return ESP_OK;
    }
    return ESP_OK;
}

esp_err_t esp_modem_sleep_enter(modem_sleep_module_t module)
{
    return ESP_OK;
}
