#include <zephyr.h>
#include "esp_phy.h"
#include "rom/ets_sys.h"

static volatile int32_t s_common_clock_enable_ref = 0;
static unsigned int intr_key;

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