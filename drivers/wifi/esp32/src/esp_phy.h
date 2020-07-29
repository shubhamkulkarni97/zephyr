#include "soc/periph_defs.h"
#include "soc/dport_reg.h"
#include "phy_init_data.h"

void IRAM_ATTR esp_phy_common_clock_enable(void);
void IRAM_ATTR esp_phy_common_clock_disable(void);
void periph_module_enable(periph_module_t periph);
void periph_module_disable(periph_module_t periph);
void esp_phy_load_cal_and_init(phy_rf_module_t module);
esp_err_t esp_phy_rf_deinit(phy_rf_module_t module);
