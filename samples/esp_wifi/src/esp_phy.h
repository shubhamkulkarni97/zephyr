#include "soc/periph_defs.h"
#include "soc/dport_reg.h"

void IRAM_ATTR esp_phy_common_clock_enable(void);
void IRAM_ATTR esp_phy_common_clock_disable(void);
void periph_module_enable(periph_module_t periph);
void periph_module_disable(periph_module_t periph);
