#include <stdint.h>
#include <string.h>

uint32_t phy_enter_critical(void);
void phy_exit_critical(uint32_t level);
void abort(void);
