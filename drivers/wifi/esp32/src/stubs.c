#include "stubs.h"


// needs to go into ram due to ISR context
uint32_t phy_enter_critical(void)
{
    //  FreeRTOS function
    //  return portENTER_CRITICAL_NESTED();
    return 0;
}

// needs to go into ram due to ISR context
void phy_exit_critical(uint32_t level)
{
    //  FreeRTOS function
    //portEXIT_CRITICAL_NESTED(level);
    return;
}

void abort(void) {

}
