#include <stdint.h>

/*----------------------------------------------------------------*/

typedef void (*security_present_t)(uint32_t addr, uint8_t *input, uint32_t size, uint8_t *output);

security_present_t security_present_func = (security_present_t)(0x00001200+1);

void sys_security_data_use_present(uint32_t addr, uint8_t *input, uint32_t size, uint8_t *output)
{
#if defined(CFG_SECURT_BOOT)
    security_present_func(addr, input, size, output);
#endif
    return;
}
#pragma arm section code
