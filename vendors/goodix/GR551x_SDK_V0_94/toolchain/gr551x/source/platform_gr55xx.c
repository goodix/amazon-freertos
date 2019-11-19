#include "gr55xx_sys.h"
#include "hal_flash.h"
#include "custom_config.h"
#include "platform_sdk.h"
#include "patch.h"

io_table_t io_table =
{
   .gpio = 0,
   .aon_gpio = 0,
   .msio = 0,
};

void platform_init(void)
{
    hal_flash_init(); 
    nvds_init(NVDS_START_ADDR, NVDS_NUM_SECTOR);
		
    system_io_leakage_protect(&io_table);
    set_patch_flag(MANDATORY_PATCH);  
    platform_sdk_init();
    return;
}

extern void system_platform_init(void);
#if defined ( __GNUC__ )
void __main(void)
{
    __asm("ldr    r1, =__etext\n");
    __asm("ldr    r2, =__data_start__\n");
    __asm("ldr    r3, =__data_end__\n");
    __asm(".L_loop1:\n");
    __asm("cmp    r2, r3\n");
    __asm("ittt   lt\n");
    __asm("ldrlt  r0, [r1], #4\n");
    __asm("strlt  r0, [r2], #4\n");
    __asm("blt    .L_loop1\n");
    __asm("ldr    r1, =__bss_start__\n");
    __asm("ldr    r2, =__bss_end__\n");
    __asm("movs   r0, 0\n");
    __asm(".L_loop3:\n");
    __asm("cmp    r1, r2\n");
    __asm("itt    lt\n");
    __asm("strlt  r0, [r1], #4\n");
    __asm("blt    .L_loop3\n");
    system_platform_init();
    main();    
}
#endif

#if defined ( __CC_ARM )
extern void $Super$$main(void); 
void $Sub$$main(void)
{
    system_platform_init();
    $Super$$main();
}
#endif


