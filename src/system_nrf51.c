/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */  
  /**
 *
 * @file
 * Startup instructions executed upon reset
 */ 
/* NOTE: Template files (including this one) are application specific and therefore expected to 
   be copied into the application project folder prior to its use! */ 
  
#include <stdint.h>
#include <stdbool.h>
#include "nrf.h"
#include "system_nrf51.h"
  
/*lint ++flb "Enter library region" */ 
   
#define __SYSTEM_CLOCK      (16000000UL)     /*!< nRF51 devices use a fixed System Clock Frequency of 16MHz */
static bool is_manual_peripheral_setup_needed(void);
static bool is_disabled_in_debug_needed(void);

  
#if defined ( __CC_ARM )
  uint32_t SystemCoreClock __attribute__ ((used)) = __SYSTEM_CLOCK;

#elif defined ( __ICCARM__ )
  __root uint32_t SystemCoreClock = __SYSTEM_CLOCK;

#elif defined   ( __GNUC__ )
  uint32_t SystemCoreClock __attribute__ ((used)) = __SYSTEM_CLOCK;

#endif  /*  */
  void
SystemCoreClockUpdate(void) 
{
  SystemCoreClock = __SYSTEM_CLOCK;
}  void
SystemInit(void) 
{
  
    /* If desired, switch off the unused RAM to lower consumption by the use of RAMON register.
       It can also be done in the application main() function. */ 
    
    /* Prepare the peripherals for use as indicated by the PAN 26 "System: Manual setup is required
       to enable the use of peripherals" found at Product Anomaly document for your device found at
       https://www.nordicsemi.com/. The side effect of executing these instructions in the devices 
       that do not need it is that the new peripherals in the second generation devices (LPCOMP for
       example) will not be available. */ 
    if(is_manual_peripheral_setup_needed())
     {
    *(uint32_t volatile *)0x40000504 = 0xC007FFDF;
    *(uint32_t volatile *)0x40006C18 = 0x00008000;
    }
   
    /* Disable PROTENSET registers under debug, as indicated by PAN 59 "MPU: Reset value of DISABLEINDEBUG
       register is incorrect" found at Product Anomaly document four your device found at 
       https://www.nordicsemi.com/. There is no side effect of using these instruction if not needed. */ 
    if(is_disabled_in_debug_needed())
     {
    NRF_MPU->DISABLEINDEBUG =
      MPU_DISABLEINDEBUG_DISABLEINDEBUG_Disabled <<
      MPU_DISABLEINDEBUG_DISABLEINDEBUG_Pos;
    }
}
  static bool
is_manual_peripheral_setup_needed(void) 
{
  if((((*(uint32_t *) 0xF0000FE0) & 0x000000FF) == 0x1)
      && (((*(uint32_t *) 0xF0000FE4) & 0x0000000F) == 0x0))
     {
    if((((*(uint32_t *) 0xF0000FE8) & 0x000000F0) == 0x00)
        && (((*(uint32_t *) 0xF0000FEC) & 0x000000F0) == 0x0))
       {
      return true;
      }
    if((((*(uint32_t *) 0xF0000FE8) & 0x000000F0) == 0x10)
        && (((*(uint32_t *) 0xF0000FEC) & 0x000000F0) == 0x0))
       {
      return true;
      }
    if((((*(uint32_t *) 0xF0000FE8) & 0x000000F0) == 0x30)
        && (((*(uint32_t *) 0xF0000FEC) & 0x000000F0) == 0x0))
       {
      return true;
      }
    }
   return false;
}
 static bool
is_disabled_in_debug_needed(void) 
{
  if((((*(uint32_t *) 0xF0000FE0) & 0x000000FF) == 0x1)
      && (((*(uint32_t *) 0xF0000FE4) & 0x0000000F) == 0x0))
     {
    if((((*(uint32_t *) 0xF0000FE8) & 0x000000F0) == 0x40)
        && (((*(uint32_t *) 0xF0000FEC) & 0x000000F0) == 0x0))
       {
      return true;
      }
    }
   return false;
}
 
/*lint --flb "Leave library region" */ 
