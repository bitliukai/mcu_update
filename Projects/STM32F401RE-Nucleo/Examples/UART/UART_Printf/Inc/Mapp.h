
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USR_H
#define __USR_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"
#include "stdio.h"
#include "main.h"



/**
 *    LOG switch 
 *
*/

#define  MAPP_LOG(info)    info
 
 
/**
 *
 *    function config enable/disable
 *
*/
#ifndef   CONFIG_LED
#define   CONFIG_LED    1U
#endif 

#define  T    10.0
#define  Ti   20.0
#define  Kp   1.0
#define  Td   0
#define  DA_STAND  1241  //1000mV
#define  PID_MAX   2048
#define  PID_MIN    0

typedef  struct 
{
    UINT16 dealt_now;
    UINT16 dealt_last;
    UINT16 dealt_last_last;
    UINT16 dealt_o;
}PID_T;


typedef enum
{
    RET_OK,
    RET_ERROE
} RET_TYPE;


#ifndef  FLASH_ERASE_SIZE
#define  FLASH_ERASE_SIZE   1024
#endif 

#ifndef   FLASH_SIZE
#define   FLASH_SIZE         uint8_t
#endif  

/**
*
*    报文解析过程定义
*
*/
typedef enum
{
    STATE_CHECK_HEAD,
    STATE_RCV_BIN,
    STATE_CHECK_LENGTH,
    STATE_CHECK_CRC,
    STATE_DONE
}PACKET_PARSE_STATE;


typedef enum
{
    STATE_TOP_INIT,
    STATE_TOP_UPDATE,
    STATE_TOP_RESET,
    STATE_TOP_LOOP
}TOP_STATE;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
