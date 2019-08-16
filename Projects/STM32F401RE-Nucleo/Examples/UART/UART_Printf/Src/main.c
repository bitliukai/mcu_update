/**
*
*  @Description:   
                   stm32 在线升级demo , 使用片内sram重定向实现\
                   适用于.bin size<96KB 的程序  \
                   使用Python通过串口将镜像发送至下位机 ， 以后根据使用场景可以扩展为USB , Ethnet , CAN \
                   下位机解析报文并擦写flash，实现升级
                   
*  @Author     :   刘 凯
*  @Date       :   2019-08-16
*  @Company    :   ZTE.Inc (Copyright) 
*
*/

#include "main.h"
#include "Mapp.h"
/**
*
*    Function Prototype
*/
static void SystemClock_Config(void);
static void Error_Handler(void);



/**
*
*    Global Variables Definition
*/
UART_HandleTypeDef UartHandle;
GPIO_InitTypeDef   GPIO_InitStructure;
UINT8  g_exti13_flag;
UINT8  g_exti0_flag;
UINT16 g_exti13_cnt;
UINT16 g_exti0_cnt;
UINT16 g_current_time;


/**
*
*    store the raw bin data parsr from packet
*/
FLASH_SIZE   code_bin[FLASH_ERASE_SIZE]={0};

/**
*
*     main state machine 
*/
TOP_STATE  state_of_top=STATE_TOP_INIT;

/**
*
*      main routine entry
*/
int  main(void)
{
 
    MAPP_LOG(printf("Entering main...\n\n"););
    
    #if CONFIG_PID
      uint16_t  old_time;
      PID_T pid;
    #endif 
    
  switch (state_of_top)
  {
      case STATE_TOP_INIT:
          MAPP_LOG(printf("Top state:STATE_TOP_INIT"));
                  HAL_Init();
                  /* Configure the system clock to 84 MHz */
                  SystemClock_Config();
                  
                  #if CONFIG_UART
                  UART_Init();
                  #endif 
                    
                  #if CONFIG_LED
                  BSP_LED_Init(LED2);
                  #endif 
                    
                  #if CONFIG_PID
                  /*起一个1ms的定时器*/
                  if(TIM2_Init(10) != STATUS_OK)  
                  {
                     printf("Time2 init Failed...\r\n"); 
                  }
                  
                  old_time = 0;
                  #endif 
                  
                  #if CONFIG_EXTI_INTERRUPT
                   if(BSP_EXTI13_Init(TOGGLE_FALLING) != STATUS_OK)
                  {
                      printf("\r\nEXTI13 init Failed...");
                  }
                  if(BSP_EXTI0_Init(TOGGLE_FALLING) != STATUS_OK)
                  {
                      printf("\r\nEXTI0init Failed...");
                  }
                  #endif 
                  
                  state_of_top = STATE_TOP_LOOP;
          break;
      case STATE_TOP_UPDATE:
          
      
                      MAPP_LOG(printf("Top state:STATE_TOP_UPDATE"));
                   
                    if(1) //
                          state_of_top=STATE_TOP_RESET;
          break;
      case STATE_TOP_RESET:
          
                     MAPP_LOG(printf("Top state:STATE_TOP_RESET"));

          break;
      case STATE_TOP_LOOP:
          
                    MAPP_LOG(printf("Top state:STATE_TOP_LOOP"));

             #if CONFIG_PID
                     //10ms执行一次PID程序
                     if(g_current_time-old_time >= 10)   
                     {
                         old_time = g_current_time;
                         BSP_LED_Toggle(LED2);
                         PID_Execute();
                     }
             #endif 
             
               #if CONFIG_LED
                     BSP_LED_Toggle(LED2);
               #endif 
          break;
      default:
          break;
  } /*end of switch */
}  /*end of main*/


#if CONFIG_PID

UINT8 PID_Execute(void)
{
    #if CONFIG_PID
    PID_T pid;
    UINT16 ad_val;
    UINT16 da_val;
    #endif 
    
    //STM32_READ_AD(&ad_val);
    
    pid.dealt_last_last = pid.dealt_last;
    pid.dealt_last = pid.dealt_now;
    pid.dealt_now = ad_val-DA_STAND;
    pid.dealt_o += (UINT16)(Kp*(pid.dealt_now-pid.dealt_last)+Kp*T/Ti*pid.dealt_now+Kp*Td/T*(pid.dealt_now-2*pid.dealt_last+pid.dealt_last_last));
    
    if(pid.dealt_o > PID_MAX)
        pid.dealt_o = PID_MAX;
    if(pid.dealt_o < PID_MIN)
        pid.dealt_o = PID_MIN;
    
    da_val = pid.dealt_o;
    
    //STM32_WRITE_DA(pid.dealt_o);
    
}
#endif

/*--------------------------------------------------*/

status TIM2_Init(uint32_t Period)
{
    if(Period > 0xFFFF)
        return STATUS_ERROR;
    RCC  -> APB1ENR |= (0x00000001<<0); //Enable the TIM2 clock
    TIM2 -> CNT      = 0L; 
    TIM2 -> PSC      = 8399;
    TIM2 -> ARR      = Period;
    TIM2 -> CR1     |= 0x01<<0;       //Enable TIM2 
    TIM2 -> DIER    |= 0x00001;       //Enable TIM2 interrupt 
    
    HAL_NVIC_SetPriority(EXTI0_IRQn , 2 ,3);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    return STATUS_OK;
}

status BSP_EXTI13_Init(UINT8 EdgeFlag)
{
    //Parameter check 
    if(EdgeFlag!=TOGGLE_RISING && EdgeFlag!=TOGGLE_FALLING && EdgeFlag!=TOGGLE_RISING_FALLING)
        return STATUS_ERROR;
    //Enable the clock of PC.13
    RCC ->AHB1ENR  |= 0x0001<<2;
    RCC ->APB2ENR  |= 0x0001<<14;
    
    //Config The PC.13 as input 
    GPIOC ->MODER  &= 0xF3FFFFFF;
    
    //Slect PC.13 as the source of EXTI line 13
    SYSCFG ->EXTICR[3] |= (0x2<<4);
    
    if(EdgeFlag == 0x1)
        EXTI ->RTSR   |= 0x0001 <<  13;
    else if(EdgeFlag == 0x2) 
        EXTI ->FTSR   |= 0x0001 <<  13;
    else if(EdgeFlag == 0x3)
    {
        EXTI ->RTSR   |= 0x0001 <<  13;
        EXTI ->FTSR   |= 0x0001 <<  13;        
    }
        
     //Enable the EXTI13
    EXTI ->IMR  |= 0x0001 <<  13;
     
    //Enable the interrup 
    HAL_NVIC_SetPriority(EXTI15_10_IRQn , 2 ,0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    
    return STATUS_OK;
}

//Set PA.0 as exti0 interrup line 
status BSP_EXTI0_Init(UINT8 EdgeFlag)
{
    //Parameter check 
    if(EdgeFlag!=TOGGLE_RISING && EdgeFlag!=TOGGLE_FALLING && EdgeFlag!=TOGGLE_RISING_FALLING)
        return STATUS_ERROR;
    //Enable the clock of PA.0
    RCC ->AHB1ENR  |= 0x0001<<0;
    
    //Enable the system config clock
    RCC ->APB2ENR  |= 0x0001<<14;
    
    //Config The PA.0 as input 
    GPIOA ->MODER  &= 0xFFFFFFFC;
    
    //Slect PA.0 as the source of EXTI line 0
    SYSCFG ->EXTICR[0] |= (0x0<<0);
    
    if(EdgeFlag == 0x1)
        EXTI ->RTSR   |= 0x0001 <<  0;
    else if(EdgeFlag == 0x2) 
        EXTI ->FTSR   |= 0x0001 <<  0;
    else if(EdgeFlag == 0x3)
    {
        EXTI ->RTSR   |= 0x0001 <<  0;
        EXTI ->FTSR   |= 0x0001 <<  0;        
    }
    
    //Enable the EXTI13
    EXTI ->IMR  |= 0x0001 <<  0;
     
    //Enable the interrup 
    HAL_NVIC_SetPriority(EXTI0_IRQn , 2 ,1);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    
    return STATUS_OK;
    
}


void UART_Init(void)
{
      /*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART1 configured as follow:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = ODD parity
      - BaudRate = 9600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  
      UartHandle.Instance          = USARTx;
      
      UartHandle.Init.BaudRate     = 9600;
      UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
      UartHandle.Init.StopBits     = UART_STOPBITS_1;
      UartHandle.Init.Parity       = UART_PARITY_NONE;
      UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
      UartHandle.Init.Mode         = UART_MODE_TX_RX;
      UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
      
      if(HAL_UART_Init(&UartHandle) != HAL_OK)
      {
        /* Initialization Error */
        Error_Handler(); 
      }
  
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */

/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
  
  
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 0xFFFF); 

  return ch;
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 84000000
  *            HCLK(Hz)                       = 84000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSI Frequency(Hz)              = 16000000
  *            PLL_M                          = 16
  *            PLL_N                          = 336
  *            PLL_P                          = 4
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale2 mode
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  
  /* Enable HSI Oscillator and activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
 
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

void BSP_LED_Init(Led_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /* Enable the GPIO_LED Clock */
  LEDx_GPIO_CLK_ENABLE(Led);
  
  /* Configure the GPIO_LED pin */
  GPIO_InitStruct.Pin = LD2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  
  HAL_GPIO_Init(LD2_PORT, &GPIO_InitStruct);
  
  HAL_GPIO_WritePin(LD2_PORT, LD2_PIN, GPIO_PIN_RESET); 
}


/**
  * @brief  DeInit LEDs.
  * @param  Led: LED to be de-init. 
  *   This parameter can be one of the following values:
  *     @arg  LED2
  * @note Led DeInit does not disable the GPIO clock nor disable the Mfx 
  */
void BSP_LED_DeInit(Led_TypeDef Led)
{
  GPIO_InitTypeDef  gpio_init_structure;

  /* Turn off LED */
  HAL_GPIO_WritePin(LD2_PORT, LD2_PIN, GPIO_PIN_RESET);
  /* DeInit the GPIO_LED pin */
  gpio_init_structure.Pin = LD2_PIN;
  HAL_GPIO_DeInit(LD2_PORT, gpio_init_structure.Pin);
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on. 
  *   This parameter can be one of following parameters:
  *     @arg LED2
  */
void BSP_LED_On(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(LD2_PORT, LD2_PIN, GPIO_PIN_SET); 
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off. 
  *   This parameter can be one of following parameters:
  *     @arg LED2
  */
void BSP_LED_Off(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(LD2_PORT, LD2_PIN, GPIO_PIN_RESET); 
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled. 
  *   This parameter can be one of following parameters:
  *     @arg LED2  
  */
void BSP_LED_Toggle(Led_TypeDef Led)
{
  HAL_GPIO_TogglePin(LD2_PORT, LD2_PIN);
}



/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED2 on */
  BSP_LED_On(LED2);
  while(1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
