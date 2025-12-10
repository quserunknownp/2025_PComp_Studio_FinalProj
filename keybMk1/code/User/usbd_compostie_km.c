/********************************** (C) COPYRIGHT *******************************
 * File Name          : usbd_composite_km.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2022/08/18
 * Description        : USB keyboard and mouse processing.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/


/*******************************************************************************/
/* Header Files */
#include "ch32v30x_usbhs_device.h"
#include "usbd_composite_km.h"

#include <math.h>
#include <string.h>

/*******************************************************************************/
/* Variable Definition */

/* Mouse */
volatile uint8_t  MS_Scan_Done = 0x00;                                          // Mouse Movement Scan Done
volatile uint16_t MS_Scan_Result = 0x000F;                                      // Mouse Movement Scan Result
uint8_t  MS_Data_Pack[ 4 ] = { 0x00 };                                          // Mouse IN Data Packet

/* Keyboard */
volatile uint8_t  KB_Scan_Done = 0x00;                                          // Keyboard Keys Scan Done
volatile uint16_t KB_Scan_Result = 0xF000;                                      // Keyboard Keys Current Scan Result
volatile uint16_t KB_Scan_Last_Result = 0xF000;                                 // Keyboard Keys Last Scan Result
uint8_t  KB_Data_Pack[ 8 ] = { 0x00 };                                          // Keyboard IN Data Packet
volatile uint8_t  KB_LED_Last_Status = 0x00;                                    // Keyboard LED Last Result
volatile uint8_t  KB_LED_Cur_Status = 0x00;                                     // Keyboard LED Current Result

/* ADC */
char Data_Buffer[64];
#define NUM_CHANNELS    7
volatile u16 ADC_Values[NUM_CHANNELS]; 
s16 Calibrattion_Val = 0;
volatile uint8_t  ADC_Scan_Done = 0x00;                                          // Keyboard Keys Scan Done
volatile uint16_t ADC_Scan_Result = 0xF800;                                      // Keyboard Keys Current Scan Result
volatile uint16_t ADC_Scan_Last_Result = 0xF800;                                 // Keyboard Keys Last Scan Result
uint8_t  ADC_Data_Pack[ 8 ] = { 0x00 };                                          // Keyboard IN Data Packet

/* ADC Mouse (Joystick) */
volatile uint8_t  ADCMS_Scan_Done = 0x00;                                         // ADC Mouse Scan Done Flag
uint8_t  ADCMS_Data_Pack[ 4 ] = { 0x00 };                                         // ADC Mouse IN Data Packet
volatile uint8_t  Current_Button_State = 0x00;                                    // Shared Button State (Bitmap)

#define JS_IDX_X            5       // ADC 배열 인덱스 (X축)
#define JS_IDX_Y            6       // ADC 배열 인덱스 (Y축)

#define JS_CENTER_X         2047    // 측정된 X축 중앙값
#define JS_CENTER_Y         2067    // 측정된 Y축 중앙값

#define JS_JUMP_THRESHOLD   50     // 급발진 방지 데드존 (점프 보정)
#define JS_MAX_RADIUS       250.0f  // 미동 나사 최대 반경 (픽셀)
#define JS_FULL_SCALE       2048.0f // ADC 전체 스케일 (0~4095의 절반)

/* USART */
volatile uint8_t  USART_Recv_Dat = 0x00;
volatile uint8_t  USART_Send_Flag = 0x00;
volatile uint8_t  USART_Send_Cnt = 0x00;



/*******************************************************************************/
/* Interrupt Function Declaration */
void TIM3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      TIM3_Init
 *
 * @brief   Initialize timer3 for keyboard and mouse scan.
 *
 * @param   arr - The specific period value
 *          psc - The specifies prescaler value
 *
 * @return  none
 */
void TIM3_Init( uint16_t arr, uint16_t psc )
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable Timer3 Clock */
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM3, ENABLE );

    /* Initialize Timer3 */
    TIM_TimeBaseStructure.TIM_Period = arr;
    TIM_TimeBaseStructure.TIM_Prescaler = psc;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit( TIM3, &TIM_TimeBaseStructure );

    TIM_ITConfig( TIM3, TIM_IT_Update, ENABLE );

    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init( &NVIC_InitStructure );

    /* Enable Timer3 */
    TIM_Cmd( TIM3, ENABLE );
}

/*********************************************************************
 * @fn      TIM3_IRQHandler
 *
 * @brief   This function handles TIM3 global interrupt request.
 *
 * @return  none
 */
void TIM3_IRQHandler( void )
{
    if( TIM_GetITStatus( TIM3, TIM_IT_Update ) != RESET )
    {
        /* Clear interrupt flag */
        TIM_ClearITPendingBit( TIM3, TIM_IT_Update );

        /* Handle keyboard scan */
        KB_Scan( );

        /* Handle mouse scan */
        MS_Scan( );

        /* ADC scan*/   
        ADCKB_Scan( );

        ADCMS_Scan();

        /* Start timing for uploading the key value received from 3 */
        if( USART_Send_Flag )
        {
            USART_Send_Cnt++;
        }
    }
}

/*********************************************************************
 * @fn      USART3_Init
 *
 * @brief   Initialize UART2 to receive keyboard data sent through the
 *          PC serial software.
 *
 * @param   baudrate - Serial baud rate
 *
 * @return  none
 */
void USART3_Init( uint32_t baudrate )
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART3, ENABLE );
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init( GPIOB, &GPIO_InitStructure );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init( GPIOB, &GPIO_InitStructure );

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn; 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init( &NVIC_InitStructure );

    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init( USART3, &USART_InitStructure );
    USART_ITConfig( USART3, USART_IT_RXNE, ENABLE );
    USART_Cmd( USART3, ENABLE );
}

/*********************************************************************
 * @fn      USART3_IRQHandler
 *
 * @brief   This function handles USART3 global interrupt request.
 *
 * @return  none
 */
void USART3_IRQHandler( void )
{
    if( USART_GetITStatus( USART3, USART_IT_RXNE) != RESET )
    {
        /* Save the key value received from USART3 */
        USART_Recv_Dat = USART_ReceiveData( USART3 ) & 0xFF;
    }
}

/*********************************************************************
 * @fn      USART3_Receive_Handle
 *
 * @brief   This function handles the key value received from USART3.
 *
 * @return  none
 */
void USART3_Receive_Handle( void )
{
    uint8_t status;
    static uint8_t flag = 0x00;

    if( flag == 0 )
    {
        /* Store the received specified key value into the keyboard data buffer */
        if( ( USART_Recv_Dat == DEF_KEY_CHAR_A ) ||
            ( USART_Recv_Dat == DEF_KEY_CHAR_W ) ||
            ( USART_Recv_Dat == DEF_KEY_CHAR_S ) ||
            ( USART_Recv_Dat == DEF_KEY_CHAR_D ) ||
            ( USART_Recv_Dat == DEF_KEY_CHAR_F ) )
        {
            memset( KB_Data_Pack, 0x00, sizeof( KB_Data_Pack ) );
            KB_Data_Pack[ 2 ] = USART_Recv_Dat;
            flag = 1;
        }
    }
    else if( flag == 1 )
    {
        /* Load keyboard data to endpoint 1 */
        status = USBHS_Endp_DataUp( DEF_UEP1, KB_Data_Pack, sizeof( KB_Data_Pack ), DEF_UEP_CPY_LOAD );

        if( status == READY )
        {
            /* Enable timing for uploading the key value */
            USART_Send_Cnt = 0;
            USART_Send_Flag = 1;
            flag = 2;
        }
    }
    else if( flag == 2 )
    {
        /* Delay 10ms to ensure that the key value is successfully uploaded,
         * and prepare the data packet indicating the key release.
         */
        if( USART_Send_Cnt >= 50 )
        {
            USART_Send_Flag = 0;
            memset( KB_Data_Pack, 0x00, sizeof( KB_Data_Pack ) );
            flag = 3;
        }
    }
    else if( flag == 3 )
    {
        /* Load keyboard data to endpoint 1 */
        status = USBHS_Endp_DataUp( DEF_UEP1, KB_Data_Pack, sizeof( KB_Data_Pack ), DEF_UEP_CPY_LOAD );

        /* Clear variables for next reception */
        if( status == READY )
        {
            USART_Recv_Dat = 0;
            flag = 0;
        }
    }
}

/*********************************************************************
 * @fn      KB_Scan_Init
 *
 * @brief   Initialize IO for keyboard scan.
 *
 * @return  none
 */
void KB_Scan_Init( void )
{
    GPIO_InitTypeDef GPIO_InitStructure = { 0 };

    /* Enable GPIOB clock */
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE );

    /* Initialize GPIOB for the keyboard scan */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOB, &GPIO_InitStructure );
}

/*********************************************************************
 * @fn      KB_Sleep_Wakeup_Cfg
 *
 * @brief   Configure keyboard wake up mode.
 *
 * @return  none
 */
void KB_Sleep_Wakeup_Cfg( void )
{
    EXTI_InitTypeDef EXTI_InitStructure = { 0 };

    /* Enable GPIOB clock */
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO, ENABLE );

    GPIO_EXTILineConfig( GPIO_PortSourceGPIOB, GPIO_PinSource12 );
    EXTI_InitStructure.EXTI_Line = EXTI_Line12;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Event;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init( &EXTI_InitStructure );

    GPIO_EXTILineConfig( GPIO_PortSourceGPIOB, GPIO_PinSource13 );
    EXTI_InitStructure.EXTI_Line = EXTI_Line13;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Event;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init( &EXTI_InitStructure );

    GPIO_EXTILineConfig( GPIO_PortSourceGPIOB, GPIO_PinSource14 );
    EXTI_InitStructure.EXTI_Line = EXTI_Line14;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Event;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init( &EXTI_InitStructure );

    GPIO_EXTILineConfig( GPIO_PortSourceGPIOB, GPIO_PinSource15 );
    EXTI_InitStructure.EXTI_Line = EXTI_Line15;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Event;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init( &EXTI_InitStructure );

    EXTI->INTENR |= EXTI_INTENR_MR12 | EXTI_INTENR_MR13 | EXTI_INTENR_MR14 | EXTI_INTENR_MR15;
}

/*********************************************************************
 * @fn      KB_Scan
 *
 * @brief   Perform keyboard scan.
 *
 * @return  none
 */
void KB_Scan( void )
{

    static uint16_t scan_cnt = 0;
    static uint16_t scan_result = 0;

    scan_cnt++;
    if( ( scan_cnt % 10 ) == 0 )
    {
        scan_cnt = 0;

        /* Determine whether the two scan results are consistent */
        if( scan_result == ( GPIO_ReadInputData( GPIOB ) & 0xF000 ) )
        //여기에 scan을 GPIO_READInputData 말고 ADC 데이터로 바꾼 후, if문 추가로 자석축 사용.
        {
            KB_Scan_Done = 1;
            KB_Scan_Result = scan_result;
        }
    }
    else if( ( scan_cnt % 5 ) == 0 )
    {
        /* Save the first scan result */
        scan_result = ( GPIO_ReadInputData( GPIOB ) & 0xF000 );
    }
}


/*********************************************************************
 * @fn      KB_Scan_Handle
 *
 * @brief   Handle keyboard scan data.
 *
 * @return  none
 */
void KB_Scan_Handle( void ){
    uint8_t i, j;
    uint8_t status;
    static uint8_t key_cnt = 0x00;
    static uint8_t flag = 0x00;

    if( KB_Scan_Done ){
        KB_Scan_Done = 0;

        if( KB_Scan_Result != KB_Scan_Last_Result ){
            for( i = 12; i < 16; i++ ){
                /* Determine that there is at least one key is pressed or released */
                if( ( KB_Scan_Result & ( 1 << i ) ) != ( KB_Scan_Last_Result & ( 1 << i ) ) ){
                    if( ( KB_Scan_Result & ( 1 << i ) ) ) {          // Key press
                        if( i == 12 ){
                            for( j = 2; j < 8; j++ )
                            {
                                if( KB_Data_Pack[ j ] == DEF_KEY_CHAR_W )
                                {
                                    break;
                                }
                            }
                        }
                        else if( i == 13 )
                        {
                            for( j = 2; j < 8; j++ )
                            {
                                if( KB_Data_Pack[ j ] == DEF_KEY_CHAR_A )
                                {
                                    break;
                                }
                            }
                        }
                        else if( i == 14 )
                        {
                            for( j = 2; j < 8; j++ )
                            {
                                if( KB_Data_Pack[ j ] == DEF_KEY_CHAR_S )
                                {
                                    break;
                                }
                            }
                        }
                        else if( i == 15 )
                        {
                            for( j = 2; j < 8; j++ )
                            {
                                if( KB_Data_Pack[ j ] == DEF_KEY_CHAR_D )
                                {
                                    break;
                                }
                            }
                        }

                        if( j == 8 )
                        {
                            KB_Data_Pack[ 5 ] = 0;
                        }
                        else
                        {
                            memcpy( &KB_Data_Pack[ j ], &KB_Data_Pack[ j + 1 ], ( 8 - j - 1 ) );
                            KB_Data_Pack[ 7 ] = 0;
                        }
                        key_cnt--;
                    }
                    else                                            // Key release
                    {
                        if( i == 12 )
                        {
                            KB_Data_Pack[ 2 + key_cnt ] = DEF_KEY_CHAR_W;
                        }
                        else if( i == 13 )
                        {
                            KB_Data_Pack[ 2 + key_cnt ] = DEF_KEY_CHAR_A;
                        }
                        else if( i == 14 )
                        {
                            KB_Data_Pack[ 2 + key_cnt ] = DEF_KEY_CHAR_S;
                        }
                        else if( i == 15 )
                        {
                            KB_Data_Pack[ 2 + key_cnt ] = DEF_KEY_CHAR_D;
                        }
                        key_cnt++;
                    }
                }
            }

            /* Copy the keyboard data to the buffer of endpoint 1 and set the data uploading flag */
            KB_Scan_Last_Result = KB_Scan_Result;
            flag = 1;
        }
    }

    if( flag )
    {
        /* Load keyboard data to endpoint 1 */
        status = USBHS_Endp_DataUp( DEF_UEP1, KB_Data_Pack, sizeof( KB_Data_Pack ), DEF_UEP_CPY_LOAD );

        if( status == READY )
        {
            /* Clear flag after successful loading */
            flag = 0;
        }
    }
}

/*********************************************************************
 * @fn      KB_LED_Handle
 *
 * @brief   Handle keyboard lighting.
 *
 * @return  none
 */
void KB_LED_Handle( void )
{
    if( KB_LED_Cur_Status != KB_LED_Last_Status )
    {
        if( ( KB_LED_Cur_Status & 0x01 ) != ( KB_LED_Last_Status & 0x01 ) )
        {
            if( KB_LED_Cur_Status & 0x01 )
            {
                printf("Turn on the NUM LED\r\n");
            }
            else
            {
                printf("Turn off the NUM LED\r\n");
            }
        }
        if( ( KB_LED_Cur_Status & 0x02 ) != ( KB_LED_Last_Status & 0x02 ) )
        {
            if( KB_LED_Cur_Status & 0x02 )
            {
                printf("Turn on the CAPS LED\r\n");
            }
            else
            {
                printf("Turn off the CAPS LED\r\n");
            }
        }
        if( ( KB_LED_Cur_Status & 0x04 ) != ( KB_LED_Last_Status & 0x04 ) )
        {
            if( KB_LED_Cur_Status & 0x04 )
            {
                printf("Turn on the SCROLL LED\r\n");
            }
            else
            {
                printf("Turn off the SCROLL LED\r\n");
            }
        }
        KB_LED_Last_Status = KB_LED_Cur_Status;
    }
}


/*********************************************************************
 * @fn      MS_Scan_Init
 *
 * @brief   Initialize IO for mouse scan.
 *
 * @return  none
 */
void MS_Scan_Init( void )
{
    GPIO_InitTypeDef GPIO_InitStructure = { 0 };

    /* Enable GPIOA clock */
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC, ENABLE );

    /* Initialize GPIOA for the mouse scan */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOA, &GPIO_InitStructure );
}

/*********************************************************************
 * @fn      MS_Sleep_Wakeup_Cfg
 *
 * @brief   Configure mouse wake up mode.
 *
 * @return  none
 */
void MS_Sleep_Wakeup_Cfg( void )
{
    EXTI_InitTypeDef EXTI_InitStructure = { 0 };

    /* Enable GPIOC clock */
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO, ENABLE );

    GPIO_EXTILineConfig( RCC_APB2Periph_GPIOC, GPIO_PinSource0 );
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Event;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init( &EXTI_InitStructure );

    GPIO_EXTILineConfig( RCC_APB2Periph_GPIOC, GPIO_PinSource1 );
    EXTI_InitStructure.EXTI_Line = EXTI_Line1;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Event;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init( &EXTI_InitStructure );

    GPIO_EXTILineConfig( RCC_APB2Periph_GPIOC, GPIO_PinSource2 );
    EXTI_InitStructure.EXTI_Line = EXTI_Line2;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Event;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init( &EXTI_InitStructure );

    GPIO_EXTILineConfig( RCC_APB2Periph_GPIOC, GPIO_PinSource3 );
    EXTI_InitStructure.EXTI_Line = EXTI_Line3;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Event;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init( &EXTI_InitStructure );

    EXTI->INTENR |= EXTI_INTENR_MR0 | EXTI_INTENR_MR1 | EXTI_INTENR_MR2 | EXTI_INTENR_MR3;
}

/*********************************************************************
 * @fn      MS_Scan
 *
 * @brief   Perform mouse scan.
 *
 * @return  none
 */
void MS_Scan( void )
{
    static uint16_t scan_cnt = 0;
    static uint16_t scan_result = 0;

    scan_cnt++;
    if( scan_cnt >= 2 )
    {
        scan_cnt = 0;

        /* Determine whether the two scan results are consistent */
        if( scan_result == ( GPIO_ReadInputData( GPIOC ) & 0x000F ) )
        {
            MS_Scan_Result = scan_result;
            MS_Scan_Done = 1;
        }
    }
    else if( scan_cnt >= 1 )
    {
        /* Save the first scan result */
        scan_result = ( GPIO_ReadInputData( GPIOC ) & 0x000F );
    }
}

/*********************************************************************
 * @fn      MS_Scan_Handle
 *
 * @brief   Handle mouse scan data.
 *
 * @return  none
 */
void MS_Scan_Handle( void )
{
    uint8_t i;
    uint8_t status;
    static uint8_t flag = 0x00;

    if( MS_Scan_Done )
    {
        MS_Scan_Done = 0;

        memset( MS_Data_Pack, 0x00, sizeof( MS_Data_Pack ) );

        for( i = 4; i < 8; i++ )
        {
            /* Determine that the mouse is moved */
            if( ( MS_Scan_Result & ( 1 << i ) ) == 0 )
            {
                if( i == 4 )
                {
                    MS_Data_Pack[ 1 ] += 0x02;
                }
                else if( i == 5 )
                {
                    MS_Data_Pack[ 1 ] += 0xFE;
                }
                else if( i == 6 )
                {
                    MS_Data_Pack[ 2 ] += 0x02;
                }
                else if( i == 7 )
                {
                    MS_Data_Pack[ 2 ] += 0xFE;
                }

                /* Set the data uploading flag */
                flag = 1;
            }
        }
    }

    if( flag )
    {
        /* Load mouse data to endpoint 2 */
        status = USBHS_Endp_DataUp( DEF_UEP2, MS_Data_Pack, sizeof( MS_Data_Pack ), DEF_UEP_CPY_LOAD );

        if( status == READY )
        {
            /* Clear flag after successful loading */
            flag = 0;
        }
    }
}

/*********************************************************************
 * @fn      USB_Sleep_Wakeup_CFG
 *
 * @brief   Configure USB wake up mode
 *
 * @return  none
 */
void USB_Sleep_Wakeup_CFG( void )
{
    EXTI_InitTypeDef EXTI_InitStructure = { 0 };

    EXTI_InitStructure.EXTI_Line = EXTI_Line20;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Event;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init( &EXTI_InitStructure );
}

/*********************************************************************
 * @fn      MCU_Sleep_Wakeup_Operate
 *
 * @brief   Perform sleep operation
 *
 * @return  none
 */
void MCU_Sleep_Wakeup_Operate( void )
{
    printf( "Sleep\r\n" );
    __disable_irq( );
    USBHSD->HOST_CTRL &= ~USBHS_UH_PHY_SUSPENDM;
    EXTI_ClearFlag( EXTI_Line12 | EXTI_Line13 | EXTI_Line14 | EXTI_Line15 );
    EXTI_ClearFlag( EXTI_Line0 | EXTI_Line1 | EXTI_Line2 | EXTI_Line3 );

    _SEV( );
    _WFE( );
    if( USBHSD->SUSPEND & USBHS_USB_WAKEUP_ST )
    {
        USBHSD->HOST_CTRL |= USBHS_UH_PHY_SUSPENDM;
        __enable_irq( );
        return;
    }
    PWR_EnterSTOPMode(PWR_Regulator_LowPower,PWR_STOPEntry_WFE);

    SystemInit();
    SystemCoreClockUpdate();
    USBHSD->HOST_CTRL |= USBHS_UH_PHY_SUSPENDM;
    USBHS_RCC_Init();

    if( EXTI_GetFlagStatus( EXTI_Line12 | EXTI_Line13 | EXTI_Line14 | EXTI_Line15 ) != RESET  )
    {
        EXTI_ClearFlag( EXTI_Line12 | EXTI_Line13 | EXTI_Line14 | EXTI_Line15 );
        USBHS_Send_Resume( );
    }
    else if( EXTI_GetFlagStatus( EXTI_Line0 | EXTI_Line1 | EXTI_Line2 | EXTI_Line3 ) != RESET )
    {
        EXTI_ClearFlag( EXTI_Line0 | EXTI_Line1 | EXTI_Line2 | EXTI_Line3 );
        USBHS_Send_Resume( );
    }
    __enable_irq( );
    printf( "Wake\r\n" );
}


/*********************************************************************
 * @fn      KB_ADC_INIT
 *
 * @brief   adc init
 *
 * @return  none
 */

void KB_ADC_INIT( void )
{
    ADC_InitTypeDef ADC_InitStructure={0};
    GPIO_InitTypeDef GPIO_InitStructure={0};
    DMA_InitTypeDef DMA_InitStructure={0};

    // 1. 클럭 활성화
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE );
    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_DMA1, ENABLE );
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);

    // 2. GPIO 설정 (PA0~PA4)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 3. DMA1 Channel1 설정
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->RDATAR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)ADC_Values;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = NUM_CHANNELS;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; 
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init( DMA1_Channel1, &DMA_InitStructure );
    DMA_Cmd( DMA1_Channel1, ENABLE );

    // 4. ADC 설정
    ADC_DeInit(ADC1);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = NUM_CHANNELS;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 6, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 7, ADC_SampleTime_239Cycles5);

    ADC_DMACmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
    Calibrattion_Val = Get_CalibrationValue(ADC1); 

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

/*********************************************************************
 * @fn      adcKB
 *
 * @brief   keyboard scan adc data.
 *
 * @return  none
 */

uint16_t adcKB( void ){
    uint16_t scan_result = 0;
    scan_result |= (ADC_Values[0] > 1700) ? 0x8000 : 0;
    scan_result |= (ADC_Values[1] > 1700) ? 0x4000 : 0;
    scan_result |= (ADC_Values[2] > 1700) ? 0x2000 : 0;
    scan_result |= (ADC_Values[3] > 1700) ? 0x1000 : 0;
    scan_result |= (ADC_Values[4] > 1700) ? 0x0800 : 0;
    return scan_result;
}


/*********************************************************************
 * @fn      ADCKB_Scan
 *
 * @brief   Perform keyboard scan.
 *
 * @return  none
 */

void ADCKB_Scan( void )
{

    static uint16_t scan_cnt = 0;
    static uint16_t scan_result = 0;

    scan_cnt++;
    if( ( scan_cnt % 10 ) == 0 )
    {
        scan_cnt = 0;

        /* Determine whether the two scan results are consistent */
        if( scan_result == ( adcKB() & 0xF800 ) )
        //여기에 scan을 GPIO_READInputData 말고 ADC 데이터로 바꾼 후, if문 추가로 자석축 사용.
        {
            ADC_Scan_Done = 1;
            ADC_Scan_Result = scan_result;
        }
    }
    else if( ( scan_cnt % 5 ) == 0 )
    {
        /* Save the first scan result */
        scan_result = ( adcKB() & 0xF800 );
    }
}



/*********************************************************************
 * @fn      ADC_Scan_Handle
 *
 * @brief   Handle keyboard scan data.
 *
 * @return  none
 */
void ADC_Scan_Handle( void ){
    uint8_t i, j;
    uint8_t status;
    static uint8_t key_cnt = 0x00;
    static uint8_t flag = 0x00;

    if( ADC_Scan_Done ){
        ADC_Scan_Done = 0;

        if( ADC_Scan_Result != ADC_Scan_Last_Result ){
            for( i = 11; i < 16; i++ ){
                /* Determine that there is at least one key is pressed or released */
                if( ( ADC_Scan_Result & ( 1 << i ) ) != ( ADC_Scan_Last_Result & ( 1 << i ) ) ){
                    if( ( ADC_Scan_Result & ( 1 << i ) ) ) {          // Key press
                        if( i == 11 ){
                            for( j = 2; j < 8; j++ )
                            {
                                if( ADC_Data_Pack[ j ] == DEF_KEY_CHAR_F )
                                {
                                    break;
                                }
                            }
                        }
                        if( i == 12 ){
                            for( j = 2; j < 8; j++ )
                            {
                                if( ADC_Data_Pack[ j ] == DEF_KEY_CHAR_W )
                                {
                                    break;
                                }
                            }
                        }
                        else if( i == 13 )
                        {
                            for( j = 2; j < 8; j++ )
                            {
                                if( ADC_Data_Pack[ j ] == DEF_KEY_CHAR_A )
                                {
                                    break;
                                }
                            }
                        }
                        else if( i == 14 )
                        {
                            for( j = 2; j < 8; j++ )
                            {
                                if( ADC_Data_Pack[ j ] == DEF_KEY_CHAR_S )
                                {
                                    break;
                                }
                            }
                        }
                        else if( i == 15 )
                        {
                            for( j = 2; j < 8; j++ )
                            {
                                if( ADC_Data_Pack[ j ] == DEF_KEY_CHAR_D )
                                {
                                    break;
                                }
                            }
                        }

                        if( j == 8 )
                        {
                            ADC_Data_Pack[ 5 ] = 0;
                        }
                        else
                        {
                            memcpy( &ADC_Data_Pack[ j ], &ADC_Data_Pack[ j + 1 ], ( 8 - j - 1 ) );
                            ADC_Data_Pack[ 7 ] = 0;
                        }
                        key_cnt--;
                    }
                    else                                            // Key release
                    {
                        if( i == 11 )
                        {
                            ADC_Data_Pack[ 2 + key_cnt ] = DEF_KEY_CHAR_F;
                        }
                        else if( i == 12 )
                        {
                            ADC_Data_Pack[ 2 + key_cnt ] = DEF_KEY_CHAR_W;
                        }
                        else if( i == 13 )
                        {
                            ADC_Data_Pack[ 2 + key_cnt ] = DEF_KEY_CHAR_A;
                        }
                        else if( i == 14 )
                        {
                            ADC_Data_Pack[ 2 + key_cnt ] = DEF_KEY_CHAR_S;
                        }
                        else if( i == 15 )
                        {
                            ADC_Data_Pack[ 2 + key_cnt ] = DEF_KEY_CHAR_D;
                        }
                        key_cnt++;
                    }
                }
            }

            /* Copy the keyboard data to the buffer of endpoint 1 and set the data uploading flag */
            ADC_Scan_Last_Result = ADC_Scan_Result;
            flag = 1;
        }
    }

    if( flag )
    {
        /* Load keyboard data to endpoint 1 */
        status = USBHS_Endp_DataUp( DEF_UEP1, ADC_Data_Pack, sizeof( ADC_Data_Pack ), DEF_UEP_CPY_LOAD );

        if( status == READY )
        {
            /* Clear flag after successful loading */
            flag = 0;
        }
    }
}


void ADCMS_Scan( void )
{
    static uint16_t adc_tick = 0;

    adc_tick++;
    
    // 2ms 마다 플래그 세우기 (속도가 너무 빠르면 3~4로 늘리세요)
    if( adc_tick >= 2 ) 
    {
        adc_tick = 0;
        ADCMS_Scan_Done = 1; // "메인 루프야, 조이스틱 계산해라!" 신호
    }
}

void ADCMS_Scan_Handle( void )
{
    // 적분 오차를 기억하기 위한 정적 변수 (함수 종료 후에도 값 유지)
    static float virt_x = 0.0f;
    static float virt_y = 0.0f;

    // 플래그 확인: 타이머 인터럽트가 스캔 신호를 보냈는가?
    if( ADCMS_Scan_Done )
    {
        ADCMS_Scan_Done = 0; // 플래그 초기화

        // 1. DMA 버퍼에서 최신 ADC Raw 값 읽기
        uint16_t raw_x = ADC_Values[JS_IDX_X];
        uint16_t raw_y = ADC_Values[JS_IDX_Y];

        // 2. 중앙값 기준 차이 계산 (Offset)
        float diff_x = (float)raw_x - JS_CENTER_X;
        float diff_y = (float)raw_y - JS_CENTER_Y;
        
        // 절대값 계산 (fabsf는 float 전용 절대값 함수)
        float abs_x = fabsf(diff_x);
        float abs_y = fabsf(diff_y);
        
        // 정규화된 값을 담을 변수 (-1.0 ~ 1.0)
        float norm_x = 0.0f;
        float norm_y = 0.0f;

        // 3. 점프 보정 (Jump Correction) & 정규화
        // X축 처리
        if (abs_x > JS_JUMP_THRESHOLD) 
        {
            // (현재 오차 - 임계값) / (최대 범위 - 임계값) = 0.0 ~ 1.0 비율 생성
            float ratio = (abs_x - JS_JUMP_THRESHOLD) / (JS_FULL_SCALE - JS_JUMP_THRESHOLD);
            
            // 원래 방향(부호) 복구
            if (diff_x > 0) norm_x = ratio;
            else            norm_x = -ratio;
        }
        else 
        {
            norm_x = 0.0f; // 임계값 이하는 노이즈로 간주하여 0 처리
        }

        // Y축 처리
        if (abs_y > JS_JUMP_THRESHOLD) 
        {
            float ratio = (abs_y - JS_JUMP_THRESHOLD) / (JS_FULL_SCALE - JS_JUMP_THRESHOLD);
            
            if (diff_y > 0) norm_y = ratio;
            else            norm_y = -ratio;
        }
        else 
        {
            norm_y = 0.0f;
        }

        // 4. 원형 제한 (Circular Clamping)
        // 대각선 이동 시 사각형 모서리(길이 > 1.0)로 나가는 것을 방지
        float magnitude = sqrtf(norm_x * norm_x + norm_y * norm_y);
        
        if (magnitude > 1.0f) 
        {
            norm_x = norm_x / magnitude;
            norm_y = norm_y / magnitude;
        }

        // 5. 가상 좌표 적분 (Virtual Coordinate Integration)
        // 목표 위치 계산 (비율 * 최대 반경 100픽셀)
        float target_x = norm_x * JS_MAX_RADIUS;
        float target_y = norm_y * JS_MAX_RADIUS;

        // 이동해야 할 델타값(차이) 계산: 목표 위치 - 현재 가상 위치
        float delta_x = target_x - virt_x;
        float delta_y = target_y - virt_y;

        // 실제 USB로 보낼 정수값 변환 (int8_t: -128 ~ 127)
        int8_t send_x = (int8_t)delta_x;
        int8_t send_y = (int8_t)delta_y;

        // 6. 유의미한 움직임이 있을 때만 USB 전송
        if (send_x != 0 || send_y != 0)
        {
            // [중요] 버튼 상태 동기화
            // ADCMS 패킷이지만, 버튼 상태는 공유 변수(Current_Button_State)를 사용해야 함
            ADCMS_Data_Pack[0] = Current_Button_State; 
            
            // 이동 데이터 입력
            ADCMS_Data_Pack[1] = send_x;
            ADCMS_Data_Pack[2] = send_y;
            ADCMS_Data_Pack[3] = 0; // 휠 데이터 (사용 안 함)

            // 가상 좌표 업데이트 (중요: 실제 보낸 send 값만큼만 더해야 오차 누적 안 됨)
            virt_x += (float)send_x;
            virt_y += (float)send_y;

            // USB 엔드포인트 2번으로 데이터 업로드
            USBHS_Endp_DataUp( DEF_UEP2, ADCMS_Data_Pack, 4, DEF_UEP_CPY_LOAD );
        }
    }
}