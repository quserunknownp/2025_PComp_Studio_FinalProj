/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : WCH / Modified by Gemini
* Version            : V1.1
* Description        : USBHS CDC + 5-Channel ADC (DMA)
********************************************************************************/

#include "debug.h"
#include "ch32v30x_usbhs_device.h" // USBHS 함수들이 정의된 헤더
#include <string.h>                // memset, strlen 등 사용

/* * [중요] UART.h는 삭제했습니다. 
 * UART 중계 기능(Bridge)을 안 쓸 것이기 때문입니다.
 */

/* ================= ADC 관련 변수 및 정의 ================= */
#define NUM_CHANNELS    5
volatile u16 ADC_Values[NUM_CHANNELS]; 
s16 Calibrattion_Val = 0;
char Data_Buffer[64]; // USB로 보낼 문자열을 담을 버퍼

/* ================= ADC 초기화 함수 (그대로 가져옴) ================= */
void ADC_DMA_Init(void)
{
    ADC_InitTypeDef ADC_InitStructure={0};
    GPIO_InitTypeDef GPIO_InitStructure={0};
    DMA_InitTypeDef DMA_InitStructure={0};

    // 1. 클럭 활성화
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE );
    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_DMA1, ENABLE );
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);

    // 2. GPIO 설정 (PA0~PA4)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
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
 * @fn      main
 * @brief   Main program.
 * @return  none
 */
int main(void)
{
    /* 1. 시스템 초기화 */
    SystemCoreClockUpdate( );
    Delay_Init( );
    
    /* 디버그용 UART1 (PA9/PA10)은 놔두셔도 됩니다 (로그 확인용) */
    USART_Printf_Init( 115200 ); 
        
    printf( "SystemClk:%d\r\n", SystemCoreClock );
    printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );
    printf( "ADC to USB-CDC Mode Start!\r\n" );

    /* [삭제됨] RCC_Configuration(); -> 보통 USBHS_RCC_Init 안에 포함되거나 필요없음 */
    /* [삭제됨] TIM2_Init(); */
    /* [삭제됨] UART2_Init(...); */

    /* 2. ADC 초기화 (우리가 추가한 함수) */
    ADC_DMA_Init();

    /* 3. USBHS 초기화 (기존 코드 유지) */
    USBHS_RCC_Init( );
    USBHS_Device_Init( ENABLE );

    while(1)
    {
        /* [삭제됨] UART2_DataRx_Deal(); */
        /* [삭제됨] UART2_DataTx_Deal(); */

        /* * 4. 데이터 전송 로직 
         * ADC 값을 문자열로 포맷팅하여 USB 엔드포인트 2번(EP2 IN)으로 쏘아 올립니다.
         */
        
        // sprintf: 문자열 만들기 (예: "1024,2048,100,50,4095\n")
        // 주의: 너무 긴 문자열은 Data_Buffer 크기(64)를 넘지 않게 조절하세요.
        sprintf(Data_Buffer, "%d,%d,%d,%d,%d\r\n", 
                ADC_Values[0], ADC_Values[1], ADC_Values[2], ADC_Values[3], ADC_Values[4]);

        /* * USBHS_Endp2_DataUp: 이 함수가 바로 PC로 데이터를 쏘는 함수입니다.
         * WCH 라이브러리(ch32v30x_usbhs_device.c) 내부에 정의되어 있습니다.
         */
        USBHS_Endp2_DataUp( (uint8_t*)Data_Buffer, strlen(Data_Buffer) );

        /* * 딜레이: PC의 시리얼 터미널이 뻗지 않도록 약간의 텀을 줍니다.
         * 실제 키보드용으로는 딜레이를 1ms 이하로 줄이거나 없애야 합니다.
         */
        Delay_Ms( 10 ); 
    }
}