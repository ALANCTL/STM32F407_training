#include "main.h"


static inline void Delay_1us(uint32_t nCnt_1us)
{
  volatile uint32_t nCnt;

  for (; nCnt_1us != 0; nCnt_1us--)
    for (nCnt = 13; nCnt != 0; nCnt--);
}

void RCC_Configuration(void)
{
      /* --------------------------- System Clocks Configuration -----------------*/
      /* GPIOA clock enable */
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
}
 
/**************************************************************************************/
 
void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /*-------------------------- GPIO Configuration for Push Button ----------------------------*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD ;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

}
 
/**************************************************************************************/
 
void LED_Initialization(void){

  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD , ENABLE); //LED3/4 GPIO Port

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 |GPIO_Pin_13 | GPIO_Pin_14 |GPIO_Pin_15;  // LED is connected to PG13/PG14
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

}

void LED3_On(void){

  GPIO_SetBits(GPIOD,GPIO_Pin_13);

}

void LED3_Off(void){

  GPIO_ResetBits(GPIOD,GPIO_Pin_13);

}

void LED5_On(void){

  GPIO_SetBits(GPIOD,GPIO_Pin_14);

}

void LED5_Off(void){

  GPIO_ResetBits(GPIOD,GPIO_Pin_14);

}

void LED3_Toggle(void){


  GPIO_ToggleBits(GPIOD,GPIO_Pin_13);

}

void LED5_Toggle(void){


  GPIO_ToggleBits(GPIOD,GPIO_Pin_14);

}

uint8_t PushButton_Read(void){

    return GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0);

}
/**************************************************************************************/
int main(void)
{
    RCC_Configuration();
    GPIO_Configuration();
    LED_Initialization();
    
    while(1)
    {

        LED5_Toggle();
        Delay_1us(100000);

      if(PushButton_Read()){

        LED3_Toggle();
      }else{

        LED3_Off();
      }
      

    }


}

