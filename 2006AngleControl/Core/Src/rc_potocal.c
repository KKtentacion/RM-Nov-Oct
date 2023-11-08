/* ----------------------- Function Implements ---------------------------- */
/******************************************************************************
* @fn RC_Init
*
* @brief configure stm32 usart3 port
* - USART Parameters
* - 100Kbps
* - 8-N-1
* - DMA Mode
*
* @return None.
*
* @note This code is fully tested on STM32F405RGT6 Platform, You can port
it
* to the other platform. Using doube buffer to receive data prevent
losing data.
*/
#include "rc_potocal.h"
/******************************************************************************
* @fn RemoteDataProcess
*
* @brief resolution rc protocol data.
* @pData a point to rc receive buffer.
* @return None.
* @note RC_CtrlDataData is a global variable.you can deal with it in other place.
*/
/* ----------------------- Internal Data ----------------------------------- */

#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define MAX_ANGLE								((uint16_t)360)
#define MIN_ANGLE								((uint16_t)0)
#define pi											3.1415926

volatile unsigned char sbus_rx_buffer[2][RC_FRAME_LENGTH]; 
//double sbus rx buffer to save data
static RC_Ctl_t RC_CtrlData;

extern float RealSetAngle[MOTOR_MAX_NUM];
float Setdeltangle;
int realdata;


int flag2=0;

void USART3_rxDataHandler(uint8_t *pData)
{
		flag2++;
    if(pData == NULL)
    {
        return;
    }
    RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;
    RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5))
    & 0x07FF;
    RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |
    ((int16_t)pData[4] << 10)) & 0x07FF;
    RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) &
    0x07FF;
    RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
    RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003);
    RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
    RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
    RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);
    RC_CtrlData.mouse.press_l = pData[12];
    RC_CtrlData.mouse.press_r = pData[13];
    RC_CtrlData.key.v = ((int16_t)pData[14]);// | ((int16_t)pData[15] << 8);

		//your control code ....
		
		Setdeltangle =RC_CtrlData.rc.ch2-RC_CH_VALUE_OFFSET;
		Setdeltangle=Setdeltangle/660*5;
		
		for(int i=0;i<MOTOR_MAX_NUM;i++)
		{
			RealSetAngle[i]+=Setdeltangle;
			if(RealSetAngle[i]>=MAX_ANGLE)
				RealSetAngle[i]-=MAX_ANGLE;
			
			if(RealSetAngle[i]<=MIN_ANGLE)
				RealSetAngle[i]+=MAX_ANGLE;
		}		
		
}