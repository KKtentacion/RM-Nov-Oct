/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
 
#ifndef __BSP_LED
#define __BSP_LED

#define LED_GREEN_ON()     HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_SET)
#define LED_GREEN_OFF()    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_RESET)
#define LED_GREEN_TOGGLE() HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_11)
     
#define LED_RED_ON()       HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_SET)
#define LED_RED_OFF()      HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_RESET)
#define LED_RED_TOGGLE()   HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_12)
     
#define LED_ALL_OFF        LED_RED_OFF(); LED_RED_OFF();

#endif
