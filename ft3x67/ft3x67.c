/**
  ******************************************************************************
  * @file    ft3x67.c
  * @author  MCD Application Team
  * @brief   This file provides a set of functions needed to manage the FT3X67
  *          touch screen devices.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "ft3x67.h"
#include "stm32l4xx_hal.h"
#include "stm32l4r9xx.h"
#include "stm32l4r9i_eval.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup Component
  * @{
  */

/** @addtogroup FT3X67
  * @{
  */

/* Private typedef -----------------------------------------------------------*/

/** @defgroup FT3X67_Private_Types_Definitions
  * @{
  */

/* ft3x67 Handle definition. */
typedef struct
{
  uint8_t i2cInitialized;

  /* field holding the current number of simultaneous active touches */
  uint8_t currActiveTouchNb;

  /* field holding the touch index currently managed */
  uint8_t currActiveTouchIdx;

} ft3x67_handle_TypeDef;

/**
  * @}
  */

/* Private define ------------------------------------------------------------*/
enum raydium_pt_report_idx {
    POS_SEQ = 0,/*1:touch, 0:no touch*/
    POS_PT_AMOUNT,
    POS_PT_ID = 0,
    POS_X_L,
    POS_X_H,
    POS_Y_L,
    POS_Y_H,
    POS_PRESSURE_L,
    POS_PRESSURE_H,
    POS_WX_L,
    POS_WX_H,
    POS_WY_L,
    POS_WY_H,
    LENGTH_PT = 11
};

/* Raydium device id */
#define RAYDIUM_DEVICE_ID   0xF301

/* I2C bus slave address(ID) */
#define RAYDIUM_I2C_EID     (0x5A<<1)
#define RAYDIUM_I2C_NID     (0x39<<1)

#define RAYDIUM_I2C_PDA_MODE_DISABLE      0x00
#define RAYDIUM_I2C_PDA_MODE_ENABLE       0x80
#define RAYDIUM_I2C_PDA_MODE_WORD_MODE    0x40    // Using byte mode due to data might be not word-aligment
#define RAYDIUM_I2C_PDA_2_MODE_DISABLE    0x20

/* PDA literal */
#define MASK_8BIT    0xFF
#define RAYDIUM_I2C_PDA_ADDRESS_LENGTH    4

/* PDA address and bit definition*/
#define RAYDIUM_READ_FT_DATA_CMD        0x2000019C
#define RAYDIUM_GESTURE_STATE_CMD       0x200005F4    // 1byte, disable:0x00 ; enable:0x20
#define RAYDIUM_GESTURE_DISABLE         0x00
#define RAYDIUM_GESTURE_ENABLE          0x20
#define RAYDIUM_GESTURE_RESULT_CMD      0x200005F0    // 4bytes, [0]:ready ; [1]:type ; [2]:direction
#define RAYDIUM_CHECK_I2C_CMD           0x500009BC
#define RAYDIUM_PDA2_CTRL_CMD           0x50000628
#define RAYDIM_ENABLE_PDA2              0x04

/* PDA2 setting */
#define MAX_PAGE_AMOUNT    11    // Page 0 ~ Page A

/* PDA2 address and setting definition*/
#define RAYDIUM_PDA2_TCH_RPT_STATUS_ADDR    0x00    // only in Page 0
#define RAYDIUM_PDA2_TCH_RPT_ADDR           0x01    // only in Page 0
#define RAYDIUM_PDA2_HOST_CMD_ADDR          0x02    // only in Page 0
#define RAYDIUM_PDA2_PALM_AREA_ADDR         0x03    // only in Page 0
#define RAYDIUM_PDA2_GESTURE_RPT_ADDR       0x04    // only in Page 0
#define RAYDIUM_PDA2_PALM_STATUS_ADDR       0x05    // only in Page 0
#define RAYDIUM_PDA2_FW_VERSION_ADDR        0x06    // only in Page 0
#define RAYDIUM_PDA2_PANEL_VERSION_ADDR     0x07    // only in Page 0
#define RAYDIUM_PDA2_DISPLAY_MODE_ADDR      0x08    // only in Page 0
#define RAYDIUM_PDA2_PDA_CFG_ADDR           0x09    // only in Page 0
#define RAYDIUM_PDA2_PAGE_ADDR              0x0A    // Page 0 ~ Page 9 will be directed to Page 0
#define RAYDIUM_PDA2_PAGE_0                 0x00
#define RAYDIUM_PDA2_ENABLE_PDA             0x0A    // temporary switch to PDA once
#define RAYDIUM_PDA2_2_PDA                  (MAX_PAGE_AMOUNT + 2)    // permanently switch to PDA mode

/* Touch report */
#define MAX_TOUCH_NUM                 2
#define MAX_REPORT_PAKAGE_SIZE        35
#define MAX_TCH_STATUS_PAKAGE_SIZE    2
#define MAX_GESTURERESULT_SIZE        4


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/** @defgroup FT3X67_Private_Variables
  * @{
  */

/* Touch screen driver structure initialization */
TS_DrvTypeDef ft3x67_ts_drv =
{
  ft3x67_Init,
  ft3x67_ReadID,
  ft3x67_Reset,
  ft3x67_TS_Start,
  ft3x67_TS_DetectTouch,
  ft3x67_TS_GetXY,
  ft3x67_TS_EnableIT,
  ft3x67_TS_ClearIT,
  ft3x67_TS_ITStatus,
  ft3x67_TS_DisableIT
};

/* Global ft3x67 handle */
static ft3x67_handle_TypeDef ft3x67_handle = {FT3X67_I2C_NOT_INITIALIZED, 0U, 0U};

/**
  * @}
  */

/* Private functions prototypes-----------------------------------------------*/

/** @defgroup FT3X67_Private_Functions
  * @{
  */

static uint8_t  ft3x67_Get_I2C_InitializedStatus(void);
static void     ft3x67_I2C_InitializeIfRequired(void);
static uint32_t ft3x67_TS_Configure(uint16_t DeviceAddr);

/** @defgroup Raydium_Private_Functions
  * @{
  */
static void delay_us(uint32_t delay);
static void raydium_pda_set_addr(uint32_t address, uint8_t mode);
static void raydium_pda_read(uint32_t addr, uint8_t *r_data, uint16_t length);
static void raydium_pda_write(uint32_t addr, uint8_t *w_data, uint16_t length);
static void raydium_pda2_set_page(uint8_t page);
static void raydium_pda2_read(uint8_t addr, uint8_t *r_data, uint16_t length);
static void raydium_pda2_write(uint8_t addr, uint8_t *w_data, uint16_t length);
static void raydium_hw_reset(void);


/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/

/** @addtogroup FT3X67_Exported_Functions
  * @{
  */

/**
  * @brief  Initialize the ft3x67 communication bus
  *         from MCU to FT3X67 : ie I2C channel initialization (if required).
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT3X67).
  * @retval None
  */
void ft3x67_Init(uint16_t DeviceAddr)
{
  GPIO_InitTypeDef gpio_init_structure;

  TS_DSI_RES_GPIO_CLK_ENABLE();
  TS_DSI_INT_GPIO_CLK_ENABLE();

  // Config Reset pin
  gpio_init_structure.Pin = TS_DSI_RES_PIN;
  gpio_init_structure.Pull = GPIO_PULLUP;
  gpio_init_structure.Speed = GPIO_SPEED_FAST;
  gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(TS_DSI_RES_GPIO_PORT, &gpio_init_structure);

  // Reset the touch sensor
  raydium_hw_reset();

  /* Initialize I2C link if needed */
  ft3x67_I2C_InitializeIfRequired();
}

/**
  * @brief  Software Reset the ft3x67.
  *         @note : Not applicable to FT3X67.
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT3X67).
  * @retval None
  */
void ft3x67_Reset(uint16_t DeviceAddr)
{
  /* Do nothing */
  /* No software reset sequence available in FT3X67 IC */
}

/**
  * @brief  Read the ft3x67 device ID, pre initialize I2C in case of need to be
  *         able to read the FT3X67 device ID, and verify this is a FT3X67.
  * @param  DeviceAddr: I2C FT3X67 Slave address.
  * @retval The Device ID (two bytes).
  */
uint16_t ft3x67_ReadID(uint16_t DeviceAddr)
{
  ///* Initialize I2C link if needed */
  ft3x67_I2C_InitializeIfRequired();
  //
  ///* Return the device ID value */
  //return(TS_IO_Read(DeviceAddr, FT3X67_CHIP_ID_REG));

  // Check I2C bus then return the FT3X67 ID value.
  uint8_t raydium_id[4];
  uint16_t device_id;

  raydium_pda2_set_page(RAYDIUM_PDA2_PAGE_0);
  raydium_pda_read(RAYDIUM_CHECK_I2C_CMD, raydium_id, 4);

  device_id = (raydium_id[3] << 8) | raydium_id[2];

  if(device_id == RAYDIUM_DEVICE_ID)
  {
    return FT3X67_ID_VALUE;
  }
  else
  {
    return 0;
  }
}

/**
  * @brief  Configures the touch Screen IC device to start detecting touches
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address).
  * @retval None.
  */
void ft3x67_TS_Start(uint16_t DeviceAddr)
{
  /* Minimum static configuration of FT3X67 */
  ft3x67_TS_Configure(DeviceAddr);

  /* By default set FT3X67 IC in Polling mode : no INT generation on FT3X67 for new touch available */
  /* Note TS_INT is active low                                                                      */
  ft3x67_TS_DisableIT(DeviceAddr);
}

/**
  * @brief  Return if there is touches detected or not.
  *         Try to detect new touches and forget the old ones (reset internal global
  *         variables).
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval : Number of active touches detected (can be 0, 1 or 2).
  */
uint8_t ft3x67_TS_DetectTouch(uint16_t DeviceAddr)
{
  //volatile uint8_t nbTouch = 0U;
  //
  ///* Read register FT3X67_TD_STAT_REG to check number of touches detection */
  //nbTouch = TS_IO_Read(DeviceAddr, FT3X67_TD_STAT_REG);
  //nbTouch &= FT3X67_TD_STAT_MASK;
  //
  //if(nbTouch > FT3X67_MAX_DETECTABLE_TOUCH)
  //{
  //  /* If invalid number of touch detected, set it to zero */
  //  nbTouch = 0U;
  //}
  //
  ///* Update ft3x67 driver internal global : current number of active touches */
  //ft3x67_handle.currActiveTouchNb = nbTouch;
  //
  ///* Reset current active touch index on which to work on */
  //ft3x67_handle.currActiveTouchIdx = 0U;
  //
  //return(nbTouch);


  uint8_t nbTouch[MAX_TCH_STATUS_PAKAGE_SIZE];

  // Clear interrupt
  raydium_pda2_set_page(RAYDIUM_PDA2_PAGE_0);

  //read touch point information
  raydium_pda2_read(RAYDIUM_PDA2_TCH_RPT_STATUS_ADDR, nbTouch, MAX_TCH_STATUS_PAKAGE_SIZE);

  if(nbTouch[POS_PT_AMOUNT] > FT3X67_MAX_DETECTABLE_TOUCH)
  {
    // If invalid number of touch detected, set it to zero
    nbTouch[POS_PT_AMOUNT] = 0U;
  }

  // Update ft3x67 driver internal global : current number of active touches
  ft3x67_handle.currActiveTouchNb = nbTouch[POS_PT_AMOUNT];

  // Reset current active touch index on which to work on
  ft3x67_handle.currActiveTouchIdx = 0U;

  if(nbTouch[POS_PT_AMOUNT] == 0)
  {
    // clear touch seq num
    uint8_t w_data = 0;
    raydium_pda2_write(RAYDIUM_PDA2_TCH_RPT_STATUS_ADDR, &w_data, 1);
  }

  return(nbTouch[POS_PT_AMOUNT]);
}

/**
  * @brief  Get the touch screen X and Y positions values
  *         Manage multi touch thanks to touch Index global
  *         variable 'ft3x67_handle.currActiveTouchIdx'.
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  X: Pointer to X position value
  * @param  Y: Pointer to Y position value
  * @retval None.
  */
void ft3x67_TS_GetXY(uint16_t DeviceAddr, uint16_t *X, uint16_t *Y)
{
  //uint8_t regAddress = 0U;
  //uint8_t dataxy[4U];
  //
  //if(ft3x67_handle.currActiveTouchIdx < ft3x67_handle.currActiveTouchNb)
  //{
  //  switch(ft3x67_handle.currActiveTouchIdx)
  //  {
  //  case 0U :
  //    regAddress = FT3X67_P1_XH_REG;
  //    break;
  //
  //  case 1U :
  //    regAddress = FT3X67_P2_XH_REG;
  //    break;
  //
  //  default :
  //    break;
  //  } /* end switch(ft3x67_handle.currActiveTouchIdx) */
  //
  //  /* Read X and Y positions */
  //  TS_IO_ReadMultiple(DeviceAddr, regAddress, dataxy, sizeof(dataxy));
  //
  //  /* Send back ready X position to caller */
  //  *X = ((dataxy[0U] & FT3X67_TOUCH_POS_MSB_MASK) << 8U) | dataxy[1U];
  //
  //  /* Send back ready Y position to caller */
  //  *Y = ((dataxy[2U] & FT3X67_TOUCH_POS_MSB_MASK) << 8U) | dataxy[3U];
  //
  //  /* Increment current touch index */
  //  ft3x67_handle.currActiveTouchIdx++;
  //}

  uint8_t data_index = 0;
  uint8_t w_data = 0;
  uint8_t regAddress = RAYDIUM_PDA2_TCH_RPT_ADDR;

  static uint8_t dataxy[LENGTH_PT * MAX_TOUCH_NUM]; // May be the raydium IC packed all touch point data in a page
                                             // We have to read the whole page to get these information in a single command.
                                             // It means that if we want to get report for touch id "1", we have to read the report of
                                             // touch id "0" also.

                                             // NOTE: There is proposal to optimize this function:
                                             // Read the whole page to get all touch report if this function is called when
                                             // currActiveTouchIdx = 0, then backup X/Y value of the touch report id "1"
                                             // If the function is called when currActiveTouchIdx = 1, this function just need
                                             // to return the backup X/Y value.

  if(ft3x67_handle.currActiveTouchIdx < ft3x67_handle.currActiveTouchNb)
  {
    /* Read X and Y positions */

    switch(ft3x67_handle.currActiveTouchIdx)
    {
    case 0U :
      data_index = 0;
      raydium_pda2_read(regAddress, dataxy, LENGTH_PT * ft3x67_handle.currActiveTouchNb);

      // clear touch seq num
      raydium_pda2_write(RAYDIUM_PDA2_TCH_RPT_STATUS_ADDR, &w_data, 1);

      break;

    case 1U :
      data_index = LENGTH_PT;
      break;

    default :
      break;
    } /* end switch(ft3x67_handle.currActiveTouchIdx) */
    /* Send back ready X position to caller */
    *X = (dataxy[data_index + POS_X_H] << 8U) | dataxy[data_index + POS_X_L];

    /* Send back ready Y position to caller */
    *Y = (dataxy[data_index + POS_Y_H] << 8U) | dataxy[data_index + POS_Y_L];

    /* Increment current touch index */
    ft3x67_handle.currActiveTouchIdx++;
  }
}

/**
  * @brief  Configure the FT3X67 device to generate IT on given INT pin
  *         connected to MCU as EXTI.
  * @param  DeviceAddr: Device address on communication Bus (Slave I2C address of FT3X67).
  * @retval None
  */
void ft3x67_TS_EnableIT(uint16_t DeviceAddr)
{
  /* Set interrupt trigger mode in FT3X67_GMODE_REG */
  //TS_IO_Write(DeviceAddr, FT3X67_GMODE_REG, FT3X67_G_MODE_INTERRUPT_TRIGGER);
  GPIO_InitTypeDef gpio_init_structure;

  /* Configure Interrupt mode for TS detection pin */
  gpio_init_structure.Pin = TS_DSI_INT_PIN;
  gpio_init_structure.Pull = GPIO_PULLUP;
  gpio_init_structure.Speed = GPIO_SPEED_HIGH;
  gpio_init_structure.Mode = GPIO_MODE_IT_FALLING;
  HAL_GPIO_Init(TS_DSI_INT_GPIO_PORT, &gpio_init_structure);
}

/**
  * @brief  Configure the FT3X67 device to stop generating IT on the given INT pin
  *         connected to MCU as EXTI.
  * @param  DeviceAddr: Device address on communication Bus (Slave I2C address of FT3X67).
  * @retval None
  */
void ft3x67_TS_DisableIT(uint16_t DeviceAddr)
{
  /* Set interrupt polling mode in FT3X67_GMODE_REG */
  //TS_IO_Write(DeviceAddr, FT3X67_GMODE_REG, FT3X67_G_MODE_INTERRUPT_POLLING);
  GPIO_InitTypeDef gpio_init_structure;

  /* Configure Interrupt mode for TS detection pin */
  gpio_init_structure.Pin = TS_DSI_INT_PIN;
  gpio_init_structure.Pull = GPIO_PULLUP;
  gpio_init_structure.Speed = GPIO_SPEED_HIGH;
  gpio_init_structure.Mode = GPIO_MODE_INPUT;
  HAL_GPIO_Init(TS_DSI_INT_GPIO_PORT, &gpio_init_structure);
}

/**
  * @brief  Get IT status from FT3X67 interrupt status registers
  *         Should be called Following an EXTI coming to the MCU to know the detailed
  *         reason of the interrupt.
  *         @note : This feature is not applicable to FT3X67.
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT3X67).
  * @retval TS interrupts status : always return 0 here
  */
uint8_t ft3x67_TS_ITStatus(uint16_t DeviceAddr)
{
  /* Always return 0 as feature not applicable to FT3X67 */
  return 0U;
}

/**
  * @brief  Clear IT status in FT3X67 interrupt status clear registers
  *         Should be called Following an EXTI coming to the MCU.
  *         @note : This feature is not applicable to FT3X67.
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT3X67).
  * @retval None
  */
void ft3x67_TS_ClearIT(uint16_t DeviceAddr)
{
  /* Nothing to be done here for FT3X67 */
}

/**
  * @brief  Configure gesture feature (enable/disable).
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT3X67).
  * @param  Activation : Enable or disable gesture feature. Possible values are
  *         FT3X67_GESTURE_DISABLE or FT3X67_GESTURE_ENABLE.
  * @retval None.
  */
void ft3x67_TS_GestureConfig(uint16_t DeviceAddr, uint32_t Activation)
{
  //if(Activation == FT3X67_GESTURE_ENABLE)
  //{
  //  /* Enable gesture feature. */
  //  TS_IO_Write(DeviceAddr, FT3X67_GESTURE_FLAG_REG, FT3X67_GEST_ALL_FLAGS_ENABLE);
  //  TS_IO_Write(DeviceAddr, FT3X67_GESTURE_ENABLE_REG, FT3X67_GESTURE_ENABLE);
  //}
  //else
  //{
  //  /* Disable gesture feature. */
  //  TS_IO_Write(DeviceAddr, FT3X67_GESTURE_FLAG_REG, FT3X67_GEST_ALL_FLAGS_DISABLE);
  //  TS_IO_Write(DeviceAddr, FT3X67_GESTURE_ENABLE_REG, FT3X67_GESTURE_DISABLE);
  //}

  uint8_t buf;

  if(FT3X67_GESTURE_ENABLE == Activation)
  {
    buf = RAYDIUM_GESTURE_ENABLE;
  }
  else
  {
    buf = RAYDIUM_GESTURE_DISABLE;
  }

  raydium_pda_write(RAYDIUM_GESTURE_STATE_CMD, &buf, 1);
}

/**
  * @brief  Get the last touch gesture identification (zoom, move up/down...).
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT3X67).
  * @param  pGestureId : Pointer to get last touch gesture Identification.
  * @retval None.
  */
void ft3x67_TS_GetGestureID(uint16_t DeviceAddr, uint32_t * pGestureId)
{
  //volatile uint8_t ucReadData = 0U;
  //
  //ucReadData = TS_IO_Read(DeviceAddr, FT3X67_GEST_ID_REG);
  //
  //*pGestureId = ucReadData;


  // NOTE: Since we do not have datasheet for the Raydium IC, and the Linux driver does not have any information about the
  //       format of gesture report, so we have to debug to find out the format of the Raydium gesture report then convert it into
  //       ft3x67 Gesture ID.

  uint8_t buf[MAX_GESTURERESULT_SIZE]; // 4bytes, [0]:ready ; [1]:type ; [2]:direction

  raydium_pda2_read(RAYDIUM_PDA2_GESTURE_RPT_ADDR, buf, sizeof(buf));

  // TODO
  // Convert into ft3x67 gesture ID
}

/**
  * @brief  Get the touch detailed informations on touch number 'touchIdx' (0..1)
  *         This touch detailed information contains :
  *         - weight that was applied to this touch
  *         - sub-area of the touch in the touch panel
  *         - event of linked to the touch (press down, lift up, ...)
  * @param  DeviceAddr: Device address on communication Bus (I2C slave address of FT3X67).
  * @param  touchIdx : Passed index of the touch (0..1) on which we want to get the
  *                    detailed information.
  * @param  pWeight : Pointer to to get the weight information of 'touchIdx'.
  * @param  pArea   : Pointer to to get the sub-area information of 'touchIdx'.
  * @param  pEvent  : Pointer to to get the event information of 'touchIdx'.
  * @note   Area and Weight features are not supported by FT3X67. Return always 0 value.
  * @retval None.
  */
void ft3x67_TS_GetTouchInfo(uint16_t   DeviceAddr,
                            uint32_t   touchIdx,
                            uint32_t * pWeight,
                            uint32_t * pArea,
                            uint32_t * pEvent)
{
  //volatile uint8_t ucReadData = 0U;
  //uint8_t regAddressXHigh     = 0U;
  //
  //if(touchIdx < ft3x67_handle.currActiveTouchNb)
  //{
  //  switch(touchIdx)
  //  {
  //  case 0U :
  //    regAddressXHigh   = FT3X67_P1_XH_REG;
  //    break;
  //
  //  case 1U :
  //    regAddressXHigh   = FT3X67_P2_XH_REG;
  //    break;
  //  default :
  //    break;
  //  } /* end switch(touchIdx) */
  //
  //  /* Read Event Id of touch index */
  //  ucReadData = TS_IO_Read(DeviceAddr, regAddressXHigh);
  //  *pEvent = (ucReadData & FT3X67_TOUCH_EVT_FLAG_MASK) >> FT3X67_TOUCH_EVT_FLAG_SHIFT;
  //
  //  /* Weight and area of touch index not supported by FT3X67 */
  //  *pWeight = 0;
  //  *pArea   = 0;
  //} /* of if(touchIdx < ft3x67_handle.currActiveTouchNb) */

  // NOTE:      Since the Raydium IC does not support to get the touch info of specific touch id.
  //            We have to read the whole page to get all touch info.
  //            There are proposals to optimize this function:
  //
                                             // 1.  Read the whole page to get all touch report if this function is called when
                                             //     currActiveTouchIdx = 0, then backup X/Y value of the touch report id "1"
                                             //     If the function is called when currActiveTouchIdx = 1, this function just need
                                             //     to return the backup X/Y value.

                                             // 2.  Since return value of both functions ft3x67_TS_GetTouchInfo and ft3x67_TS_GetXY
                                             //     are get from the RAYDIUM_PDA2_TCH_RPT_ADDR page.
                                             //     So we can use a function which will be called by ft3x67_TS_GetTouchInfo and ft3x67_TS_GetXY
                                             //     to get all touch information in the first read, then backup all of these information.
                                             //     Any calling of the function later, it just need to return the backup data.
                                             //     But we have to specify when we have to read the whole page again to update the backup data.

  // In current implement of Sparrow Demo, it seem like the Graphic lib does not use
  // pWeight, pArea (which does not support by ft3x67, but the Raydium does) data.
  // Another problem is ft3x67 has event report specify if a touch is pressed down, lifted up or in contacting.
  // And it seem like the Graphic lib is using the event report.
  // The Raydium does not has similar report for that, so we have to use software to classify the event report base on
  // the pressure and weight report from the Raydium.

  // TODO
  // Classify touch event base on Raydium touch report.
  //  *pEvent = ?????

  *pWeight = 0;
  *pArea   = 0;
  *pEvent  = FT3X67_TOUCH_EVT_FLAG_NO_EVENT;
}

/**
  * @}
  */

/* Private functions bodies---------------------------------------------------*/

/** @addtogroup FT3X67_Private_Functions
  * @{
  */

/**
  * @brief  Return the status of I2C was initialized or not.
  * @param  None.
  * @retval : I2C initialization status.
  */
static uint8_t ft3x67_Get_I2C_InitializedStatus(void)
{
  return(ft3x67_handle.i2cInitialized);
}

/**
  * @brief  I2C initialize if needed.
  * @param  None.
  * @retval : None.
  */
static void ft3x67_I2C_InitializeIfRequired(void)
{
  if(ft3x67_Get_I2C_InitializedStatus() == FT3X67_I2C_NOT_INITIALIZED)
  {
    /* Initialize TS IO BUS layer (I2C) */
    TS_IO_Init();

    /* Set state to initialized */
    ft3x67_handle.i2cInitialized = FT3X67_I2C_INITIALIZED;
  }
}

/**
  * @brief  Basic static configuration of TouchScreen
  * @param  DeviceAddr: FT3X67 Device address for communication on I2C Bus.
  * @retval Status FT3X67_STATUS_OK or FT3X67_STATUS_NOT_OK.
  */
static uint32_t ft3x67_TS_Configure(uint16_t DeviceAddr)
{
  uint32_t status = FT3X67_STATUS_OK;
  uint8_t w_data[4];

  /* Disable gesture feature */
  //TS_IO_Write(DeviceAddr, FT3X67_GESTURE_ENABLE_REG, FT3X67_GESTURE_DISABLE);
  //ft3x67_TS_GestureConfig(0, RAYDIUM_GESTURE_ENABLE);

  raydium_pda2_set_page(RAYDIUM_PDA2_PAGE_0);

  w_data[0] = 0x30;
  raydium_pda2_write(0x02, &w_data[0], 1);

  memset(w_data, 0, 4);
  raydium_pda_write(0x5000000C, w_data, 4);

  ft3x67_TS_DisableIT(0);

  raydium_hw_reset();

  ft3x67_TS_EnableIT(0);

  return(status);
}


static void delay_us(uint32_t delay)
{
  uint32_t wait_loop_index = (delay * (SystemCoreClock / (1000000U)));
  while(wait_loop_index != 0U)
  {
    wait_loop_index--;
  }
}

static void raydium_pda_set_addr(uint32_t address, uint8_t mode)
{
  uint8_t buf[RAYDIUM_I2C_PDA_ADDRESS_LENGTH];
  uint8_t addr;

  addr = RAYDIUM_I2C_EID;
  buf[0] = (address & 0x0000FF00) >> 8;
  buf[1] = (address & 0x00FF0000) >> 16;
  buf[2] = (address & 0xFF000000) >> 24;
  buf[3] = mode;

  TS_IO_WriteMultiple(addr, buf[0],&buf[1], 3);
}

static void raydium_pda_read(uint32_t addr, uint8_t *r_data, uint16_t length)
{
  uint8_t mode = 0x00;
  uint8_t buf;

  if (length == 4)
  {
      mode |= RAYDIUM_I2C_PDA_MODE_ENABLE | RAYDIUM_I2C_PDA_2_MODE_DISABLE | RAYDIUM_I2C_PDA_MODE_WORD_MODE;
  } else
  {
      mode |= RAYDIUM_I2C_PDA_MODE_ENABLE | RAYDIUM_I2C_PDA_2_MODE_DISABLE;
  }

  buf = addr & MASK_8BIT;

  raydium_pda_set_addr(addr, mode);

  //wait about 80uS
  delay_us(80);

  // TODO
  // May be need to separate below step into 2 i2c transaction to make the raydium ic work correctly.
  // one for the addr set and the other for reading data.
  TS_IO_ReadMultiple(RAYDIUM_I2C_NID, buf, r_data, length);
}


static void raydium_pda_write(uint32_t addr, uint8_t *w_data, uint16_t length)
{
  uint8_t mode = 0x00;
  uint8_t buf;

  if (length == 4)
  {
      mode |= RAYDIUM_I2C_PDA_MODE_ENABLE | RAYDIUM_I2C_PDA_2_MODE_DISABLE | RAYDIUM_I2C_PDA_MODE_WORD_MODE;
  } else
  {
      mode |= RAYDIUM_I2C_PDA_MODE_ENABLE | RAYDIUM_I2C_PDA_2_MODE_DISABLE;
  }

  buf = addr & MASK_8BIT;

  raydium_pda_set_addr(addr, mode);

  //wait about 80uS
  delay_us(80);

  // TODO
  // May be need to separate below step into 2 i2c transaction to make the raydium ic work correctly.
  // one for the addr set and the other for writing data.
  TS_IO_WriteMultiple(RAYDIUM_I2C_NID, buf, w_data, length);
}

static void raydium_hw_reset(void)
{
  HAL_GPIO_WritePin(TS_DSI_RES_GPIO_PORT, TS_DSI_RES_PIN, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(TS_DSI_RES_GPIO_PORT, TS_DSI_RES_PIN, GPIO_PIN_SET);
  HAL_Delay(400);
}

static void raydium_pda2_set_page(uint8_t page)
{
  TS_IO_WriteMultiple(RAYDIUM_I2C_NID, RAYDIUM_PDA2_PAGE_ADDR, &page, 1);
}

static void raydium_pda2_read(uint8_t addr, uint8_t *r_data, uint16_t length)
{
  // TODO
  // May be need to separate below step into 2 i2c transaction to make the raydium ic work correctly.
  // one for the addr set and the other for reading data.
  TS_IO_ReadMultiple(RAYDIUM_I2C_NID, addr, r_data, length);
}


static void raydium_pda2_write(uint8_t addr, uint8_t *w_data, uint16_t length)
{
  // TODO
  // May be need to separate below step into 2 i2c transaction to make the raydium ic work correctly.
  // one for the addr set and the other for writing data.
  TS_IO_WriteMultiple(RAYDIUM_I2C_NID, addr, w_data, length);
}


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
