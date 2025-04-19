#include "lcd.h"
#include "main.h"
#include "stlogo.h"

/**
  * @brief  Display main demo messages.
  * @param  None
  * @retval None
  */
void Display_DemoDescription(void)
 {
   uint8_t desc[58];
   
   BSP_LCD_SetFont(&LCD_DEFAULT_FONT);
   
   /* Clear the LCD */ 
   BSP_LCD_SetBackColor(LCD_COLOR_WHITE); 
   BSP_LCD_Clear(LCD_COLOR_WHITE);
   
   /* Set the LCD Text Color */
   BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);  
   
   /* Display LCD messages */
   BSP_LCD_DisplayStringAt(0, 2, (uint8_t *)"STM32F412G", CENTER_MODE);
   BSP_LCD_DisplayStringAt(0, 14, (uint8_t *)"SD card speed measurement", CENTER_MODE);
   
   /* Draw Bitmap */
   BSP_LCD_DrawBitmap((BSP_LCD_GetXSize() - 80)/2, 30, (uint8_t *)stlogo);
   
   BSP_LCD_SetFont(&Font12);
   BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()-12, (uint8_t *)"uedatatsu 2025", CENTER_MODE);
   
   BSP_LCD_SetFont(&Font12);
   BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
   BSP_LCD_FillRect(0, BSP_LCD_GetYSize()/2 + 1, BSP_LCD_GetXSize(), 60);
   BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
   BSP_LCD_SetBackColor(LCD_COLOR_BLUE); 
   BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 + 20, (uint8_t *)"Press button to start :", CENTER_MODE);
//   sprintf((char *)desc,"%s example", BSP_examples[DemoIndex].DemoName);
//    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()/2 + 35, (uint8_t *)desc, CENTER_MODE);
 }
 
