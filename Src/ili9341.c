/**
  ******************************************************************************
  * @file    ili9341.c
  * @author  MCD Application Team
  * @version V1.0.2
  * @date    02-December-2014
  * @brief   This file includes the LCD driver for ILI9341 LCD.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
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
#include "ili9341.h"
#include "main.h"

void LCD_Delay(uint32_t d)
{
	Delay(d);
}

#define LCD_REG      (*((volatile uint8_t *) 0x60000000))
// A18
#define LCD_RAM     (*((volatile uint8_t *) (0x60000000+(1<<(18))) ))
// A16

inline  void FMC_BANK1_WriteData(uint8_t Data) 
{
  LCD_RAM = Data;
}

inline void FMC_BANK1_WriteComand(uint8_t Reg) 
{
  LCD_REG = Reg;
}

#if 0
uint8_t LCD_RD_DATA(void)
{
	return LCD_RAM;
}
#endif
/*

void LCD_ReadReg(uint8_t LCD_Reg,uint8_t *Rval,int n)
{
	LCD_REG = LCD_Reg;
	while(n--)
	{
		*(Rval++) = LCD_RD_DATA();
		Delay(1);
	}
}
uint16_t LCD_Read_ID4(void)
{
	uint8_t val[4] = {0};
	LCD_ReadReg(0xD300,val,4);
	printf("%x %x %x %x \n ",val[0],val[1],val[2],val[3]);
	return (val[2]<<8)|val[3];
}
*/

/**
  * @brief  Reads register value.
  * @retval Read value
  */
inline uint8_t FMC_BANK1_ReadData(void)
{
	//~ return FMC_BANK1->RAM;
	//~ return FWREAD();
  return LCD_RAM;
}
void FMC_BANK1_ReadDataN16(uint16_t *pnt,int samples)
{
	uint8_t dummy = FMC_BANK1_ReadData();
	for(int k=0;k<samples;k++)
	{
		uint32_t color;// = 0;
		color = FMC_BANK1_ReadData()<<8;
		color |=FMC_BANK1_ReadData();
		pnt[k] = color;
	}
}
#if 0

/**
  * @brief  Writes data on LCD data register.
  * @param  Data: Data to be written
  */
void  LCD_IO_WriteData(uint16_t Data)
{
  /* Write 16-bit Reg */
  FMC_BANK1_WriteData(Data);
}

/**
  * @brief  Writes multiple data on LCD data register.
  * @param  pData Pointer on the register value
  * @param  Size Size of byte to transmit to the register
  * @retval None
  */
inline void LCD_IO_WriteMultipleData(uint8_t *pData, uint32_t Size)
{
  uint32_t counter;
  uint16_t *ptr = (uint16_t *) pData;
  
  for (counter = 0; counter < Size; counter+=2)
  {  
    /* Write 16-bit Reg */
    FMC_BANK1_WriteData(*ptr);
    ptr++;
  }
}
#endif
/*****************************************************************************
 * @name       :void LCD_WR_REG(u16 data)
 * @date       :2018-08-09
 * @function   :Write an 16-bit command to the LCD screen
 * @parameters :data:Command value to be written
 * @retvalue   :None
******************************************************************************/
#if 0
void LCD_WR_REG(uint16_t data)
{
	LCD_REG = data;
}

/*****************************************************************************
 * @name       :void LCD_WR_DATA(u16 data)
 * @date       :2018-08-09
 * @function   :Write an 16-bit data to the LCD screen
 * @parameters :data:data value to be written
 * @retvalue   :None
******************************************************************************/
void LCD_WR_DATA(uint16_t data)
{
	LCD_RAM=data;
}


/**
  * @brief  Writes register on LCD register.
  * @param  Reg: Register to be written
  */
void LCD_IO_WriteReg(uint8_t Reg)
{
  /* Write 16-bit Index, then Write Reg */
  FMC_BANK1_WriteReg(Reg);
}

/**
  * @brief  Reads data from LCD data register.
  * @param  Reg: Register to be read
  * @retval Read data.
  */
inline uint8_t LCD_IO_ReadData(uint8_t Reg)
{
	FMC_BANK1_WriteComand(Reg);
  
  /* Read 16-bit Reg */  
  return FMC_BANK1_ReadData();
}

#endif
uint32_t FMC_BANK1_ReadData4()
{
	uint32_t res  = 0;
	res |= FMC_BANK1_ReadData(); res<<=8;
	res |= FMC_BANK1_ReadData(); res<<=8;
	res |= FMC_BANK1_ReadData(); res<<=8;
	res |= FMC_BANK1_ReadData();
  return res;
}


void LCD_ON()
{
	FMC_BANK1_WriteComand(LCD_DISPLAY_ON);
}
void LCD_OFF()
{
	FMC_BANK1_WriteComand(LCD_DISPLAY_OFF);
}


#define TFT_REG_COL         0x2A
#define TFT_REG_PAGE        0x2B
#define TFT_REG_MEM_WRITE   0x2C
#define TFT_REG_TE_OFF      0x34
#define TFT_REG_TE_ON       0x35
#define TFT_REG_GAMMA_1     0xe0
#define TFT_REG_GAMMA_2     0xe1

      void initTft();
     void tftReset();
     void FMC_BANK1_WriteData(uint8_t reg);
     void FMC_BANK1_WriteComand(uint8_t data);
     void sendData(uint8_t data);
     void sendData16(uint16_t data);
     void enterSleep();
     void exitSleep();
     void setOrientation(unsigned int HV);
     void setXY(uint16_t poX, uint16_t poY);
    void setCol(uint16_t startX, uint16_t endX);
    void setPage(uint16_t startY, uint16_t endY);
    void fillRectangle(uint16_t poX, uint16_t poY,
                       uint16_t width, uint16_t length,
                       uint16_t color);
    void drawHorizontalLine(uint16_t poX, uint16_t poY, uint16_t length,
                            uint16_t color);
    void setWindow(uint16_t startX, uint16_t startY, uint16_t endX, uint16_t endY);
    void setBacklightON();
    void setBacklightOff();

    void    Delay(int num)
    {
    	uint32_t start = HAL_GetTick()+num;
    	while((uint32_t)(HAL_GetTick()-start)>0x80000)
    	{

    		kscan0();
    	}
    }

/**
  * @brief  Writes data to the selected LCD register.
  * @param  LCD_Reg: address of the selected register.
  * @retval None
  */

int iBacklightPercent = 0;
void setBacklight(int percent)
{
	if(percent>100)percent = 100;
	if(percent<10)percent = 10;
	iBacklightPercent = percent;
	setBK_imp(iBacklightPercent);
}
int getBacklight()
{
	return iBacklightPercent;
}

void setBacklightON()
{

	setBK_imp(iBacklightPercent);

	//HAL_GPIO_WritePin(LCD_BL_GPIO_Port,LCD_BL_Pin,GPIO_PIN_SET);
  //~ TIM4->CCR1 = 333;
}

void setBacklightOff()
{
	setBK_imp(0);
  //~ TIM4->CCR1 = 0;
	//HAL_GPIO_WritePin(LCD_BL_GPIO_Port,LCD_BL_Pin,GPIO_PIN_RESET);
}

void ili9341_DisplayOn(void)
{
  /* Display On */
  setBacklight(200);
  FMC_BANK1_WriteComand(LCD_DISPLAY_ON);
}

/**
  * @brief  Disables the Display.
  * @param  None
  * @retval None
  */
void ili9341_DisplayOff(void)
{
  /* Display Off */
  setBK_imp(0);
  FMC_BANK1_WriteComand(LCD_DISPLAY_OFF);
}

void ili9341_WriteData(uint8_t RegValue)
{
	FMC_BANK1_WriteData(RegValue);
}
volatile int bComplete = 1;
extern DMA_HandleTypeDef hdma_memtomem_dma2_stream3;
void MyDMAComplete(DMA_HandleTypeDef* hdma)
{
	bComplete = 1;
}
#ifdef ILI9341
int16_t ttab[16][2][2];
#else
int16_t ttab[16][3][3];
#endif

void make_ttab()
{
#ifdef ILI9341
	for(int k=0;k<16;k++)
	{
		ttab[k][0][0] = (k >>3)&1;
		ttab[k][0][1] = (k >>2)&1;
		ttab[k][1][0] = (k >>1)&1;
		ttab[k][1][1] = (k >>0)&1;
	}
#else
	for(int k=0;k<16;k++)
	{
		ttab[k][0][0] = (k >>2)&1;
		ttab[k][0][2] = (k >>3)&1;
		ttab[k][2][0] = (k >>0)&1;
		ttab[k][2][2] = (k >>1)&1;

		if(ttab[k][0][0]==ttab[k][0][2])
		{
			ttab[k][0][1] = ttab[k][0][0];
		}
		else
			ttab[k][0][1] = 2;

		if(ttab[k][2][0]==ttab[k][2][2])
		{
			ttab[k][2][1] = ttab[k][2][0];
		}
		else
			ttab[k][2][1] = 2;

		if(ttab[k][0][0]==ttab[k][2][0])
		{
			ttab[k][1][0] = ttab[k][2][0];
		}
		else
			ttab[k][1][0] = 2;

		if(ttab[k][0][2]==ttab[k][2][2])
		{
			ttab[k][1][2] = ttab[k][2][2];
		}
		else
			ttab[k][1][2] = 2;

		if(ttab[k][0][0]==ttab[k][2][2])
		{
			ttab[k][1][1] = ttab[k][0][0];
		}
		else
			ttab[k][1][1] = 2;
	}
#endif
}

void ili9341_Init(void)
{
	HAL_DMA_RegisterCallback(&hdma_memtomem_dma2_stream3,HAL_DMA_XFER_CPLT_CB_ID,MyDMAComplete);
	bComplete = 1;
	make_ttab();
	initTft();
	//HAL_GPIO_WritePin(LCD_BL_GPIO_Port,LCD_BL_Pin,GPIO_PIN_SET);
}
#if 0
void LCD_WR_REG(uint8_t data)
{
	FMC_BANK1_WriteComand(data);
}
void LCD_WR_DATA(uint8_t data)
{
	FMC_BANK1_WriteData(data);
}
void LCD_Init(void)
{
	//LCD_GPIOInit();//LCD GPIO³õÊ¼»¯
	//delay_ms(100);
//************* ILI9486³õÊ¼»¯**********//
	LCD_WR_REG(0XF1);
	LCD_WR_DATA(0x36);
	LCD_WR_DATA(0x04);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x3C);
	LCD_WR_DATA(0X0F);
	LCD_WR_DATA(0x8F);
	LCD_WR_REG(0XF2);
	LCD_WR_DATA(0x18);
	LCD_WR_DATA(0xA3);
	LCD_WR_DATA(0x12);
	LCD_WR_DATA(0x02);
	LCD_WR_DATA(0XB2);
	LCD_WR_DATA(0x12);
	LCD_WR_DATA(0xFF);
	LCD_WR_DATA(0x10);
	LCD_WR_DATA(0x00);
	LCD_WR_REG(0XF8);
	LCD_WR_DATA(0x21);
	LCD_WR_DATA(0x04);
	LCD_WR_REG(0XF9);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x08);
	LCD_WR_REG(0x36);
	LCD_WR_DATA(0x08);
	LCD_WR_REG(0xB4);
	LCD_WR_DATA(0x00);
	LCD_WR_REG(0xC1);
	LCD_WR_DATA(0x41);
	LCD_WR_REG(0xC5);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x91);
	LCD_WR_DATA(0x80);
	LCD_WR_DATA(0x00);
	LCD_WR_REG(0xE0);
	LCD_WR_DATA(0x0F);
	LCD_WR_DATA(0x1F);
	LCD_WR_DATA(0x1C);
	LCD_WR_DATA(0x0C);
	LCD_WR_DATA(0x0F);
	LCD_WR_DATA(0x08);
	LCD_WR_DATA(0x48);
	LCD_WR_DATA(0x98);
	LCD_WR_DATA(0x37);
	LCD_WR_DATA(0x0A);
	LCD_WR_DATA(0x13);
	LCD_WR_DATA(0x04);
	LCD_WR_DATA(0x11);
	LCD_WR_DATA(0x0D);
	LCD_WR_DATA(0x00);
	LCD_WR_REG(0xE1);
	LCD_WR_DATA(0x0F);
	LCD_WR_DATA(0x32);
	LCD_WR_DATA(0x2E);
	LCD_WR_DATA(0x0B);
	LCD_WR_DATA(0x0D);
	LCD_WR_DATA(0x05);
	LCD_WR_DATA(0x47);
	LCD_WR_DATA(0x75);
	LCD_WR_DATA(0x37);
	LCD_WR_DATA(0x06);
	LCD_WR_DATA(0x10);
	LCD_WR_DATA(0x03);
	LCD_WR_DATA(0x24);
	LCD_WR_DATA(0x20);
	LCD_WR_DATA(0x00);
	LCD_WR_REG(0x3A);
	LCD_WR_DATA(0x55);
	LCD_WR_REG(0x11);
	LCD_WR_REG(0x36);
	LCD_WR_DATA(0x28);
	Delay(120);
	LCD_WR_REG(0x29);

//  LCD_direction(USE_HORIZONTAL);//ÉèÖÃLCDÏÔÊ¾·½Ïò
//	LCD_LED=1;//µãÁÁ±³¹â
//	LCD_Clear(WHITE);//ÇåÈ«ÆÁ°×É«
}
#endif
    void initTft()
    {
  tftReset();

  FMC_BANK1_WriteComand(LCD_SWRESET);
  FMC_BANK1_WriteComand(LCD_DISPLAY_OFF);
  
  Delay(25);
  
  FMC_BANK1_WriteComand(LCD_POWERB);
  FMC_BANK1_WriteData(0x00);
  FMC_BANK1_WriteData(0x83);
  FMC_BANK1_WriteData(0x30);

  FMC_BANK1_WriteComand(LCD_POWER_SEQ);
  FMC_BANK1_WriteData(0x64);
  FMC_BANK1_WriteData(0x03);
  FMC_BANK1_WriteData(0x12);
  FMC_BANK1_WriteData(0x81);

  FMC_BANK1_WriteComand(LCD_DTCA);
  FMC_BANK1_WriteData(0x85);
  FMC_BANK1_WriteData(0x01);
  FMC_BANK1_WriteData(0x79);

  FMC_BANK1_WriteComand(LCD_POWERA);
  FMC_BANK1_WriteData(0x39);
  FMC_BANK1_WriteData(0x2c);
  FMC_BANK1_WriteData(0x00);
  FMC_BANK1_WriteData(0x34);
  FMC_BANK1_WriteData(0x02);

  FMC_BANK1_WriteComand(LCD_PRC);
  FMC_BANK1_WriteData(0x20);

  FMC_BANK1_WriteComand(LCD_DTCB);
  FMC_BANK1_WriteData(0x00);
  FMC_BANK1_WriteData(0x00);


  FMC_BANK1_WriteComand(LCD_POWER1);
  FMC_BANK1_WriteData(0x26);

  FMC_BANK1_WriteComand(LCD_POWER2);
  FMC_BANK1_WriteData(0x0011);

  FMC_BANK1_WriteComand(LCD_VCOM1);
  FMC_BANK1_WriteData(0x35);
  FMC_BANK1_WriteData(0x3e);

  FMC_BANK1_WriteComand(LCD_VCOM2);
  FMC_BANK1_WriteData(0xbe);

  FMC_BANK1_WriteComand(LCD_MAC);
  FMC_BANK1_WriteData(0x48);//48
  FMC_BANK1_WriteComand(LCD_PIXEL_FORMAT);

#ifdef COLOR_3BYTES
  FMC_BANK1_WriteData(0x66);
#else
  FMC_BANK1_WriteData(0x55);
#endif

  FMC_BANK1_WriteComand(LCD_FRMCTR1);
  FMC_BANK1_WriteData(0x80);
  FMC_BANK1_WriteData(0x10); // default 0x1B
  //FMC_BANK1_WriteData(0x0010); // default 0x1B
  //FMC_BANK1_WriteData(0x0030); // default 0x1B

  FMC_BANK1_WriteComand(LCD_3GAMMA_EN);
  FMC_BANK1_WriteData(0x08);

  FMC_BANK1_WriteComand(LCD_GAMMA);
  FMC_BANK1_WriteData(0x01);

  FMC_BANK1_WriteComand(TFT_REG_GAMMA_1);
  FMC_BANK1_WriteData(0x1f);
  FMC_BANK1_WriteData(0x1a);
  FMC_BANK1_WriteData(0x18);
  FMC_BANK1_WriteData(0x0a);
  FMC_BANK1_WriteData(0x0f);
  FMC_BANK1_WriteData(0x06);
  FMC_BANK1_WriteData(0x45);
  FMC_BANK1_WriteData(0x87);
  FMC_BANK1_WriteData(0x32);
  FMC_BANK1_WriteData(0x0a);
  FMC_BANK1_WriteData(0x07);
  FMC_BANK1_WriteData(0x02);
  FMC_BANK1_WriteData(0x07);
  FMC_BANK1_WriteData(0x05);
  FMC_BANK1_WriteData(0x00);

  FMC_BANK1_WriteComand(TFT_REG_GAMMA_2);
  FMC_BANK1_WriteData(0x00);
  FMC_BANK1_WriteData(0x25);
  FMC_BANK1_WriteData(0x27);
  FMC_BANK1_WriteData(0x05);
  FMC_BANK1_WriteData(0x10);
  FMC_BANK1_WriteData(0x09);
  FMC_BANK1_WriteData(0x3a);
  FMC_BANK1_WriteData(0x78);
  FMC_BANK1_WriteData(0x4d);
  FMC_BANK1_WriteData(0x05);
  FMC_BANK1_WriteData(0x18);
  FMC_BANK1_WriteData(0x0d);
  FMC_BANK1_WriteData(0x38);
  FMC_BANK1_WriteData(0x3a);
  FMC_BANK1_WriteData(0x1f);

//  FMC_BANK1_WriteComand(TFT_REG_COL);
//  FMC_BANK1_WriteData(0x00);
//  FMC_BANK1_WriteData(0x00);
//  FMC_BANK1_WriteData(0x00);
//  FMC_BANK1_WriteData(0xef);
//
//  FMC_BANK1_WriteComand(TFT_REG_PAGE);
//  FMC_BANK1_WriteData(0x00);
//  FMC_BANK1_WriteData(0x00);
//  FMC_BANK1_WriteData(0x01);
//  FMC_BANK1_WriteData(0x3f);

  FMC_BANK1_WriteComand(TFT_REG_TE_OFF);
  FMC_BANK1_WriteData(0x00);

  FMC_BANK1_WriteComand(LCD_ETMOD);
  FMC_BANK1_WriteData(0x07);

  /*
   *
  FMC_BANK1_WriteComand(LCD_DFC);
  FMC_BANK1_WriteData(0x0a);
  FMC_BANK1_WriteData(0x82);
  FMC_BANK1_WriteData(0x27);
  FMC_BANK1_WriteData(0x00);
*/
  //FMC_BANK1_WriteComand(LCD_WCD);
  //FMC_BANK1_WriteData(0b00101100);

  FMC_BANK1_WriteComand(LCD_SLEEP_OUT);

  Delay(100);

  FMC_BANK1_WriteComand(LCD_DISPLAY_ON);
  
  Delay(100);

  FMC_BANK1_WriteComand(TFT_REG_MEM_WRITE);
  setBacklight(50);

  //LCD_Init();
  FMC_BANK1_WriteComand(LCD_PIXEL_FORMAT);

#ifdef COLOR_3BYTES
  FMC_BANK1_WriteData(0x66);
#else
  FMC_BANK1_WriteData(0x55);
#endif

  //FMC_BANK1_WriteComand(0x34);
  //FMC_BANK1_WriteComand(0x35);
  //FMC_BANK1_WriteData(0x0);

}
#if 0
    void LCD_WriteReg(uint8_t cmd,uint8_t data)
    {
    	FMC_BANK1_WriteComand(cmd);
    	FMC_BANK1_WriteData(data);

    }

    void LCD_direction(uint8_t direction)
    {
    			//lcddev.setxcmd=0x2A;
    			//lcddev.setycmd=0x2B;
    			//lcddev.wramcmd=0x2C;
    			//lcddev.rramcmd=0x2E;
    	switch(direction){
    		case 0:
    			//lcddev.width=LCD_W;
    			//lcddev.height=LCD_H;
    			LCD_WriteReg(0x36,(1<<6)|(1<<3));//0 degree MY=0,MX=0,MV=0,ML=0,BGR=1,MH=0
    		break;
    		case 1:
    			//lcddev.width=LCD_H;
    			//lcddev.height=LCD_W;
    			LCD_WriteReg(0x36,(1<<3)|(1<<4)|(1<<5));//90 degree MY=0,MX=1,MV=1,ML=1,BGR=1,MH=0
    		break;
    		case 2:
    			//lcddev.width=LCD_W;
    			//lcddev.height=LCD_H;
    			LCD_WriteReg(0x36,(1<<3)|(1<<7));//180 degree MY=1,MX=1,MV=0,ML=0,BGR=1,MH=0
    		break;
    		case 3:
    			//lcddev.width=LCD_H;
    			//lcddev.height=LCD_W;
    			LCD_WriteReg(0x36,(1<<3)|(1<<5)|(1<<6)|(1<<7));//270 degree MY=1,MX=0,MV=1,ML=0,BGR=1,MH=0
    		break;
    		default:break;
    	}
    }


#endif
void sendData(uint8_t data)
{
	FMC_BANK1_WriteData(data);
}
void sendData16(uint16_t data)
{
	FMC_BANK1_WriteData((data>>8u)&0xff);
	FMC_BANK1_WriteData(data&0xff);
}
void tftReset()
{

  HAL_GPIO_WritePin(LCD_RESET_GPIO_Port,LCD_RESET_Pin,GPIO_PIN_RESET);
  Delay(200);
  HAL_GPIO_WritePin(LCD_RESET_GPIO_Port,LCD_RESET_Pin,GPIO_PIN_SET);
  Delay(200);
 // LCD_RD_1;
 // LCD_CS_0;

	printf("go reset\n");
  Delay(200);
	printf("delay ok \n");
  FMC_BANK1_WriteComand(LCD_SWRESET);
  printf("writecomm ok \n");
  Delay(200);
  printf("reset\n");
}

void enterSleep()
{
  FMC_BANK1_WriteComand(0x28);
  Delay(20);
  FMC_BANK1_WriteComand(0x10);
}

void exitSleep()
{
  FMC_BANK1_WriteComand(0x11);
  Delay(120);
  FMC_BANK1_WriteComand(0x29);
}

/*
void setCol(uint16_t startX, uint16_t startY)
{
  FMC_BANK1_WriteComand(0x2A);
  FMC_BANK1_WriteData(startX);
  FMC_BANK1_WriteData(startY);
}

void setPage(uint16_t endX, uint16_t endY)
{
  FMC_BANK1_WriteComand(0x2B);
  FMC_BANK1_WriteData(endX);
  FMC_BANK1_WriteData(endY);
}

void setXY(uint16_t poX, uint16_t poY)
{
  //setCol(poX, poX);
  setPage(poY, poY);
  FMC_BANK1_WriteComand(TFT_REG_MEM_WRITE);
}
*/
int flagReadOK = 0;

void setWindow(uint16_t startX, uint16_t endX, uint16_t startY, uint16_t endY)
 {
    /*
    FMC_BANK1_WriteComand(0x2a);
    
    FMC_BANK1_WriteData((startX >> 8) & 0xFF);
    FMC_BANK1_WriteData(startX & 0xFF);
    FMC_BANK1_WriteData((endX >> 8) & 0xFF);
    FMC_BANK1_WriteData(startY & 0xFF);
  
    FMC_BANK1_WriteComand(0x2b);
    FMC_BANK1_WriteData((startY >> 8) & 0xFF);
    FMC_BANK1_WriteData(endX & 0xFF);
    FMC_BANK1_WriteData((endY >> 8) & 0xFF);
    FMC_BANK1_WriteData(endY >> 8);
    */
#ifdef ILI9341
	FMC_BANK1_WriteComand(TFT_REG_COL);
	FMC_BANK1_WriteData(startX >> 8u);
	FMC_BANK1_WriteData(startX & 0x00FF);
	FMC_BANK1_WriteData(endX >> 8u);
	FMC_BANK1_WriteData(endX& 0x00FF);
	FMC_BANK1_WriteComand(TFT_REG_PAGE);
    FMC_BANK1_WriteData(startY >> 8u);
    FMC_BANK1_WriteData(startY& 0x00FF);
    FMC_BANK1_WriteData(endY >> 8u);
    FMC_BANK1_WriteData(endY& 0x00FF);
#else
    //startX = ILI9341_LCD_PIXEL_HEIGHT-1 - startX;
    //endX = ILI9341_LCD_PIXEL_HEIGHT-1 - endX;
    if(flagReadOK)
    {
	FMC_BANK1_WriteComand(TFT_REG_COL);
	FMC_BANK1_WriteData(startX >> 8u);
	FMC_BANK1_WriteData(startX & 0x00FF);
	FMC_BANK1_WriteData(endX >> 8u);
	FMC_BANK1_WriteData(endX& 0x00FF);
	FMC_BANK1_WriteComand(TFT_REG_PAGE);
    FMC_BANK1_WriteData((startY >> 8u)& 0x00FF);
    FMC_BANK1_WriteData(startY& 0x00FF);
    FMC_BANK1_WriteData((endY >> 8u)& 0x00FF);
    FMC_BANK1_WriteData(endY& 0x00FF);
    }
    else
    {
    	uint16_t nstartX = ILI9341_LCD_PIXEL_HEIGHT-1 - endX;
    	uint16_t nendX = ILI9341_LCD_PIXEL_HEIGHT-1 - startX;
    	FMC_BANK1_WriteComand(TFT_REG_COL);
    	FMC_BANK1_WriteData(nstartX >> 8u);
    	FMC_BANK1_WriteData(nstartX & 0x00FF);
    	FMC_BANK1_WriteData(nendX >> 8u);
    	FMC_BANK1_WriteData(nendX& 0x00FF);
    	FMC_BANK1_WriteComand(TFT_REG_PAGE);
        FMC_BANK1_WriteData((startY >> 8u)& 0x00FF);
        FMC_BANK1_WriteData(startY& 0x00FF);
        FMC_BANK1_WriteData((endY >> 8u)& 0x00FF);
        FMC_BANK1_WriteData(endY& 0x00FF);

    }
#endif

 }
#if 0
void fillRectangle(uint16_t poX, uint16_t poY,
                                uint16_t width, uint16_t length,
                                uint16_t color)
{
  uint32_t windowSize = length * width;
  uint32_t count = 0;

  setWindow(poX, (poX + width), poY, (poY + length));
  FMC_BANK1_WriteComand(TFT_REG_MEM_WRITE);

  /*for( uint32_t i = 0; i <= windowSize; i++ )
  {
    sendData(color);
    count++;
  }
  sendData(color);  */
  int i = 0;
  int j = 0;
  for( i = 0; i < length; i++ )
  {
    for( j = 0; j < width + 100; j++ )
    {
      sendData(color);
    }
  }
}
#endif
void color_convert(uint16_t color,uint8_t* result)
{
	//~ #define RED             0xF800
	//~ #define BLUE            0x001F
	//~ #define GREEN           0x07E0
	result[2]=  ((color&0x1f)		<<(1+2))<<1;//|0x80;    //5 bit BLUE
	result[1]=  (((color>>5)&0x3f) <<(0+1))<<1;//|0x80;    //6 bit GREEN
	result[0]=  (((color>>11)&0x1f)<<(1+2))<<1;//|0x80;    //5 bit  //RED
}

uint16_t color_convertRGB_to16(uint8_t * adress)
{
	return ((adress[0]>>3)<<11)|((adress[1]>>2)<<5)|(adress[2]>>3);
}

void waitScreenComplete()
{
	while(!bComplete)
	{

	}
}
void LCD_fillRect(uint16_t x1, uint16_t y1, uint16_t w, uint16_t h, uint16_t color)
{
	int dw = w*h;
	waitScreenComplete();
	setWindow(x1, x1+w-1,y1,( y1+h-1));
	//setWindow(ILI9341_LCD_PIXEL_WIDTH-1-(x1+w-1),ILI9341_LCD_PIXEL_WIDTH-1-x1,  y1, y1+h-1);
	//setWindow(x1, x1+w-1,ILI9341_LCD_PIXEL_HEIGHT-1-( y1+h-1),ILI9341_LCD_PIXEL_HEIGHT-1-y1);


	//setWindow(ILI9341_LCD_PIXEL_HEIGHT-1-(x1+w-1),ILI9341_LCD_PIXEL_HEIGHT-1-x1 ,y1,( y1+h-1));

    FMC_BANK1_WriteComand(TFT_REG_MEM_WRITE);

	int k;
	uint8_t  Data0 = (color>>8u)&0xff;
	uint8_t  Data1 = (color)&0xff;
#if 0
#define BSIZ 128
    if(dw>=40)
	{
		uint8_t dtt[BSIZ*2];
		int fn = dw>BSIZ?BSIZ:dw;
		for(int k=0;k<fn;k++)
		{
			dtt[k*2] = Data0;
			dtt[k*2+1] = Data1;
		}
		for(k=dw;k>0;k -= BSIZ)
		{
			waitScreenComplete();
			bComplete = 0;
			HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream3,dtt, &LCD_RAM,k>=BSIZ?(BSIZ*2):(k*2));
		}

	}
    else
#endif
    {
		for(k=0;k<dw;k++)
		{
			LCD_RAM = Data0;
			LCD_RAM = Data1;
		}
    }
}
void LCD_Write8x8line(uint16_t x1, uint16_t y1,uint8_t * adress)
{
	setWindow(x1, (uint16_t) (x1+8-1),y1,(uint16_t) (y1+8-1));
    FMC_BANK1_WriteComand(TFT_REG_MEM_WRITE);
	//uint16_t ncolor;
	int k;
	for(k=0;k<8*8;k++)
	{
		uint16_t color = color_convertRGB_to16(adress);
		sendData16(color);
		adress+=3;
	}
}

void LCD_Write8x8line16(uint16_t x1, uint16_t y1,uint16_t * adress)
{
	setWindow(x1, (uint16_t) (x1+8-1),y1,(uint16_t) (y1+8-1));
    FMC_BANK1_WriteComand(TFT_REG_MEM_WRITE);
	//uint16_t ncolor;
	int k;
	for(k=0;k<8*8;k++)
	{
		uint8_t  Data0 = (adress[k]>>8u)&0xff;
		uint8_t  Data1 = (adress[k])&0xff;
		LCD_RAM = Data0;
		LCD_RAM = Data1;
	}
}
//const int indt[] = {0,2,3,5,6,8,9,11};
const int indt12[] = {0,1,2,4,5,6,8,9,10,12,13,14};
const int indt12h[] = {0,0,1,2,2,3,4,4,5,6,6,7};
void makeBlock12(uint16_t * colors,uint8_t * bytes,uint16_t*  block12,int stride)
{
	int k;
	for(int k=0;k<3;k++)
	{
		colors[k] = __REV16(colors[k]);
	}
	for(int yy=0;yy<4;yy++)
	{
		for(int xx=0;xx<4;xx++)
		{
			int16_t rnum = (((bytes[yy*2]>>(xx*2))&3)<<2)|((bytes[yy*2+1]>>(xx*2))&3);
			for(int y=0;y<3;y++)
			{
				uint16_t*  block12p = &block12[(yy*3+y)*stride+xx*3];
				for(int x=0;x<3;x++)
				{
					block12p[x] = colors[ttab[rnum][y][x]];
				}
			}
		}
	}
}
void makeBlock8(uint16_t * colors,uint8_t * bytes,uint16_t*  block8,int stride)
{
	int k;
	for(int k=0;k<3;k++)
	{
		colors[k] = __REV16(colors[k]);
	}
	for(int yy=0;yy<4;yy++)
	{
		for(int xx=0;xx<4;xx++)
		{
			int16_t rnum = (((bytes[yy*2]>>((3-xx)*2))&3)<<2)|((bytes[yy*2+1]>>((3-xx)*2))&3);
			for(int y=0;y<2;y++)
			{
				uint16_t*  block8p = &block8[(yy*2+y)*stride+xx*2];
				for(int x=0;x<2;x++)
				{
					block8p[x] = colors[ttab[rnum][y][x]];
				}
			}
		}
	}
}

void makeBlock12a(uint16_t * colors,uint8_t * bytes,uint16_t*  block12,int stride)
{
	int k;
	for(int k=0;k<3;k++)
	{
		colors[k] = __REV16(colors[k]);
	}
	for(int ny=0;ny<12;ny++)
	{
		int    y  = indt12h[ny];//(ny*2)/3;//;
		uint16_t*  block12p = &block12[ny*stride];
		for(int nx=0;nx<12;nx++)
		{
			int x        = indt12h[nx];//(nx*2)/3;//;
			int bt0      = (bytes[y]>>x)&1;
			block12p[nx] = bt0?colors[1]:colors[0];
		}
	}
#if 0
	for(int ny=0;ny<12;ny++)
	{
		int    y  = indt12[ny]/2;
		uint16_t*  block12p = &block12[ny*stride];
		if(indt12[ny]&1)
		{
			for(int nx=0;nx<12;nx++)
			{
				int x        = indt12[nx]/2;
				int bt0      = (bytes[y]>>x)&1;
				block12p[nx] = bt0?colors[1]:colors[0];
				if(indt12[nx]&1)
				{
					if(bt0!=((bytes[y+1]>>(x+1))&1))
					{
						block12p[nx] = colors[2];
					}
				}
				else
				{
					if(bt0!=((bytes[y+1]>>x)&1))
					{
						block12p[nx] = colors[2];
					}
				}
			}
		}
		else
		{
			for(int nx=0;nx<12;nx++)
			{
				int x 		=  indt12[nx]/2;
				int bt0 	=  (bytes[y]>>x)&1;
				block12p[nx]=  bt0?colors[1]:colors[0];
				if(indt12[nx]&1)
				{
					if(bt0!=((bytes[y]>>(x+1))&1))
					{
						block12p[nx] = colors[2];
					}
				}
			}
		}
	}
#endif
}
void LCD_Write8x8line16_2_12_32(uint16_t x1, uint16_t y1,uint16_t*  block12)
{
	int x1n = x1+x1/2;
	int y1n = y1+y1/2;
	while(!bComplete)
	{

	}
	setWindow(x1n, (uint16_t) (x1n+12*32-1),y1n,(uint16_t) (y1n+12-1));
    FMC_BANK1_WriteComand(TFT_REG_MEM_WRITE);
	bComplete = 0;
	HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream3, &block12[0], &LCD_RAM,12*12*2*32);
}
void LCD_Write8x8line16_2_8_32(uint16_t x1, uint16_t y1,uint16_t*  block8)
{
	int x1n = x1;
	int y1n = y1;
	while(!bComplete)
	{

	}
	setWindow(x1n, (uint16_t) (x1n+8*32-1),y1n,(uint16_t) (y1n+8-1));
    FMC_BANK1_WriteComand(TFT_REG_MEM_WRITE);
	bComplete = 0;
	HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream3, &block8[0], &LCD_RAM,8*8*2*32);
}

void LCD_Write8x8line16_2_12(uint16_t x1, uint16_t y1,uint16_t*  block12)
{
	int x1n = x1+x1/2;
	int y1n = y1+y1/2;
	while(!bComplete)
	{

	}
	setWindow(x1n, (uint16_t) (x1n+12-1),y1n,(uint16_t) (y1n+12-1));
    FMC_BANK1_WriteComand(TFT_REG_MEM_WRITE);

    //makeBlock12(colors,bytes,block12);

	bComplete = 0;
	HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream3, &block12[0], &LCD_RAM,12*12*2);
	/*
	for(int y=0;y<12;y++)
	{
		for(int x=0;x<12;x++)
		{
			uint16_t color = block12[y][x];
			uint8_t  Data0 = (color>>8u)&0xff;
			uint8_t  Data1 = (color)&0xff;
			LCD_RAM = Data0;
			LCD_RAM = Data1;
		}
	}
	*/
}
void LCD_Write8x8line16_2(uint16_t x1, uint16_t y1,uint16_t * colors,uint8_t * bytes)
{
	setWindow(x1, (uint16_t) (x1+8-1),y1,(uint16_t) (y1+8-1));
    FMC_BANK1_WriteComand(TFT_REG_MEM_WRITE);
	//uint16_t ncolor;
	int k;
	for(int y=0;y<8;y++)
	{
		for(int x=0;x<8;x++)
		{
			uint16_t color = ((bytes[y]>>x)&1)?colors[1]:colors[0];
			uint8_t  Data0 = (color>>8u)&0xff;
			uint8_t  Data1 = (color)&0xff;
			LCD_RAM = Data0;
			LCD_RAM = Data1;
		}
	}
}


void LCD_Read8x8line16(uint16_t x1, uint16_t y1,uint16_t * adress)
{
	setWindow(x1, (uint16_t) (x1+8-1),y1,(uint16_t) (y1+8-1));
    FMC_BANK1_WriteComand(LCD_RAMRD);
    FMC_BANK1_ReadDataN16(adress,8*8);
}

void LCD_FullRect3(uint16_t x1, uint16_t y1,uint8_t * adress,uint16_t w,uint16_t h)
{
	int dw = w*h;
	setWindow(x1, x1+w-1, y1, y1+h-1);
    FMC_BANK1_WriteComand(TFT_REG_MEM_WRITE);

	int k;
	 uint16_t color = color_convertRGB_to16(adress);
	for(k=0;k<dw;k++)
	{
		sendData16(color);
	}

}
uint32_t ili9341_ReadID()
{
	FMC_BANK1_WriteComand(LCD_READ_DISPLAY_ID);
	return FMC_BANK1_ReadData4();
}
uint32_t ili9341_ReadID4(void)
{
	FMC_BANK1_WriteComand(LCD_READ_ID4);
	return FMC_BANK1_ReadData4();
  //~ LCD_IO_Init();
}

uint32_t ili9341_ReadID3(void)
{
  //~ LCD_IO_Init();
	FMC_BANK1_WriteComand(LCD_READ_ID3);
	return FMC_BANK1_ReadData();
}
uint32_t ili9341_ReadID2(void)
{
  //~ LCD_IO_Init();
	FMC_BANK1_WriteComand(LCD_READ_ID2);
	return FMC_BANK1_ReadData();
}
uint32_t ili9341_ReadID1(void)
{
  //~ LCD_IO_Init();
	FMC_BANK1_WriteComand(LCD_READ_ID1);
	return FMC_BANK1_ReadData();
}

static uint16_t screen_width  = ILI9341_LCD_PIXEL_WIDTH;
static uint16_t screen_height = ILI9341_LCD_PIXEL_HEIGHT;
//#define ILI9341
#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04
#define MADCTL_FH  0x02
#define MADCTL_FV  0x01
#ifdef ILI9341a
void LCD_setRotation(uint8_t rotation)
{
    screen_height = ILI9341_LCD_PIXEL_HEIGHT;
    screen_width  = ILI9341_LCD_PIXEL_WIDTH;
	uint8_t madctl = 0;
	switch (rotation&0x3) {
	  case PORTRAIT:
		madctl = (MADCTL_MX | MADCTL_BGR);
		break;
	  case LANDSCAPE:
		madctl = (MADCTL_MV | MADCTL_BGR);
        screen_height = ILI9341_LCD_PIXEL_WIDTH;
        screen_width  = ILI9341_LCD_PIXEL_HEIGHT;
		break;
	  case PORTRAIT_FLIP:
		madctl = (MADCTL_MY | MADCTL_BGR);
		break;
	  case LANDSCAPE_FLIP:
		madctl = (MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
		screen_height = ILI9341_LCD_PIXEL_WIDTH;
        screen_width  = ILI9341_LCD_PIXEL_HEIGHT;
		break;
	}
	  FMC_BANK1_WriteComand(LCD_MAC);
	  ili9341_WriteData(madctl);
//    TFT_CS_RESET;
//    dmaSendCmdCont(LCD_MAC);
//    dmaSendDataCont8(&madctl, 1);
//    TFT_CS_SET;
}

#else
#ifdef ILI9341
void LCD_setRotation(uint8_t rotation)
{
    screen_height = ILI9341_LCD_PIXEL_HEIGHT;
    screen_width  = ILI9341_LCD_PIXEL_WIDTH;
	uint8_t madctl = 0;
	switch (rotation&0x3) {
	  case PORTRAIT:
		madctl = (MADCTL_MX |MADCTL_MY| MADCTL_BGR);
		break;
	  case LANDSCAPE:
		madctl = (MADCTL_MV | MADCTL_BGR | MADCTL_MY);
        screen_height = ILI9341_LCD_PIXEL_WIDTH;
        screen_width  = ILI9341_LCD_PIXEL_HEIGHT;
		break;
	  case PORTRAIT_FLIP:
		madctl = (MADCTL_BGR );
		break;
	  case LANDSCAPE_FLIP:
		madctl = (MADCTL_MX | MADCTL_MV | MADCTL_BGR);
		screen_height = ILI9341_LCD_PIXEL_WIDTH;
        screen_width  = ILI9341_LCD_PIXEL_HEIGHT;
		break;
	}
	  FMC_BANK1_WriteComand(LCD_MAC);
	  FMC_BANK1_WriteData(madctl);
	  //FMC_BANK1_WriteComand(0xb0);
	  //FMC_BANK1_WriteData(0);
	  //LCD_direction(2);
	  //ili9341_WriteData(0x28);
//    TFT_CS_RESET;
//    dmaSendCmdCont(LCD_MAC);
//    dmaSendDataCont8(&madctl, 1);
//    TFT_CS_SET;
}
#else
//#define MADCTL_MY  0x80
//#define MADCTL_MX  0x40
//#define MADCTL_MV  0x20
//#define MADCTL_ML  0x10
//#define MADCTL_RGB 0x00
//#define MADCTL_BGR 0x08
//#define MADCTL_MH  0x04
//#define MADCTL_FH  0x02
//#define MADCTL_FV  0x01
void LCD_setRotation(uint8_t rotation)
{
    screen_height = ILI9341_LCD_PIXEL_HEIGHT;
    screen_width  = ILI9341_LCD_PIXEL_WIDTH;
	uint8_t madctl = 0;
	switch (rotation&0x3) {
	  case PORTRAIT:
		madctl = (MADCTL_MX |MADCTL_MY| MADCTL_BGR | MADCTL_ML);
		break;
	  case LANDSCAPE:
		madctl = (MADCTL_MV | MADCTL_BGR | MADCTL_MY);
        screen_height = ILI9341_LCD_PIXEL_WIDTH;
        screen_width  = ILI9341_LCD_PIXEL_HEIGHT;
		break;
	  case PORTRAIT_FLIP:
		madctl = (MADCTL_BGR | MADCTL_ML);
//		madctl = (MADCTL_BGR| MADCTL_MX );
//		madctl = (MADCTL_BGR|  MADCTL_MY );
//		madctl = (MADCTL_BGR|  MADCTL_MY | MADCTL_MX );
		break;
	  case LANDSCAPE_FLIP:
		madctl = (MADCTL_MX | MADCTL_MV | MADCTL_BGR);
		screen_height = ILI9341_LCD_PIXEL_WIDTH;
        screen_width  = ILI9341_LCD_PIXEL_HEIGHT;
		break;
	}
	  FMC_BANK1_WriteComand(LCD_MAC);
	  FMC_BANK1_WriteData(madctl);
	  //FMC_BANK1_WriteComand(0xb0);
	  //FMC_BANK1_WriteData(0);
	  //LCD_direction(2);
	  //ili9341_WriteData(0x28);
//    TFT_CS_RESET;
//    dmaSendCmdCont(LCD_MAC);
//    dmaSendDataCont8(&madctl, 1);
//    TFT_CS_SET;
}
#endif
#endif
uint16_t LCD_getWidth() {
    return screen_width;
}

uint16_t LCD_getHeight() {
    return screen_height;
}


//ID=e30000
//ID4=9340

//ID=4000000
//ID4=d30090d3

void LCD_init()
{
	tftReset();
	uint32_t ID;
	ID = ili9341_ReadID4();
	printf("ID4=%x\n",ID);

	ID = ili9341_ReadID();
	printf("ID=%x\n",ID);

	ili9341_Init();
#ifdef ILI9341
#else
	flagReadOK = 1;
    if((ID & 0x00ffffffu)==0x548066u)
    {
    	flagReadOK = 1;
    }
#endif
	ID = ili9341_ReadID4();
	printf("ID4=%x\n",ID);

	ID = ili9341_ReadID();
	printf("ID=%x\n",ID);

	ID = ili9341_ReadID1();
	printf("ID1=%x\n",ID);
	ID = ili9341_ReadID2();
	printf("ID2=%x\n",ID);
	ID = ili9341_ReadID3();
	printf("ID3=%x\n",ID);
}

/*
void LCD_Draw_Char(char Character, int16_t X, int16_t Y, uint16_t Colour, uint16_t Size, uint16_t Background_Colour)
{
	//~ flagReinit = 1;
		uint8_t 	function_char;
    int16_t 	i,j;

	function_char = Character;

        if (function_char < 0x20)
	{
		Character = 0;
	}
	else
	{
		function_char -= 32;
	}

	char temp[CHAR_WIDTH];
	uint8_t k;
	for( k = 0; k<CHAR_WIDTH; k++)
	{
		temp[k] = font[function_char*CHAR_WIDTH+k];
	}

    // Draw pixels
		LCD_fillRect(X, Y, CHAR_WIDTH*Size, CHAR_HEIGHT*Size, Background_Colour);
    for (j=0; j<CHAR_WIDTH; j++) {
        for (i=0; i<CHAR_HEIGHT; i++) {
            if (temp[i] & (1<<(CHAR_WIDTH-1-j))) {
								//~ LCD_fillRect(X+(j*Size), Y+(i*Size), Size,Size, Colour);
							LCD_fillRect(X+(j*Size), Y+(i*Size), Size,Size, Colour);
            }
        }
    }
}*/
#include "fonts.h"
#if 0

void LCD_Draw_Char2(char Character, int16_t X, int16_t Y, uint16_t Colour, uint16_t SizeX,uint16_t SizeY, uint16_t Background_Colour)
{
	//~ flagReinit = 1;
	uint8_t 	function_char;
    int16_t 	i,j;

	function_char = Character;

	// Draw pixels
	LCD_fillRect(X, Y, CHAR_WIDTH*SizeX, CHAR_HEIGHT*SizeY, Background_Colour);

    if (function_char <= 0x20)
	{

	}
	else
	{
		function_char -= 0x20;
		int rw = (CHAR_WIDTH+7)/8;
		for (j=0; j<FONT_CURR.Height; j++)
		{
			uint8_t* pnt =  FONT_CURR.table+j*rw+function_char*FONT_CURR.Height*rw;
			for (i=0; i<FONT_CURR.Width; i++)
			{
				uint8_t bt = pnt[i/8];
				if(bt&(1<<(7-(i&7))))
				{
					LCD_fillRect(X+(i*SizeX), Y+(j*SizeY), SizeX,SizeY, Colour);
				}
			}
		}
	}
}

void LCD_Draw_Char(char Character, int16_t X, int16_t Y, uint16_t Colour, uint16_t Size, uint16_t Background_Colour)
{
	LCD_Draw_Char2(Character,X,Y,Colour,Size,Size,Background_Colour);
}
#endif
/*Draws an array of characters (fonts imported from fonts.h) at X,Y location with specified font colour, size and Background colour*/
/*See fonts.h implementation of font on what is required for changing to a different font when switching fonts libraries*/
void  LCD_Draw_Text(const char* Text, int16_t X, int16_t Y, uint16_t Colour, uint16_t Size, uint16_t Background_Colour)
{
    while (*Text) {
        LCD_Draw_Char(*Text, X, Y, Colour, Size, Background_Colour);
        X += CHAR_WIDTH*Size;
	Text++;
    }
}
void  LCD_Draw_Text2(const char* Text, int16_t X, int16_t Y, uint16_t Colour, uint16_t SizeX, uint16_t SizeY,uint16_t Background_Colour)
{
    while (*Text) {
        LCD_Draw_Char2(*Text, X, Y, Colour, SizeX,SizeY, Background_Colour);
        X += CHAR_WIDTH*SizeX;
	Text++;
    }
}


#if 0
#define COLOR_3BYTES 1
/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup Components
  * @{
  */ 
  
/** @addtogroup ILI9341
  * @brief This file provides a set of functions needed to drive the 
  *        ILI9341 LCD.
  * @{
  */

/** @defgroup ILI9341_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup ILI9341_Private_Defines
  * @{
  */
/**
  * @}
  */ 
  
/** @defgroup ILI9341_Private_Macros
  * @{
  */
/**
  * @}
  */  

/** @defgroup ILI9341_Private_Variables
  * @{
  */ 

LCD_DrvTypeDef   ili9341_drv = 
{
  ili9341_Init,
  ili9341_ReadID,
  ili9341_DisplayOn,
  ili9341_DisplayOff,
  0,
  0,
  0,
  0,
  0,
  0,
  ili9341_GetLcdPixelWidth,
  ili9341_GetLcdPixelHeight,
  0,
  0,    
};

/**
  * @}
  */ 
  
/** @defgroup ILI9341_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */ 
  
/** @defgroup ILI9341_Private_Functions
  * @{
  */   

/**
  * @brief  Power on the LCD.
  * @param  None
  * @retval None
  */
  
 static const uint8_t init_commands[] = {
        // Power control A
        6, LCD_POWERA, 0x39, 0x2C, 0x00, 0x34, 0x02,
        // Power control B
        4, LCD_POWERB, 0x00, 0xC1, 0x30,
        // Driver timing control A
        4, LCD_DTCA, 0x85, 0x00, 0x78,
        // Driver timing control B
        3, LCD_DTCB, 0x00, 0x00,
        // Power on sequence control
        5, LCD_POWER_SEQ, 0x64, 0x03, 0x12, 0x81,
        // Pump ratio control
        2, LCD_PRC, 0x20,
        // Power control 1
        2, LCD_POWER1, 0x10,
        // Power control 2
        2, LCD_POWER2, 0x10,
        // VCOM control 1
        3, LCD_VCOM1, 0x3E, 0x28,
        // VCOM cotnrol 2
        2, LCD_VCOM2, 0x86,
        // Memory access control
        2, LCD_MAC, 0x48,
	//~ 2, LCD_MAC, 0xC8,
        // Pixel format set
#ifdef COLOR_3BYTES
	2, LCD_PIXEL_FORMAT, 0x66,
#else
        2, LCD_PIXEL_FORMAT, 0x55,
#endif	
        // Frame rate control
        //3, LCD_FRMCTR1, 0x00, 0x1B,
	3, LCD_FRMCTR1, 0x00, 50,
        // Display function control
        4, LCD_DFC, 0x08, 0x82, 0x27,
        // 3Gamma function disable
        2, LCD_3GAMMA_EN, 0x00,
        // Gamma curve selected
        2, LCD_GAMMA, 0x01,
        // Set positive gamma
        16, LCD_PGAMMA, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00,
        16, LCD_NGAMMA, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F,
        0
};
void  dmaSendDataCont8(uint8_t *data,int cnt)
{
	int k ;
	for( k=0;k<cnt;k++)
	{
		ili9341_WriteData(data[k]);
	}
}
void LCD_exitStandby() {
    FMC_BANK1_WriteComand(LCD_SLEEP_OUT);
    Delay(150);
    FMC_BANK1_WriteComand(LCD_DISPLAY_ON);
}

void ili9341_Init(void)
{
  /* Initialize ILI9341 low level bus layer ----------------------------------*/
  //~ LCD_IO_Init();
#if 0	
  FMC_BANK1_WriteComand(0xCA);
  ili9341_WriteData(0xC3);
  ili9341_WriteData(0x08);
  ili9341_WriteData(0x50);
  FMC_BANK1_WriteComand(LCD_POWERB);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0xC1);
  ili9341_WriteData(0x30);
  FMC_BANK1_WriteComand(LCD_POWER_SEQ);
  ili9341_WriteData(0x64);
  ili9341_WriteData(0x03);
  ili9341_WriteData(0x12);
  ili9341_WriteData(0x81);
  FMC_BANK1_WriteComand(LCD_DTCA);
  ili9341_WriteData(0x85);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x78);
  FMC_BANK1_WriteComand(LCD_POWERA);
  ili9341_WriteData(0x39);
  ili9341_WriteData(0x2C);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x34);
  ili9341_WriteData(0x02);
  FMC_BANK1_WriteComand(LCD_PRC);
  ili9341_WriteData(0x20);
  FMC_BANK1_WriteComand(LCD_DTCB);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x00);
  FMC_BANK1_WriteComand(LCD_FRMCTR1);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x1B);
  FMC_BANK1_WriteComand(LCD_DFC);
  ili9341_WriteData(0x0A);
  ili9341_WriteData(0xA2);
  FMC_BANK1_WriteComand(LCD_POWER1);
  ili9341_WriteData(0x10);
  FMC_BANK1_WriteComand(LCD_POWER2);
  ili9341_WriteData(0x10);
  FMC_BANK1_WriteComand(LCD_VCOM1);
  ili9341_WriteData(0x45);
  ili9341_WriteData(0x15);
  FMC_BANK1_WriteComand(LCD_VCOM2);
  ili9341_WriteData(0x90);
  FMC_BANK1_WriteComand(LCD_MAC);
  ili9341_WriteData(0xC8);
  FMC_BANK1_WriteComand(LCD_3GAMMA_EN);
  ili9341_WriteData(0x00);
  FMC_BANK1_WriteComand(LCD_RGB_INTERFACE);
  ili9341_WriteData(0xC2);
  FMC_BANK1_WriteComand(LCD_DFC);
  ili9341_WriteData(0x0A);
  ili9341_WriteData(0xA7);
  ili9341_WriteData(0x27);
  ili9341_WriteData(0x04);
  
  /* Colomn address set */
  FMC_BANK1_WriteComand(LCD_COLUMN_ADDR);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0xEF);
  /* Page address set */
  FMC_BANK1_WriteComand(LCD_PAGE_ADDR);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x01);
  ili9341_WriteData(0x3F);
  FMC_BANK1_WriteComand(LCD_INTERFACE);
  ili9341_WriteData(0x01);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x06);
  
  FMC_BANK1_WriteComand(LCD_GRAM);
  LCD_Delay(200);
  
  FMC_BANK1_WriteComand(LCD_GAMMA);
  ili9341_WriteData(0x01);
  
  FMC_BANK1_WriteComand(LCD_PGAMMA);
  ili9341_WriteData(0x0F);
  ili9341_WriteData(0x29);
  ili9341_WriteData(0x24);
  ili9341_WriteData(0x0C);
  ili9341_WriteData(0x0E);
  ili9341_WriteData(0x09);
  ili9341_WriteData(0x4E);
  ili9341_WriteData(0x78);
  ili9341_WriteData(0x3C);
  ili9341_WriteData(0x09);
  ili9341_WriteData(0x13);
  ili9341_WriteData(0x05);
  ili9341_WriteData(0x17);
  ili9341_WriteData(0x11);
  ili9341_WriteData(0x00);
  FMC_BANK1_WriteComand(LCD_NGAMMA);
  ili9341_WriteData(0x00);
  ili9341_WriteData(0x16);
  ili9341_WriteData(0x1B);
  ili9341_WriteData(0x04);
  ili9341_WriteData(0x11);
  ili9341_WriteData(0x07);
  ili9341_WriteData(0x31);
  ili9341_WriteData(0x33);
  ili9341_WriteData(0x42);
  ili9341_WriteData(0x05);
  ili9341_WriteData(0x0C);
  ili9341_WriteData(0x0A);
  ili9341_WriteData(0x28);
  ili9341_WriteData(0x2F);
  ili9341_WriteData(0x0F);
  FMC_BANK1_WriteComand(LCD_PIXEL_FORMAT);
  ili9341_WriteData(0x66);
  
  FMC_BANK1_WriteComand(LCD_SLEEP_OUT);
  LCD_Delay(200);
  FMC_BANK1_WriteComand(LCD_DISPLAY_ON);
  /* GRAM start writing */
  FMC_BANK1_WriteComand(LCD_GRAM);
#endif
//~ LCD_exitStandby();
//~ /*
    uint8_t count;
    uint8_t *address = (uint8_t *) init_commands;
    //~ SPI_MASTER->CR1 &= ~SPI_CR1_SPE; // DISABLE SPI
    //~ SPI_MASTER->CR1 &= ~SPI_CR1_DFF; // SPI 8
    //~ SPI_MASTER->CR1 |= SPI_CR1_SPE;  // ENABLE SPI

    //~ TFT_CS_RESET;
    while (1) {
        count = *(address++);
        if (count-- == 0) break;
        FMC_BANK1_WriteComand(*(address++));
        dmaSendDataCont8(address, count);
	    //~ TFT_CS_SET;
        address += count;
    }
    //~ */
#if 0
    
  /* Configure LCD */
  #endif
}

/**
  * @brief  Disables the Display.
  * @param  None
  * @retval LCD Register Value.
  */
uint16_t ili9341_ReadID(void)
{
  //~ LCD_IO_Init();
  return ((uint16_t)ili9341_ReadData(LCD_READ_ID4, LCD_READ_ID4_SIZE));
}

/**
  * @brief  Enables the Display.
  * @param  None
  * @retval None
  */
void ili9341_DisplayOn(void)
{
  /* Display On */
  FMC_BANK1_WriteComand(LCD_DISPLAY_ON);
}

/**
  * @brief  Disables the Display.
  * @param  None
  * @retval None
  */
void ili9341_DisplayOff(void)
{
  /* Display Off */
  FMC_BANK1_WriteComand(LCD_DISPLAY_OFF);
}

/**
  * @brief  Writes  to the selected LCD register.
  * @param  LCD_Reg: address of the selected register.
  * @retval None
  */

/**
  * @brief  Reads the selected LCD Register.
  * @param  RegValue: Address of the register to read
  * @param  ReadSize: Number of bytes to read
  * @retval LCD Register Value.
  */
#ifdef    COLOR_3BYTES
inline void color_convert(uint16_t color,uint8_t* result)
{
	//~ #define RED             0xF800 
	//~ #define BLUE            0x001F
	//~ #define GREEN           0x07E0  
	result[2]=  ((color&0x1f)		<<(1+2))<<1;//|0x80;    //5 bit BLUE
	result[1]=  (((color>>5)&0x3f) <<(0+1))<<1;//|0x80;    //6 bit GREEN 
	result[0]=  (((color>>11)&0x1f)<<(1+2))<<1;//|0x80;    //5 bit  //RED
}
#endif



/**
  * @brief  Get LCD PIXEL WIDTH.
  * @param  None
  * @retval LCD PIXEL WIDTH.
  */
uint16_t ili9341_GetLcdPixelWidth(void)
{
  /* Return LCD PIXEL WIDTH */
  return ILI9341_LCD_PIXEL_WIDTH;
}

/**
  * @brief  Get LCD PIXEL HEIGHT.
  * @param  None
  * @retval LCD PIXEL HEIGHT.
  */
uint16_t ili9341_GetLcdPixelHeight(void)
{
  /* Return LCD PIXEL HEIGHT */
  return ILI9341_LCD_PIXEL_HEIGHT;
}


inline  void dmaSendDataCont16(uint16_t *data, uint32_t n) 
{
    int k;
    uint8_t dummy;
    uint8_t *pdata = (uint8_t *)data;
    for( k=0;k<n;k++)
    {
	    ili9341_WriteData(*(pdata+1));
	    ili9341_WriteData(*(pdata));
	    pdata+=2;
    }	    
}


void LCD_setAddressWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    uint16_t pointData[2];

    //~ TFT_CS_RESET;
    LCD_IO_WriteReg(LCD_COLUMN_ADDR);
    pointData[0] = x1;
    pointData[1] = x2;
    dmaSendDataCont16(pointData, 2);

    LCD_IO_WriteReg(LCD_PAGE_ADDR);
    pointData[0] = y1;
    pointData[1] = y2;
    dmaSendDataCont16(pointData, 2);
}


inline  void dmaFill16(uint16_t color, uint32_t n) {
    //~ TFT_CS_RESET;
    uint8_t dummy;
    LCD_IO_WriteReg(LCD_GRAM);
#ifdef    COLOR_3BYTES
     uint8_t pdata[3];
    color_convert(color,pdata);
    while (n != 0) 
    {
	dmaSendDataCont8(pdata,3);
	n--;    
    }
#else	
    uint8_t *pdata = (uint8_t *)&color;
    while (n != 0) 
       {
        dmaSendDataCont8(pdata,2);
	n--;    
    }
#endif	
}


void LCD_FullRect3(uint16_t x1, uint16_t y1,uint8_t * adress,uint16_t w,uint16_t h) 
{
        LCD_setAddressWindow(x1, y1, (uint16_t) (x1+w-1), (uint16_t) (y1+h-1));
    //~ LCD_setSpi16();
        uint8_t dummy;
       LCD_IO_WriteReg(LCD_GRAM);
#ifdef    COLOR_3BYTES
	int k;
	for(k=w*h;k>0;k--)
	{
		ili9341_WriteData(adress[0]);
		ili9341_WriteData(adress[1]);
		ili9341_WriteData(adress[2]);
		//~ adress+=3;
	}
#else	
#endif    
}


void LCD_fillRect(uint16_t x1, uint16_t y1, uint16_t w, uint16_t h, uint16_t color) {
     uint8_t pdata[3];
     color_convert(color,pdata);
     LCD_FullRect3(x1,y1,pdata,w,h); 
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
#endif
