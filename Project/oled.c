//////////////////////////////////////////////////////////////////////////////////
// OLED 驱动文件
// 适用于 SSD1306 OLED 显示屏
//////////////////////////////////////////////////////////////////////////////////
#include "oled.h"
#include "delay.h"
#include "oledfont.h"

//OLED 显存格式：
//[0]0 1 2 3 ... 127	第0页
//[1]0 1 2 3 ... 127	第1页
//[2]0 1 2 3 ... 127	第2页
//[3]0 1 2 3 ... 127	第3页
//[4]0 1 2 3 ... 127	第4页
//[5]0 1 2 3 ... 127	第5页
//[6]0 1 2 3 ... 127	第6页
//[7]0 1 2 3 ... 127	第7页

#if OLED_MODE==1
//向 SSD1106 写入一个字节
//dat: 要写入的数据/命令
//cmd: 数据/命令标志 0=命令, 1=数据
void OLED_WR_Byte(u8 dat,u8 cmd)
{
	DATAOUT(dat);
	if(cmd)
	  OLED_DC_Set();
	else
	  OLED_DC_Clr();
	OLED_CS_Clr();
	OLED_WR_Clr();
	OLED_WR_Set();
	OLED_CS_Set();
	OLED_DC_Set();
}
#else
//向 SSD1306 写入一个字节
//dat: 要写入的数据/命令
//cmd: 数据/命令标志 0=命令, 1=数据
void OLED_WR_Byte(u8 dat,u8 cmd)
{
	u8 i;
	if(cmd)
	  OLED_DC_Set();
	else
	  OLED_DC_Clr();
	OLED_CS_Clr();
	for(i=0;i<8;i++)
	{
		OLED_SCLK_Clr();
		if(dat&0x80)
			{
		   OLED_SDIN_Set();
			}
else
		   OLED_SDIN_Clr();
				OLED_SCLK_Set();
		dat<<=1;
	}
	OLED_CS_Set();
	OLED_DC_Set();
}
#endif

//设置显示位置
//x: 列地址 (0~127)
//y: 页地址 (0~7)
void OLED_Set_Pos(unsigned char x, unsigned char y)
{
	OLED_WR_Byte(0xb0+y,OLED_CMD);
	OLED_WR_Byte(((x&0xf0)>>4)|0x10,OLED_CMD);
	OLED_WR_Byte((x&0x0f)|0x01,OLED_CMD);
}

//开启 OLED 显示
void OLED_Display_On(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //设置电荷泵
	OLED_WR_Byte(0X14,OLED_CMD);  //电荷泵开启
	OLED_WR_Byte(0XAF,OLED_CMD);  //显示开启
}

//关闭 OLED 显示
void OLED_Display_Off(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //设置电荷泵
	OLED_WR_Byte(0X10,OLED_CMD);  //电荷泵关闭
	OLED_WR_Byte(0XAE,OLED_CMD);  //显示关闭
}

//清屏函数，清完屏后屏幕是黑色的
void OLED_Clear(void)
{
	u8 i,n;
	for(i=0;i<8;i++)
	{
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //设置页地址（0~7）
		OLED_WR_Byte (0x02,OLED_CMD);      //设置显示位置—列低地址
		OLED_WR_Byte (0x10,OLED_CMD);      //设置显示位置—列高地址
		for(n=0;n<128;n++)OLED_WR_Byte(0,OLED_DATA);
	} //清屏显示
}


//在指定位置显示一个字符
//x: 0~127
//y: 0~63
//mode: 0=正常显示, 1=反色显示
//size: 字体大小 16/12
void OLED_ShowChar(u8 x,u8 y,u8 chr)
{
	unsigned char c=0,i=0;
		c=chr-' '; //得到偏移后的值
		if(x>Max_Column-1){x=0;y=y+2;}
		if(SIZE ==16)
			{
			OLED_Set_Pos(x,y);
			for(i=0;i<8;i++)
			OLED_WR_Byte(F8X16[c*16+i],OLED_DATA);
			OLED_Set_Pos(x,y+1);
			for(i=0;i<8;i++)
			OLED_WR_Byte(F8X16[c*16+i+8],OLED_DATA);
			}
			else {
				OLED_Set_Pos(x,y+1);
				for(i=0;i<6;i++)
				OLED_WR_Byte(F6x8[c][i],OLED_DATA);

			}
}

//m^n 函数
u32 oled_pow(u8 m,u8 n)
{
	u32 result=1;
	while(n--)result*=m;
	return result;
}

//显示数字
//x,y : 起始坐标
//len : 数字的位数
//size: 字体大小
//mode: 模式 0=正常模式, 1=填充模式
//num: 数值 (0~4294967295)
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size2)
{
	u8 t,temp;
	u8 enshow=0;
	for(t=0;t<len;t++)
	{
		temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+(size2/2)*t,y,' ');
				continue;
			}else enshow=1;

		}
	 	OLED_ShowChar(x+(size2/2)*t,y,temp+'0');
	}
}

//显示一个字符串
void OLED_ShowString(u8 x,u8 y,u8 *chr)
{
	unsigned char j=0;
	while (chr[j]!='\0')
	{		OLED_ShowChar(x,y,chr[j]);
			x+=8;
		if(x>120){x=0;y+=2;}
			j++;
	}
}

//显示汉字
void OLED_ShowCHinese(u8 x,u8 y,u8 no)
{
	u8 t,adder=0;
	OLED_Set_Pos(x,y);
    for(t=0;t<16;t++)
		{
				OLED_WR_Byte(Hzk[2*no][t],OLED_DATA);
				adder+=1;
     }
		OLED_Set_Pos(x,y+1);
    for(t=0;t<16;t++)
			{
				OLED_WR_Byte(Hzk[2*no+1][t],OLED_DATA);
				adder+=1;
      }
}

//显示BMP图片128×64
//起始坐标 (x,y)，x的范围0~127，y为页的范围0~7
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[])
{
 unsigned int j=0;
 unsigned char x,y;

  if(y1%8==0) y=y1/8;
  else y=y1/8+1;
	for(y=y0;y<y1;y++)
	{
		OLED_Set_Pos(x0,y);
    for(x=x0;x<x1;x++)
	    {
	    	OLED_WR_Byte(BMP[j++],OLED_DATA);
	    }
	}
}

// ==========================================================
// OLED 底层绘图函数
// ==========================================================

// 1. 定义一个全局显存数组 (128列 x 8页 = 1024 字节)
uint8_t OLED_GRAM[128][8];

// 2. 画点函数
// x: 0~127, y: 0~63, t: 1(点亮) / 0(熄灭)
void OLED_DrawPoint(u8 x, u8 y, u8 t)
{
    u8 pos, bx, temp = 0;
    if(x > 127 || y > 63) return; // 超出屏幕范围，直接退出防止越界

    pos = y / 8; // 计算 Y 坐标在第几页 (Page 0 ~ Page 7)
    bx = y % 8;  // 计算 Y 坐标在该页内的哪一个 Bit (0 ~ 7)
    temp = 1 << bx; // 把对应位置 1

    if(t == 1) {
        OLED_GRAM[x][pos] |= temp;  // 点亮像素 (位或操作，不影响其他7个点)
    } else {
        OLED_GRAM[x][pos] &= ~temp; // 熄灭像素 (位与清除)
    }
}

// 3. 显存全部刷新函数
// 画点函数只是把数据写到了单片机的数组里，这个函数才会把整个数组的内容搬运到屏幕上
// 注意：这里调用了 oled.c 里的底层写命令函数（即上面的 OLED_WR_Byte）
void OLED_Refresh_Gram(void)
{
    u8 i, n;
    for(i = 0; i < 8; i++) {
        OLED_WR_Byte(0xb0 + i, 0); // 0=写命令：设置页地址（0~7）
        OLED_WR_Byte(0x00, 0);     // 设置显示位置—列低地址
        OLED_WR_Byte(0x10, 0);     // 设置显示位置—列高地址
        for(n = 0; n < 128; n++) {
            OLED_WR_Byte(OLED_GRAM[n][i], 1); // 1=写数据：刷新 128 列的数据
        }
    }
}


//初始化 SSD1306
void OLED_Init(void)
{



  OLED_RST_Set();
	delay_ms(100);
	OLED_RST_Clr();
	delay_ms(100);
	OLED_RST_Set();

	OLED_WR_Byte(0xAE,OLED_CMD); //关闭显示
	OLED_WR_Byte(0x02,OLED_CMD); //设置列地址低4位
	OLED_WR_Byte(0x10,OLED_CMD); //设置列地址高4位
	OLED_WR_Byte(0x40,OLED_CMD); //设置起始行地址
	OLED_WR_Byte(0x81,OLED_CMD); //设置对比度控制寄存器
	OLED_WR_Byte(0xCF,OLED_CMD); //设置 SEG 输出电流亮度
	OLED_WR_Byte(0xA1,OLED_CMD); //设置 SEG/列映射     0xa0 反转, 0xa1 正常
	OLED_WR_Byte(0xC8,OLED_CMD); //设置 COM/行扫描方向   0xc0 反转, 0xc8 正常
	OLED_WR_Byte(0xA6,OLED_CMD); //设置正常显示
	OLED_WR_Byte(0xA8,OLED_CMD); //设置复用率 (1 to 64)
	OLED_WR_Byte(0x3f,OLED_CMD); //1/64 占空比
	OLED_WR_Byte(0xD3,OLED_CMD); //设置显示偏移
	OLED_WR_Byte(0x00,OLED_CMD); //无偏移
	OLED_WR_Byte(0xd5,OLED_CMD); //设置显示时钟分频比/振荡器频率
	OLED_WR_Byte(0x80,OLED_CMD); //设置分频比，时钟为 100 帧/秒
	OLED_WR_Byte(0xD9,OLED_CMD); //设置预充电周期
	OLED_WR_Byte(0xF1,OLED_CMD); //预充电 15 个时钟，放电 1 个时钟
	OLED_WR_Byte(0xDA,OLED_CMD); //设置 COM 引脚硬件配置
	OLED_WR_Byte(0x12,OLED_CMD);
	OLED_WR_Byte(0xDB,OLED_CMD); //设置 VCOMH
	OLED_WR_Byte(0x40,OLED_CMD); //设置 VCOM 取消选择电平
	OLED_WR_Byte(0x20,OLED_CMD); //设置页面寻址模式 (0x00/0x01/0x02)
	OLED_WR_Byte(0x02,OLED_CMD); //页寻址模式
	OLED_WR_Byte(0x8D,OLED_CMD); //设置电荷泵使能/禁用
	OLED_WR_Byte(0x14,OLED_CMD); //电荷泵开启
	OLED_WR_Byte(0xA4,OLED_CMD); //禁用全屏显示 (0xa4/0xa5)
	OLED_WR_Byte(0xA6,OLED_CMD); //禁用反显 (0xa6/a7)
	OLED_WR_Byte(0xAF,OLED_CMD); //开启显示

	OLED_WR_Byte(0xAF,OLED_CMD); //显示开启
	OLED_Clear();
	OLED_Set_Pos(0,0);
}
