 /********************************************************/
// CPU需要：STM32F103--RAM内存不小于64K	Flash内存不小于128K
// 本代码已在STM32F103RDT6、VET6测试通过
// 编辑日期：20150909
// editor by 传人记
// 网店：https://shop178536326.taobao.com
/********************************************************
PLC相关的特殊寄存器
专用辅助继电器           描述
M8126                   全局标志
M8127                   通讯请求握手信号
M8128                   出错标志 
M8129                   通讯请求切换        
   
专用数据寄存器           描述
D8000 = 200;		   		  扫描时间
D8001 = 0X5EF6;	        型号版本 FX2N(C)
D8101 = 0X5EF6;	        型号版本 FX2N(C)
D8002 = 8;			   		  内存容量
D8102 = 8;			   		  内存容量
D8003 = 0x0010; 	   	  存储类型: PLC内置存储
D8006                   CPU电池电压
D8010 = 10;			   	    扫描当前值
D8011 = 20;			   	    扫描最小时间(0.1MS)
D8012 = 140;			   	  扫描最长时间(0.1MS) 
D6030 D6031 D6032 D6033       是模拟量输入
D8080 D8081                   是模拟输出



D8120 = 0X4096                通讯格式
D8121                         从站号（最多16个）
D8127                         交换数据的首地址
D8128                         交换数据量
D8129                         网络通讯超时时间确认值
D8000                         看门狗         
D8019                         对应星期
D8018                         对应年份
D8017                         对应月份
D8016                         对应日期
D8015                         对应小时
D8014                         对应分钟
D8013                         对应秒


通讯格式详解（D8120）
----------------------------------------------------------------------
位号	     |   含 义	   |          描述      
-----------+-------------+--------------------------------------------
b0	       |  数据长度	 |   0： 7位   1： 8位
-----------+-------------+--------------------------------------------
b2b1	     |  校验方式   |   00：无校验  01：奇校验  11：偶校验
-----------+-------------+--------------------------------------------
b3	       |   停止位	   |   0： 1位   1： 2位
-----------+-------------+--------------------------------------------
           |             |   0001：300      0111：4800    1011：56000
b7b6b5b4   |   波特率	   |   0100：600      1000：9600    1100：57600
           |             |   0101：1200     1001：19200   1101：115200
           |             |   0110：2400     1010：38400   
-----------+-------------+--------------------------------------------
b8    		 |   报头      |  0：无(MODBUS)；1:有[仅RS,D8124初始值：STX(02H)]
-----------+-------------+--------------------------------------------
b9    		 |   报尾      |  0:无(MODBUS); 1:有[仅RS,D8125初始值：ETX(03H)]
-----------+-------------+--------------------------------------------
b10~b12 	 |             |  预留
-----------+-------------+--------------------------------------------
b13	       |   和校验    |   0：不附加(MODBUS)；1：附加(RS)
-----------+-------------+--------------------------------------------
b14	       |   协议      |   0：专用协议(MODBUS)；1：无协议(RS)
-----------+-------------+--------------------------------------------
b15	       | 仅MODBUS    |   0：从机  1：主机			 
----------------------------------------------------------------------

举例：D8120 = 0X4096           通讯波特率是19200
*********************************************************************************/

#include "stm32f10x.h"
#include "stm32f10x_flash.h"
#include <stdio.h>
#include "PLC_Dialogue.h"
#include "PLC_IO.h"
#include "plc_conf.h"
#include "bsp_timer.h"

const u8   plc_programCodeBuf[34000] __at (PLC_RAM_ADDR)={						
//FLASH起始地址为PLC信息**************************最前的0X02表示PLC为16K的程序步,密码区域和差数区域****************
0x10,0x00,0xD8,0xBA,0x00,0x00,0x00,0x00,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,
0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,
0x20,0x20,0xF4,0x09,0xFF,0x0B,0xF4,0x01,0xE7,0x03,0x64,0x0E,0xC7,0x0E,0xDC,0x0E,0xFF,0x0E,0x90,0x01,0xFE,0x03,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x83,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0X0F,0X00,//结束指令
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,};
	

//D8000 = 200;		   		  扫描时间
//D8001 = 0X5EF6;	        型号版本 
//D8101 = 0X5EF6;	        型号版本 
//D8002 = 10;			   		  内存容量 16K
//D8102 = 10;			   		  内存容量 16000�
//D8003 = 0x0010; 	   	  存储类型: PLC内置存储
//D8006                   CPU电池电压
//D8010 = 10;			   	    扫描当前值
//D8011 = 20;			   	    扫描最小时间(0.1MS)
//D8012 = 140;			   	  扫描最长时间(0.1MS) 

const u16 special_d[256]={
0X00C8,0X5EA8,0X0008,0X0010,8066,32,27,0x0003,10,0,
0,0,0,0,0,0,0,0,0,0,
10,0X0000,0X0000,0X0000,0X0000,0X0000,0X0000,0X0000,0X0000,0X0000,
0X003D,0X001C,0X0000,0X0000,0X0014,0X00FF,0X03D7,0X0000,0X0000,0X0000,
0XFFFF,0XFFFF,0XFFFF,0XFFFF,0XFFFF,0XFFFF,0XFFFF,0XFFFF,0X0000,0XFFFF,
0X0000,0X0000,0X0000,0X0000,0X0000,0X0000,0X0000,0X0000,0X0000,0X0000,
0X0000,0X0000,0X183B,0X0000,0X0000,0X0000,0X0000,0X0000,0X0000,0X0000,
0X01F4,0X0000,0X0000,0X0000,0X0000,0X0000,0X0000,0X0000,0X0000,0X0000,
0X0000,0X0000,0X0000,0X0000,0X0000,0X0000,0X0000,0X0000,0X0000,0X0000,
0X0000,0X0000,0X0000,0X0000,0X0000,0X0000,0X0000,0X0000,0X0000,0X0000,
0X0064,0X3F68,0X0008,0X0000,0X0000,0X0000,0X0000,0X0000,0X0000,0X0000,
0X0000,0X0000,0X0000,0X0000,0X0000,0X0000,0X0DDC,0X3DB6,0X0000,0X0000,
0X0000,0X0000,0X0000,0X0000,0X0002,0X0003,0X0000,0X0000
};

const char Ascll[20]={0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0X41,0X42,0X43,0X44,0X45,0X46};
const char hex[]={
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,1,2,3,4,5,6,7,8,9,0,0,0,0,0,0,
0,10,11,12,13,14,15,0,0,0,0,0,0,};


//u16 progWriteBuf[4096];
u16 progWriteBuf[2048];
static u8 Flag_Uart_Send=1;             //发送标志位
u16 rx_count,tx_count;                  //数据计算       



char tx_data[PROGRAM_BUF_SIZE];         //发送缓存
char rx_data[PROGRAM_BUF_SIZE];         //接收缓存

u16 prog_address,data_address;          //计算数据操作起始地址缓存


u16 plc_16BitBuf[PLC_16BIT_BUF_SIZE] __at (RAM_ADDR);   //PLC_RAM运行数据

// 0x200066B0~0x2000D200 // 26KB
// 传人记，20170424修改
u8  step_status[1000];  
u8  step_address[2000]; //写步状态为0上升延与下降沿使用共2K字节1600bit

u8  data_size,block_contol[2];													
extern u8  edit_prog;
extern void RTC_Set(u16 syear,u8 smon,u8 sday,u8 hour,u8 min,u8 sec);	
												
u8 Send_out;
u8 Write_Pro_flag = 0;


void data_init(void)//D8000~D8126初始化
{                                                 
	u16 temp;                                       
	u16 temp_address;                                
	
	prog_address=0x52;                                
  for(temp=0;temp<126;temp++)                      
  {                                                                   
       temp_address=0x0C00;                        
	   plc_16BitBuf[temp_address+temp]=special_d[temp]; //传入系统FLASH备份的用户数据
  }                                                //
	// D8000
	plc_16BitBuf[0x2000] =plc_programCodeBuf[prog_address] *256; // 取字节高位                      
	plc_16BitBuf[0x2000] |=plc_programCodeBuf[prog_address+1]; // 取字节低位
	
	PLC_RW_RAM_8BIT(0x01E0) =0x09; // 令M8000 M8003置ON
	
	block_contol[0]=200; // 防止写参数时程序丢失
	block_contol[1]=200;                            
	
#if DEBUG
	  #warning "MC和MCR指令--data_init";
//	  MC.MC_SFR = 0;
//	  MC.MC_Flg = 0;
//	  MC.PLC_MC_BIT = 0;
#endif
	
}                                                  

void write_block(u16 number)                                                  // 写入FLASH
{ 
	u16 temp,wait_write,appoint_address;
	if(number<17)			                                                          // 写入参数的块必需10块
	{
		FLASH_Unlock();		                                                        // flash关闭保护
		FLASH_ErasePage(PLC_RAM_ADDR+number*0x800);                               // 擦除一块数据占用2K
		for(temp=0;temp<1024;temp++)	                                            // 操作为16bit,只需要1024次程序操作完毕
		{
			appoint_address =PLC_RAM_ADDR + number*0x800 + temp*2;                  // 起始地址加上块地址再加上块的小地址,等于目标位置 
			wait_write =progWriteBuf[temp*2] + progWriteBuf[temp*2+1]*0X100; // 写入16bit至flash
			FLASH_ProgramHalfWord(appoint_address,wait_write);                      // 等待程序写入结束
		}
		FLASH_Lock();	                                                            // 结束程序写入开启flash保护
	}
}

void backup_block(u16 number)		                                              // 程序块备份,目的在写程序之前进前面程序备份
{
	u16 temp,appoint_address;
	if(number<17)
	{
		for(temp=0;temp<2048;temp++)
		{
			appoint_address =number*0x800 + temp;                                    // 起始地址加上块地址再加上块的小地址 
			progWriteBuf[temp] =plc_programCodeBuf[appoint_address];	           // 将程序备份出来
		}
	}
}

//=======================================================================================================
// 函数名称: ErasurePLC
// 功能描述：PLC擦除FLASH空间
// 输　入:  mode 模式       
// 输　出:  void     
// 全局变量:  
// 调用模块: 
// 作　者:  传人记
// 日　期:  2014年5月18日
// 备  注:  
//-------------------------------------------------------------------------------------------------------
// 修改人:
// 日　期:
// 备  注: 
//-------------------------------------------------------------------------------------------------------
//=======================================================================================================
void ErasurePLC(u8 mode)
{
	u16 temp=0,Erasure_plc_16BitBuf;
	/*******************************************PLC存储内存清理	************************************************/
	if(mode==1)
	{
		backup_block(0);
		progWriteBuf[92]=0x0f;                                                         //赋值
		progWriteBuf[93]=0x00;                                                         //赋值
		for(temp=94;temp<2048;temp++)                                                       //从0x5E
		{ 
			progWriteBuf[temp]=0xffff;                                                    //
		}
		FLASH_Unlock();                                                                     //flash关闭保护                     
		FLASH_ErasePage(PLC_RAM_ADDR+0x800);                                            
		write_block(0);                                                                     //备份第一块flash
		for(temp=1;temp<10;temp++)                                                          //擦除10块
		FLASH_ErasePage(PLC_RAM_ADDR+temp*0x800);                                           //擦除flash
		FLASH_Lock();                                                                       //结束程序写入开启flash保护
	}
/*******************************************PLC清理数据元件位*********************************************/
   if(mode==2)
	 {
	   for(Erasure_plc_16BitBuf=0x4000;Erasure_plc_16BitBuf<0x7E7E;Erasure_plc_16BitBuf+=2)           // 清除D0000-D7999
       plc_16BitBuf[Erasure_plc_16BitBuf]=0x00;
	 }
/*******************************************PLC清理位元件**************************************************/
	 if(mode==3)
	 {
	      for(Erasure_plc_16BitBuf=0x0000;Erasure_plc_16BitBuf<0x00BE;Erasure_plc_16BitBuf+=2)	    // 清除M0000-M3071
        plc_16BitBuf[Erasure_plc_16BitBuf]=0x00;
	 }
   tx_data[1]=0x06,tx_count=1,Send_out=5;                                                 // 清除完毕报告上位机
}
				
/*******************************************************************************
函数功能：计算校验和 
备注： 20171102，传人记优化
*******************************************************************************/
u8 CheckSum(char * pBuf)//计算接收区和校验
{ 
	u16 i; 
	u8 sum;
	sum=0;		// 请除和记算器
	pBuf +=3;	// 计算和从第三位开始
	for(i=3;i<(rx_count-1);i++) // 计算和
	{ 
		sum += *pBuf; // 开始相加
		pBuf++;				// 指针加一
	}
	return sum;			// 数据正常	
}

/*******************************************************************************
函数名称：void switch_read_data(void)  
函数功能：转换ASCII码为HEX码，占用数据发送寄存器         
出口参数：无
********************************************************************************/
void switch_read_data(void)             
{ 
	u16 temp;
	for(temp=4;temp<(rx_count-2);temp++)
	{
		tx_data[temp/2]=hex[rx_data[temp]]*0x10;
		tx_data[temp/2]+=hex[rx_data[temp+1]];      
		temp++;
	}
}

void setup_HL(void)	                     //高低位交换再转换 ，小端转换
{                                                                     
	u8 temp;                                                            
	temp=tx_data[3];				               //地址高位送入16位数据区     
	prog_address=temp*0x100+tx_data[2];    //计算程序操作起始地址       
}	                                                                     

void setup_LH(void)	                     //正常地址转换
{ 
	u8 temp;
	temp=tx_data[3];				              //地址高位送入16位数据区
	data_address=temp*0x100+tx_data[4];  //计算数据操作起始地址
}

typedef union                           
{
	int data;
	char data1[2];
} usart_data;

// 传人记，20161122注释，//读取PLC型号，指令“30”
void read_plc_tyte(u8 addr)                                  
{
	u16 temp;
	u8 temp_sum; 
	usart_data plc_type;
	
	plc_type.data=special_d[addr];                             //PLC型号
	tx_data[1]=0x02;                                           //报文开始	02
	temp_sum=0;
	for(temp=0;temp<data_size;temp++)
	{ 
		tx_data[temp*2+2]=Ascll[plc_type.data1[temp]/0x10]; //取字节高位
		tx_data[temp*2+3]=Ascll[plc_type.data1[temp]%0x10]; //取字节低位
		temp_sum+=tx_data[temp*2+2]+tx_data[temp*2+3];
	}
	tx_data[temp*2+2]=0x03;                                     //报文结束	03
	temp_sum+=0x03;
	tx_data[temp*2+3]=Ascll[temp_sum/0x10];
	tx_data[temp*2+4]=Ascll[temp_sum%0x10]; 
	tx_count=temp*2+4;
}


/*******************************************************************************
函数名称PLC_Comm_Byte
函数功能：通信字节地址重定义及执行
入口参数：Cmd命令          
出口参数：映射的实际地址(16BIT地址)
********************************************************************************/
u16 PLC_Comm_Byte(u16 comm_add)
{
  /* FX3U */
	if(comm_add>=0x4000&&comm_add<=0x7E7F)      // D0000-D7999 
			return comm_add-0x2000;	
	else if(comm_add>=0x8000&&comm_add<=0x82FF)
			return comm_add-0X6800;//D8000-D8510 
	else if(comm_add>=0xffC0&&comm_add<=0x82FF)
			return comm_add-0X6800;//D8000-D8510 
	else if(comm_add>=0x8CE0&&comm_add<=0x8D5F)
			return comm_add-0x8CE0;//S0000-S1023 
	else if(comm_add>=0x8D60&&comm_add<=0x8EDF)
			return comm_add-0X8AE0;//S1024-S4095 
	else if(comm_add>=0x8CA0&&comm_add<=0x8CFF)
			return comm_add-0x8C20;//X000-X377	 
	else if(comm_add>=0x8BC0&&comm_add<=0x8BFF)
			return comm_add-0x8B20;//Y000-Y367   
	else if(comm_add>=0x8800&&comm_add<=0x88BF)
			return comm_add-0x8700;//M0000-M1535 
	else if(comm_add>=0x88C0&&comm_add<=0x8BBA)
			return comm_add-0X83C0;//M1536-M7679 
	else if(comm_add>=0x8C00&&comm_add<=0x8C3F)
			return comm_add-0x8A20;//M8000-M8255 
 
	/* FX2N */ 
	else if(comm_add>=0x0280&&comm_add<=0x02FC) // S000-S999 
			return comm_add-0x0280;
	else if(comm_add>=0x0240&&comm_add<=0x0256) // X000-X277	
			return comm_add-0x01C0;
	else if(comm_add>=0x0180&&comm_add<=0x0196) // Y000-Y267
			return comm_add-0x00E0; 
	else if(comm_add>=0x0200&&comm_add<=0x021F) // T00-T255  OVER触点
			return comm_add-0x0140;
	else if(comm_add>=0x0500&&comm_add<=0x051F) // T00-T255  Enable 线圈
			return comm_add-0x0240; 
	else if(comm_add>=0x01E0&&comm_add<=0x01FF) // C00-C255  OVER触点
			return comm_add-0x0020;
	else if(comm_add<=0x00BF) // M0000-M1535 
			return comm_add+0x0100;
	else if(comm_add>=0x00C0&&comm_add<=0x017F) // M1536-M3071
			return comm_add+0x0440;
	else if(comm_add>=0x01C0&&comm_add<=0x01DF) // M8000-M8255 
			return comm_add+0x0020;
	else // 无效地址
			return comm_add;                                              
}

/*******************************************************************************
函数名称：PLC_Com_BIT
函数功能：通信位地址重定义及执行
入口参数：Cmd命令          
出口参数：映射的实际地址(BIT地址)
********************************************************************************/
u16 PLC_Com_BIT(u16 addr)        
{ 
  /* FX3U */
 if((addr>=0x4000)&&(addr<=0x45FF)) // M0000-M1535 *
	 return (addr-0X3800); 
 else if((addr>=0x4600)&&(addr<=0x5DFF)) // M1536-M7679 *
	 return (addr-0x1E00); 
 else if((addr>=0x5E00)&&(addr<=0x5EFF)) // Y00-Y377    *
	 return (addr-0x5900);
 else if((addr>=0x6000)&&(addr<=0x61FF)) // M8000-M8255 *   
	 return (addr-0X5100); 
 else if((addr>=0x6500)&&(addr<=0x65FF)) // X00-X377	   *
	 return (addr-0x6100); 
 else if((addr>=0x6700)&&(addr<=0x6AFF)) // S00-S1023	 *
	 return (addr-0x6700); 
 else if((addr>=0x6B00)&&(addr<=0x76FF)) // S1024-S4095 *
	 return (addr-0X5700); 
 else if((addr>=0x9800)&&(addr<=0x99FF)) // T00-T255 Enable 线圈 *
	 return (addr-0X7200); 
 else if((addr>=0x9700)&&(addr<=0x97FF)) // C00-C255 Enable 线圈 *
	 return (addr-0X7200); 
 else if((addr>=0x6300)&&(addr<=0x64FF)) // T00-T255 OVER   触点 *
	 return (addr-0X5D00); 
 else if((addr>=0x6200)&&(addr<=0x62FF)) // C00-C255	OVER   触点	*
	 return (addr-0X5400); 

	/*FX2N*/
	if((addr<=0x05FF))                      
		return (addr+0x0800); //M0000-M1535
	else if((addr>=0x0600)&&(addr<=0x0BFF)) 
		return (addr+0x2200); //M1536-M3071
	else if((addr>=0x0C00)&&(addr<=0x0CB7)) 
		return (addr-0x0700); //Y00-Y267
	else if((addr>=0x1200)&&(addr<=0x12BF)) 
		return (addr-0x0E00); //X00-X267	
	else if((addr>=0x1400)&&(addr<=0x17E7)) 
		return (addr-0X1400); //S00-S999	
	else if((addr>=0x2800)&&(addr<=0x28FF)) 
		return (addr-0X1200); //T00-T255 Enable 线圈
	else if((addr>=0x1000)&&(addr<=0x10FF)) 
		return (addr-0X0A00); //T00-T255 OVER   触点
	else if((addr>=0x0E00)&&(addr<=0x0EFF)) 
		return (addr+0X0100); //M8000-M8255
	else if((addr>=0x0F00)&&(addr<=0x0FFF)) 
		return (addr-0X0100); //C00-C255 OVER触点
	else  return addr;
}


//=======================================================================================================
// 函数名称:  void READ_data(void)
// 功能描述： 读数据  X,Y,M,S,T,C,D   
// 输　入:   void  
// 输　出:   void   
// 全局变量:  
// 调用模块: 
// 作　者:  传人记
// 日　期:  2014年6月10日
// 备  注:  
//-------------------------------------------------------------------------------------------------------
// 修改人:
// 日　期:
// 备  注:  传人记，20161122注释，PLC_Comm_Byte获取FX0N编程口通讯访问时的内部软元件地址，S X Y T M C M8， 【？ T C16 C32 D8 D D1000 】
//=======================================================================================================
void read_other_data(void)	                                       //指令“30”
{
	u16 i;
	u8 temp_sum;
	tx_data[1]=0x02;                                                 //报文开始
	temp_sum=0;
	for(i=0;i<data_size;i++)
	{ 
		tx_data[i*2 + 2]=Ascll[PLC_RW_RAM_8BIT(PLC_Comm_Byte(i + prog_address))/0x10];//取字节高位
		tx_data[i*2 + 3]=Ascll[PLC_RW_RAM_8BIT(PLC_Comm_Byte(i + prog_address))%0x10];//取字节低位
		temp_sum+=tx_data[i*2+2]+tx_data[i*2+3];
	}
	tx_data[i*2 + 2]=0x03;                                          //报文结束	03
	temp_sum+=0x03;
	tx_data[i*2 + 3]=Ascll[temp_sum/0x10];
	tx_data[i*2 + 4]=Ascll[temp_sum%0x10]; 
	tx_count=i*2+4;
}

//=======================================================================================================
// 函数名称:  void READ_data(void)
// 功能描述： 写数据  X,Y,M,S,T,C,D   
// 输　入:   void  
// 输　出:   void   
// 全局变量:  
// 调用模块: 
// 作　者:  传人记
// 日　期:  2014年6月10日
// 备  注:  
//-------------------------------------------------------------------------------------------------------
// 修改人:
// 日　期:
// 备  注: 
//=======================================================================================================
void PC_WRITE_byte(void)                       //写字
{ 
	u16 temp;
	prog_address=tx_data[2]*0x100+tx_data[3]+4;//计算数据操作起始地址
	for(temp=0;temp<data_size;temp++)
	{
		PLC_RW_RAM_8BIT(PLC_Comm_Byte(temp+prog_address))=tx_data[5+temp];
	}
	tx_data[1]=0x06,tx_count=1;                //报告上位机
}

//=======================================================================================================
// 函数名称:  void FORCE_ON_data(void)   
// 功能描述： FORCE ON  X,Y,M,S,T,C
// 输　入:   void  
// 输　出:   void   
// 全局变量:  
// 调用模块: 
// 作　者:  传人记
// 日　期:  2014年6月10日
// 备  注:  
//-------------------------------------------------------------------------------------------------------
// 修改人:
// 日　期:
// 备  注: 
//=======================================================================================================
void PC_FORCE_ON(void)	                                //强制  38 ON
{ 
	PLC_BIT_ON(PLC_Com_BIT(tx_data[2]*0x100+tx_data[3]));//计算数据操作起始地址
	tx_data[1]=0x06,tx_count=1;  
}

//======================================================================================================
// 函数名称:  void FORCE_ON_data(void)   
// 功能描述： FORCE OFF  X,Y,M,S,T,C
// 输　入:   void  
// 输　出:   void   
// 全局变量:  
// 调用模块: 
// 作　者:  传人记
// 日　期:  2014年6月10日
// 备  注:  
//------------------------------------------------------------------------------------------------------
// 修改人:
// 日　期:
// 备  注: 
//=======================================================================================================
void PC_FORCE_OFF(void)	                      //强制  37  OFF 
{ 
	PLC_BIT_OFF(PLC_Com_BIT(tx_data[2]*0x100+tx_data[3]));//计算数据操作起始地址
	tx_data[1]=0x06,tx_count=1;   
}

void PC_READ_byte(void)	                        //读字
{
	prog_address=tx_data[2]*0x100+tx_data[3];     //计算数据操作起始地址
	switch(prog_address)
	{ 
		case 0x0ECA: read_plc_tyte(101);  break;  //读PLC TYPE      传人记，20161122注释，D8101
		case 0x0E02: read_plc_tyte(1);    break;  //读PLC TYPE      传人记，20161122注释，D8001
		default: read_other_data();       break;
	}
}


void EPC_FORCE_ON(void)	        //使用扩展功能"E"强制ON
{  
   switch(prog_address)
   {
			case 0x6023: PLC_RW_RAM_8BIT(0X01E0)=0x09;  break;  // 远程操作请求是否可以进行
	    case 0x6024: // 远程操作需要运行
			{
				PLC_RW_RAM_8BIT(0X01E0)=0x09;  
				Write_Pro_flag = 0;  
			  break;  
			}
	    case 0x6025: PLC_RW_RAM_8BIT(0X01E0)=0x0A;  break;  // 远程操作需要停止
      default:     PLC_BIT_ON(PLC_Com_BIT(prog_address));break;  // 其它操作区域
	 }
   tx_data[1]=0x06,tx_count=1;
}

void EPC_FORCE_OFF(void)	  //使用扩展功能"E"强制OFF 
{ 
  PLC_BIT_OFF(PLC_Com_BIT(prog_address));
  tx_data[1]=0x06,tx_count=1;
}  

void PC_READ_Parameter(void)                                  //读配置	  E00
{
  u16 temp,temp_bit,temp_addr,mov_bit,temp1;
  u8 temp_sum;
  u8 send,monitor,monitor1,monitor2; 
  tx_data[1]=0x02;                                             //报文开始
  temp_sum=0;
  prog_address=PLC_Comm_Byte(tx_data[3]*0x100+tx_data[4]);       //计算数据操作起始地址
  if((prog_address==0x1790)||(prog_address==0x17D0))             //请求读监控数据区0X1790 与 0X17D0地址
  {
		  if(prog_address==0x1790)
      {
   	     monitor1=PLC_RAM8(0x20002400);	                         //读出需要监控字数量
	       for(temp1=monitor=0;monitor<monitor1;monitor++)         //读监控字数据
	       { 
					  temp_bit=PLC_Comm_Byte(plc_16BitBuf[0X0A02+monitor])/2;
					  if(0x7FE0==temp_bit)
						{ 
							monitor++;
						  temp_bit=plc_16BitBuf[0X0A02+monitor];
							plc_16BitBuf[0x0BC8+temp1]=plc_16BitBuf[temp_bit];temp1++;
						}
						else
				    plc_16BitBuf[0x0BC8+temp1]=plc_16BitBuf[temp_bit];temp1++;//将需要的数据传到缓存		
						
					  if((temp_bit>=0x600)&&(temp_bit<=0x66E))             //主要是C200-C255以后的地址是32位的 
					  {   
						  plc_16BitBuf[0x1790/2+temp1]=plc_16BitBuf[temp_bit+1];   //将需要的数据传到缓存
						  temp1++;
					  }							
		     }
	       monitor2=PLC_RAM8(0x20002402);                          //读出需要监控位数量
				 temp1=temp1+16;                                         //
	       for(monitor1=0;monitor1<monitor2;monitor1++)            //读监控位数据
	       {  
						temp_addr=PLC_Com_BIT(plc_16BitBuf[0x0A02+monitor+monitor1]);
		        temp_bit=plc_16BitBuf[temp_addr/0x10];
		        mov_bit = temp_addr%0x10;
		        if((temp_bit&(1<<mov_bit))==(1<<mov_bit))
		        plc_16BitBuf[0x0BC8+temp1+monitor1/0x10]|=1<<(monitor1%0x10);//串口缓存数据	
			      else plc_16BitBuf[0x0BC8+temp1+monitor1/0x10]&=~(1<<(monitor1%0x10));	      					
		     }
	     }
   }
	 for(temp=0;temp<data_size;temp++)	    //读RAM
   { 
			send=PLC_RW_RAM_8BIT(prog_address+temp);
	    tx_data[temp*2+2]=Ascll[send/0x10]; //取字节高位
      tx_data[temp*2+3]=Ascll[send%0x10]; //取字节低位
	    temp_sum+=tx_data[temp*2+2]+tx_data[temp*2+3];
   }
   tx_data[temp*2+2]=0x03;    //
   temp_sum+=0x03;
   tx_data[temp*2+3]=Ascll[temp_sum/0x10];
   tx_data[temp*2+4]=Ascll[temp_sum%0x10]; 
   tx_count=temp*2+4;
}

void PC_WRITE_Parameter(void)                //写配置	  E10
{  
	u16 temp;
	prog_address=tx_data[3]*0x100+tx_data[4];  //计算数据操作起始地址
	for(temp=0;temp<data_size;temp++)	         //write  RAM
	{
	    PLC_RW_RAM_8BIT(PLC_Comm_Byte(prog_address+temp))=tx_data[6+temp];
	}
	tx_data[1]=0x06,tx_count=1; //报告上位机
}
 

void PC_READ_PORG(void)	          //读程序E01
{
	u16 temp;
	u8 temp_sum; 
	
	if(0x805C < data_address)
	{
		tx_data[1]=0x15;
		tx_count=1; 
		return;
	}	
	
	tx_data[1]=0x02;                 //报文开始
	temp_sum=0;
	data_address-=0x8000;	           //读FLASH 地址减0x8000等于实际位置
	for(temp=0;temp<data_size;temp++)
	{ 
		tx_data[temp*2+2]=Ascll[plc_programCodeBuf[data_address+temp]/0x10]; //取字节高位
		tx_data[temp*2+3]=Ascll[plc_programCodeBuf[data_address+temp]%0x10]; //取字节低位
		temp_sum+=tx_data[temp*2+2]+tx_data[temp*2+3];
	}
	tx_data[temp*2+2]=0x03;  
	temp_sum+=0x03;
	tx_data[temp*2+3]=Ascll[temp_sum/0x10];
	tx_data[temp*2+4]=Ascll[temp_sum%0x10]; 
	tx_count=temp*2+4;
}

//  梯形图下载
void PC_WRITE_PORG(void)	                           //写程序 E11 
{  
	 u16 i;
   prog_address=tx_data[3]*0x100+tx_data[4];         //计算数据操作起始地址
	 
	 PLC_RW_RAM_8BIT(0X01E0)=0x0A; 
	 Write_Pro_flag = 1;			   //防止在下载过程中程序运行 ADD 传人记
	
	 edit_prog=0;                                      //把程序编辑清除,因为写程序时可能存在P地址发生变化，PLC应计算P地址
	 prog_address-=0x8000;                             //地址减0X8000等于实际位置
	 for(i=0;i<data_size;i++)
	 {
		    block_contol[0]=(prog_address+i)/0x800;	 //每一块占用的地址 0X800＝2K 字节
	      if(block_contol[0]==block_contol[1])			   //是否需要跳块，
	      {
			    progWriteBuf[(prog_address+i)-block_contol[0]*0x800]=tx_data[6+i];  //将数据写入缓存中
	      }
		    else							                           //需要跳块处理
		    {
			    write_block(block_contol[1]);              //将前一块写入到FLASH
		      backup_block(block_contol[0]);             //备份需要写的FLASH块
		      block_contol[1]=block_contol[0];
		      progWriteBuf[(prog_address+i)-block_contol[0]*0x800]=tx_data[6+i];
		    }
   }
   tx_data[1]=0x06,tx_count=1;
}

static u16 find_data(u16 addr,u16 find_data)             //查找数据地址，并返回找到的数据地址
{
	 u8 find_ok,data_H,data_L;
   find_ok=5;
   data_H=find_data/0x100;
   data_L=find_data%0x100;
	 addr-=0x8000;
   do{
	      if((plc_programCodeBuf[addr]==data_L)&&(plc_programCodeBuf[addr+1]==data_H))
	      find_ok=0;		                                   //找到需要的指令
	      else
	      addr+=2;
		    if(addr>(0xFD5C-0x8000))
		    find_ok=1;                                       //在有效的范围内没有找到END指令
	   }while(find_ok>3);
	 
	return addr+0X8000;
}
 
static void find_data_address(void)       //查找上位机需要的指令地址
{ 
	   u8 temp_sum,data_H,data_L;  
		 data_L=tx_data[5];	           //需要查找数据的内容低位	
     data_H=tx_data[6];            //需要查找数据的内容高位
	   data_address=find_data(data_address,data_H*0X100+data_L);	//读FLASH
	   tx_data[1]=0x02;              //报文开始
     temp_sum=0;
	   tx_data[2]=0x31;
	   temp_sum+=tx_data[2];
	   data_H=data_address/0x100;
	   data_L=data_address%0x100;
	   tx_data[3]=Ascll[data_H/0X10];
	   tx_data[4]=Ascll[data_H%0X10];
	   tx_data[5]=Ascll[data_L/0X10];
	   tx_data[6]=Ascll[data_L%0X10];
     tx_data[7]=0X03;
	   temp_sum+=tx_data[3];
	   temp_sum+=tx_data[4];
	   temp_sum+=tx_data[5];
	   temp_sum+=tx_data[6];
	   temp_sum+=tx_data[7];
	   tx_data[8]=Ascll[temp_sum/0x10];
     tx_data[9]=Ascll[temp_sum%0x10]; 
	   tx_count=9;
}

void mov_flash(u16 addr,u8 mov_addr) 
{
	u16 start_addr,end_addr,backup_addr,temp,temp1,temp2,mov_byte,addr_mov; 
  static u8 offset;
  offset=mov_addr;
  end_addr=find_data(addr+0x8000,0x000f)+mov_addr-0x8000; //找出END指令所在位置，再加上移动量，等于最到END所在位置
  start_addr=end_addr;
  addr_mov=addr;   
  if(addr>0x5B)            //当前移动是否大于0步
	{
		  addr_mov-=0X5C;	     //减去参数地址
	    end_addr-=0x5C;
	    addr_mov/=2;		     //一步占用两个字节　除2为指到起始字节
	    end_addr/=2;
		  addr_mov/=8;			   //指令向每一步的壮态地址
	    end_addr/=8;
	    offset/=2;
	    mov_byte=offset/8;
	    offset%=8;
 	    while(!(end_addr==addr_mov))	                               //END address and curennt addr  是否相等	 ,不相等需要循环
	    {
	     temp=step_address[end_addr]*0x100+step_address[end_addr-1]; //调出前16位的壮态值，为后面需要移动
		   temp<<=offset;                                              //向后移动偏移的地址余数
		   step_address[end_addr+mov_byte]=temp/0x100;		             //附值到目标地址
		   end_addr--;							                                   //地址向前移8步
		  }
      temp=step_address[end_addr]*0x100+step_address[end_addr-1];	 //调出前16位的壮态值，为后面需要移动
      temp<<=offset;                                               //向后移动偏移的地址余数
      step_address[end_addr+mov_byte]=temp/0x100;		               //附值到目标地址	    
	}
	end_addr=start_addr;   
	temp=start_addr;
do{
     if((end_addr/0x800)==(addr/0x800))                              //目标移动位置与实际移动位置到了一个块吗
     start_addr=addr%0x800;			                                     //如果已到了一个块那么起始地址为目标位置开模
	   else
     start_addr=0;					                                         //没到则从0开始移动
     if((temp/0x800)==(end_addr/0x800))                              //END所在的地址，跟正在移动的地址是在一个块吗，如果在一个块，则需要进行结束度度处理
     temp1=end_addr%0x800+1; 
	   else
	   temp1=2048;					                                           //如果没有在一个块直接到一个块的结束位置2048
	   backup_block(end_addr/0x800);
	   for(temp2=start_addr;temp2<temp1+1;temp2++)
	   {
		   backup_addr=(end_addr/0x800)*0x800+temp2-mov_addr;            //地址向前偏移需要移动的量
	     progWriteBuf[temp2]=plc_programCodeBuf[backup_addr];			  	       //备份地址
	   }
	   write_block(end_addr/0x800);                                    //将移动过的数据写到FLASH
	   end_addr-=(temp1-start_addr);                                   //算出现在移动后的位置  
	 }while(end_addr>addr+mov_addr);   
}


void online_write_data(void)                             //在线写程序
{  
	 static u16 temp,Size;
   signed short temp1,temp2;
   temp1=tx_data[6];                                      //存入的是在多少个数据写
	 temp2=tx_data[8];                                      //需要写入多少个字节
	 temp2-=temp1;                                          //计算一块地址 
	 if(temp2>0)                                            //如果写入的数长度大于实际的数据长度，则需要挪动flash地址
	 {
	   mov_flash(data_address-0x8000,temp2); //需要挪动flash的地址，以及挪动的长度
	 }                
   edit_prog=0;                                           //把程序编辑清除,因为写程序时可能存在P地址发生变化，PLC应计算P地址 
	 
	 block_contol[0]=100;
	 block_contol[1]=100;
     prog_address=(tx_data[3]*0x100+tx_data[4])-0x8000;     //计算数据操作起始地址
     data_size=tx_data[8];
	 for(temp=0;temp<data_size;temp++)
	 {
		     block_contol[0]=(prog_address+temp)/0x800;	      //每一块占用的地址 0X800＝2K 字节
	       if(block_contol[0]==block_contol[1])			        //是否需要跳块处理
	       {progWriteBuf[(prog_address+temp)-block_contol[0]*0x800]=tx_data[9+temp];}  //将数据写入缓存中
		     else							                                //需要跳块处理
		     {
			    write_block(block_contol[1]);                   //将前一块写入到FLASH
		      backup_block(block_contol[0]);                  //备份需要写的FLASH块
		      block_contol[1]=block_contol[0];
		      progWriteBuf[(prog_address+temp)-block_contol[0]*0x800]=tx_data[9+temp];
		     }
	  }
	  write_block(block_contol[0]);                         //将前一块写入到FLASH  把串口下发程序写入plc_programCodeBuf 
    if(temp2<0)	                                          //删除程序数量
		{    
			   temp2=0-temp2;                                   //计算删除程序数量
			   Size=find_data(0x8000,0x000f)-prog_address;      //把 END地址-起始操作地址=操作数量
		   	 for(;temp<Size;temp++)                           //temp 保留上 写入状态
	       {
		        block_contol[0]=(prog_address+temp)/0x800;	  //每一块占用的地址 0X800＝2K 字节
	          if(block_contol[0]==block_contol[1])			    //是否需要跳块处理
	          {
			       progWriteBuf[(prog_address+temp)-block_contol[0]*0x800]=plc_programCodeBuf[prog_address+temp+temp2];  //将数据写入缓存中
	          }
		        else							                            //需要跳块处理
		        {
			        write_block(block_contol[1]);               //将前一块写入到FLASH
		          backup_block(block_contol[0]);              //备份需要写的FLASH块
		          block_contol[1]=block_contol[0];            //
		          progWriteBuf[(prog_address+temp)-block_contol[0]*0x800]=plc_programCodeBuf[prog_address+temp+temp2];
		        }
	       }   
				 write_block(block_contol[0]);                    //将前一块写入到FLASH   
		}
    tx_data[1]=0x06,tx_count=1;
}

void all_flash_unlock(void)                               //FLASH全部解锁
{
  block_contol[1]=200;
  block_contol[0]=200;
  tx_data[1]=0x06,tx_count=1;
}

void all_flash_lock(void) 	                                  //FLASH全部加锁
{
  write_block(block_contol[1]);                               //加锁之前把需要写的数据写到FLASH
  block_contol[1]=200;
  block_contol[0]=200;
  FLASH_Lock();
  tx_data[1]=0x06,tx_count=1;
}

void PC_OPTION_PROG(void)                                      //扩展功能 "E" 代码
{ 	
	 u16 temp;
	 if((rx_count==11)&&((rx_data[4]==0x37)||(rx_data[4]==0x38)))//是否为强制功能 
	 {	
         prog_address=hex[rx_data[5]]*0x10+hex[rx_data[6]]+hex[rx_data[7]]*0x1000+hex[rx_data[8]]*0x100;
	     if(rx_data[4]==0x37) 
		   EPC_FORCE_ON();  
		   else
		   EPC_FORCE_OFF();
		}
		else
	  {
			 setup_LH();			                                      //调用算出地址函数
	     temp=tx_data[2];
	     switch(temp) 
		   { 
				 case 0x00: PC_READ_Parameter();  break;               //读配置 E00
			case 0x10: //写配置 E10
			{
				// 传人记，20180524，新增gxwork2直接设置时钟，应该需要特殊处理？
				// E10 0E1A[d8013] 0E24[d8018] 0010[分] 0011[时] 0018[日] 0005[月] 00 E207[年] 04[星期] 00
				
				PC_WRITE_Parameter(); 				
			  break;               
			}
			case 0x01: // 读程序 E01
			{
				PC_READ_PORG();       
			  break; 
      }				
			   case 0x11: PC_WRITE_PORG();      break;               //写程序 E11 
			   case 0x77: all_flash_unlock();   break;               //使用E指令进行写程序写请求77
			   case 0x87: all_flash_lock();     break;               //使用E指令进行写程序结束请求87
			   case 0x41: find_data_address();  break;               //查找END指令地址	
				 case 0x61: all_flash_unlock();   break;               //PLC存储内存清理 应为下发了多次 我在这FLASH全部解锁
				 case 0x60: ErasurePLC(1);        break;               //PLC存储内存清理	
				 case 0x63: ErasurePLC(2);        break;               //PLC清理数据元件位	
				 case 0x62: ErasurePLC(3);        break;               //PLC清理位元件	
			   default: tx_data[1]=0x15,tx_count=1; break;           //遇到不支持的命令
			 } 
		}
}

void READ_plc_programCodeBuf()                              //读16位数据包括只读
{
  u8 temp_sum; 
  tx_data[1]=0x02;                               //报文开始
  temp_sum=0;
	tx_data[2]=Ascll[plc_programCodeBuf[0]/0x10];             //取字节高位
  tx_data[3]=Ascll[plc_programCodeBuf[0]%0x10];             //取字节低位
	tx_data[4]=Ascll[plc_programCodeBuf[1]/0x10];             //取字节高位
  tx_data[5]=Ascll[plc_programCodeBuf[1]%0x10];             //取字节低位
	temp_sum+=tx_data[2]+tx_data[3]+tx_data[4]+tx_data[5];
  tx_data[6]=0x03;                               // 结束
  temp_sum+=0x03;
  tx_data[7]=Ascll[temp_sum/0x10];
  tx_data[8]=Ascll[temp_sum%0x10]; 
  tx_count=8;
}

void PLC_1BIT_State()                                           //读取 PLC工作状态  启动 
{
	u8 temp_sum;
	tx_data[1]=0x02;
	temp_sum=0;
	if(PLC_BIT_TEST(PLC_Com_BIT(tx_data[5]*0X100+tx_data[4])))    //读取位寄存器地址
	tx_data[2]=0x31;
	else 
	tx_data[2]=0x30;
	tx_data[3]=0x03;
	temp_sum+=tx_data[2];
	temp_sum+=tx_data[3];
  tx_data[4]=Ascll[temp_sum/0x10];
  tx_data[5]=Ascll[temp_sum%0x10]; 
	tx_count=5;
}	

void PLC_Download(void)  //查看寄存器 状态
{
	 u16 temp;
	 temp=tx_data[3];                              //计算数据操作起始地址
	 switch(temp) 
	 { 
		  case 0x11: READ_plc_programCodeBuf();          break;  //读容量	
		  case 0x10: PLC_1BIT_State();        break;  //读取PLC运行状态
		  default: tx_data[1]=0x15,tx_count=1;break;  //遇到不支持的命令
	 } 
}

void PLC_Upload(void)           //查看程序梯形图结束地址
{
	u16 *PLC_Addr,temp;
	u8 temp_sum,data_H,data_L;
	temp=0X805A;                  //
  PLC_Addr=((u16*)(0x800605A));
  do{PLC_Addr++;temp+=2;}
	while(!(*PLC_Addr==0x000f));  //如果END 就0X000F	 
	tx_data[1]=0x02;              //报文开始
  temp_sum=0;
  tx_data[2]=0x31;
	temp_sum+=tx_data[2];
	data_H=temp/0x100;            //
	data_L=temp%0x100;            //
	tx_data[3]=Ascll[data_H/0X10];
	tx_data[4]=Ascll[data_H%0X10];
	tx_data[5]=Ascll[data_L/0X10];
	tx_data[6]=Ascll[data_L%0X10];
  tx_data[7]=0X03;
	temp_sum+=tx_data[3];
	temp_sum+=tx_data[4];
	temp_sum+=tx_data[5];
	temp_sum+=tx_data[6];
	temp_sum+=tx_data[7];
	tx_data[8]=Ascll[temp_sum/0x10];
  tx_data[9]=Ascll[temp_sum%0x10]; 
	tx_count=9;
}

void PLC_E_Expand(void)                             //扩展功能 "F" 上传程序和下载程序调用
{ 	
	 u16 temp;
	 temp=rx_data[4];                                 //计算数据操作起始地址
	 setup_LH();			                                //调用算出地址函数
	 switch(temp) 
	 { 
		 case 0X35: PLC_Download();           break;    //下载 程序时调用
		 case 0X38: PLC_Upload();             break;    //查找END指令地址
		 case 0x43: online_write_data();      break;    //在线写程序	
		 default: tx_data[1]=0x06,tx_count=1; break;    //遇到不支持的命令
	 } 
}

void find_end(void)		   //查找程序中是否存在END指令，
{
	if(rx_count==13)
			tx_data[1]=0x06,tx_count=1;  
	else
			tx_data[1]=0x06,tx_count=1; 
}

void Process_switch(void)
{            
	   u8 temp;
     switch_read_data();                                          //把第三位开始的ASCII码转换成HEX	，地址为数据发送区
		 temp=rx_data[3];
		 switch(temp) 
		 { 
					case 0x30: data_size=tx_data[4],PC_READ_byte();  break; //代入请求据数长度位“tx_data[4]”    读数据
			    case 0x31: data_size=tx_data[4];PC_WRITE_byte(); break; //代入请求据数长度位“tx_data[4]”    写数据
			    case 0x34: find_end();                           break; //查找指令，如查找到有数据则返回6
			    case 0x37: setup_HL(),PC_FORCE_ON();             break; //PLC	启动 远程 “0x37”
			    case 0x38: setup_HL(),PC_FORCE_OFF();            break; //PLC 停止 远程 “0x38”
			    case 0x42: all_flash_lock();                     break; //写参数结束命令
			    case 0x46: PLC_E_Expand();                       break; //通讯F功能指令 
			    case 0x45: data_size=tx_data[5],PC_OPTION_PROG();break; //通讯E功能指令 
			    default:	                                       break;
		}                                                                                                                        
	   if((tx_count==0)&&(rx_count==0))	                            //return error code for 0x15
	 tx_data[1]=0x15,tx_count=1;			 		
	rx_count=0;Send_out=1;                                     //报告上位机	
}		

void usart(u32 DEFAULT_BAUD)
{
	  USART_InitTypeDef USART_InitStructure;                          //串口配置寄存器	
	  USART_InitStructure.USART_BaudRate = DEFAULT_BAUD;              //设置波特率
	  USART_InitStructure.USART_WordLength = USART_WordLength_8b;     //8位数据位
	  USART_InitStructure.USART_StopBits = USART_StopBits_1;          //一位停止位
	  USART_InitStructure.USART_Parity = USART_Parity_Even;           //校验位 
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1,&USART_InitStructure);                        //初始化串口   
}  

//---------------------串口功能配置---------------------
void USART1_Configuration(void) 
{
	DMA_InitTypeDef  DMA_InitStructure;                           //DMA初始化结构体
	GPIO_InitTypeDef GPIO_InitStructure;                          //串口引脚配置寄存器
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 , ENABLE);       //打开串口对应的外设时钟  
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);            //启动DMA时钟
	DMA_DeInit(DMA1_Channel4);                                    //DMA1通道4配置
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART1->DR);//外设地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)tx_data+1;        //内存地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;            //dma传输方向单向
	DMA_InitStructure.DMA_BufferSize = PROGRAM_BUF_SIZE;                       //设置DMA在传输时缓冲区的长度
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//设置DMA的外设递增模式，一个外设
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;       //设置DMA的内存递增模式
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据字长
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;//内存数据字长
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                 //设置DMA的传输模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;           //设置DMA的优先级别
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                  //设置DMA的2个memory中的变量互相访问
	DMA_Init(DMA1_Channel4,&DMA_InitStructure);
	DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,ENABLE);
	usart(19200);                                //初始化参数 
	//TXE发送中断,TC传输完成中断,RXNE接收中断,PE奇偶错误中断,可以是多个   
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);  
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);//采用DMA方式发送
	USART_Cmd(USART1, ENABLE);                  //启动串口    
	
	//*********************串口1的管脚初始化 ****************************************   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;                       //管脚9  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;               //选择GPIO响应速度  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                 //复用推挽输出  
	GPIO_Init(GPIOA, &GPIO_InitStructure);                          //TX初始化  
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;                      //管脚10  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;           //浮空输入  
	GPIO_Init(GPIOA, &GPIO_InitStructure);                          //RX初始化                                                      
}

void DMA1_Channel4_IRQHandler(void)                 //串口1DMA方式发送中断
{
	DMA1->IFCR |= DMA1_FLAG_TC4;                      //清除标志位
	DMA1_Channel4->CCR &= 0xFFFE;                     //关闭DMA
	Flag_Uart_Send = 1;                               //允许再次发送
}

void  TX_Process(void) // 发送串口数据
{  
	if(Flag_Uart_Send)
	{
		Send_out=Flag_Uart_Send = 0;
		DMA1_Channel4->CNDTR = tx_count;// 设置传输数据长度
		DMA1_Channel4->CCR |= 0x0001;   // 打开DMA
	}
}


void RX_Process(void)                                //接收串口数据
{   
	static u8 sum,f=1;
	rx_data[0]=0x7f&USART1->DR;

	if(rx_data[0]==0X05)		                            //上位机提出通讯请求
	{
		rx_count=0;
		
		tx_data[1]=0x06;
		tx_count=1;
		TX_Process(); //报告上位机并且返回0X06应答	
		
		 gCommLedFlashFlg =2; // 闪烁
	} 
	else if(rx_data[0]==0X02)	                          //报文开始
	{
		rx_count=0x01;
	}
	else if(rx_count==0)		
	{
		if(f==1)
		{
			usart(19200);
			rx_count=0,tx_data[1]=0x06,tx_count=1,TX_Process();//报告上位机并且返回0X06应答	 
			f=0;
		}
		else if(f==0)
		{
			usart(9600);
			rx_count=0,tx_data[1]=0x06,tx_count=1,TX_Process();//报告上位机并且返回0X06应答	 	 
			f=1;
		}		
	}
	if(rx_count>0)		                              //确认报文开始
	{
		rx_count++; 
		rx_data[rx_count]=rx_data[0];
		if(rx_count > PROGRAM_BUF_SIZE)	             //读取数据误差大于610
		{
			tx_count=0;
			rx_count=0;
		}
		if((rx_count>3)&&(rx_data[rx_count-2]==0x03))	 //数据是否传送结束
		{ 
			sum = CheckSum(rx_data);            
			if((rx_data[rx_count-1] == Ascll[sum/0x10])&&(rx_data[rx_count] == Ascll[sum%0x10]))// 计算数据和状态 数据是否正常
			{
				Process_switch(); 
			}
			else 
			{
				tx_data[1]=0x15;
				tx_count=1;
				TX_Process();     //报告上位机并且回答数据异常返值0X15
			}
			
			gCommLedFlashFlg =2;  // 闪烁
		}
	}	   
}

void USART1_IRQHandler(void)                         //中断调用
{ 
	if(USART_GetITStatus(USART1,USART_IT_RXNE)==SET)  //接收中断
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
		RX_Process();
	}
	//溢出-如果发生溢出需要先读SR,再读DR寄存器则可清除不断入中断的问题
	if(USART_GetFlagStatus(USART1,USART_FLAG_ORE)==SET)
	{
		USART_ClearFlag(USART1,USART_FLAG_ORE);         //读SR其实就是清除标志
		USART_ReceiveData(USART1);                      //读DR
	}
	if(USART_GetITStatus(USART1, USART_IT_TXE)==SET)
	{
		USART_ClearITPendingBit(USART1,USART_IT_TXE);
		USART_ITConfig(USART1,USART_IT_TXE,DISABLE);
	}
	if(USART_GetITStatus(USART1, USART_IT_TC)==SET)  //发送中断
	{
	  USART_ClearITPendingBit(USART1,USART_IT_TC);
	}
}



