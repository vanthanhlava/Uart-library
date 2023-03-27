 /********************************************************/
// CPUÐèÒª£ºSTM32F103--RAMÄÚ´æ²»Ð¡ÓÚ64K	FlashÄÚ´æ²»Ð¡ÓÚ128K
// ±¾´úÂëÒÑÔÚSTM32F103RDT6¡¢VET6²âÊÔÍ¨¹ý
// ±à¼­ÈÕÆÚ£º20150909
// editor by ´«ÈË¼Ç
// Íøµê£ºhttps://shop178536326.taobao.com
/********************************************************
PLCÏà¹ØµÄÌØÊâ¼Ä´æÆ÷
×¨ÓÃ¸¨Öú¼ÌµçÆ÷           ÃèÊö
M8126                   È«¾Ö±êÖ¾
M8127                   Í¨Ñ¶ÇëÇóÎÕÊÖÐÅºÅ
M8128                   ³ö´í±êÖ¾ 
M8129                   Í¨Ñ¶ÇëÇóÇÐ»»        
   
×¨ÓÃÊý¾Ý¼Ä´æÆ÷           ÃèÊö
D8000 = 200;		   		  É¨ÃèÊ±¼ä
D8001 = 0X5EF6;	        ÐÍºÅ°æ±¾ FX2N(C)
D8101 = 0X5EF6;	        ÐÍºÅ°æ±¾ FX2N(C)
D8002 = 8;			   		  ÄÚ´æÈÝÁ¿
D8102 = 8;			   		  ÄÚ´æÈÝÁ¿
D8003 = 0x0010; 	   	  ´æ´¢ÀàÐÍ: PLCÄÚÖÃ´æ´¢
D8006                   CPUµç³ØµçÑ¹
D8010 = 10;			   	    É¨Ãèµ±Ç°Öµ
D8011 = 20;			   	    É¨Ãè×îÐ¡Ê±¼ä(0.1MS)
D8012 = 140;			   	  É¨Ãè×î³¤Ê±¼ä(0.1MS) 
D6030 D6031 D6032 D6033       ÊÇÄ£ÄâÁ¿ÊäÈë
D8080 D8081                   ÊÇÄ£ÄâÊä³ö



D8120 = 0X4096                Í¨Ñ¶¸ñÊ½
D8121                         ´ÓÕ¾ºÅ£¨×î¶à16¸ö£©
D8127                         ½»»»Êý¾ÝµÄÊ×µØÖ·
D8128                         ½»»»Êý¾ÝÁ¿
D8129                         ÍøÂçÍ¨Ñ¶³¬Ê±Ê±¼äÈ·ÈÏÖµ
D8000                         ¿´ÃÅ¹·         
D8019                         ¶ÔÓ¦ÐÇÆÚ
D8018                         ¶ÔÓ¦Äê·Ý
D8017                         ¶ÔÓ¦ÔÂ·Ý
D8016                         ¶ÔÓ¦ÈÕÆÚ
D8015                         ¶ÔÓ¦Ð¡Ê±
D8014                         ¶ÔÓ¦·ÖÖÓ
D8013                         ¶ÔÓ¦Ãë


Í¨Ñ¶¸ñÊ½Ïê½â£¨D8120£©
----------------------------------------------------------------------
Î»ºÅ	     |   º¬ Òå	   |          ÃèÊö      
-----------+-------------+--------------------------------------------
b0	       |  Êý¾Ý³¤¶È	 |   0£º 7Î»   1£º 8Î»
-----------+-------------+--------------------------------------------
b2b1	     |  Ð£Ñé·½Ê½   |   00£ºÎÞÐ£Ñé  01£ºÆæÐ£Ñé  11£ºÅ¼Ð£Ñé
-----------+-------------+--------------------------------------------
b3	       |   Í£Ö¹Î»	   |   0£º 1Î»   1£º 2Î»
-----------+-------------+--------------------------------------------
           |             |   0001£º300      0111£º4800    1011£º56000
b7b6b5b4   |   ²¨ÌØÂÊ	   |   0100£º600      1000£º9600    1100£º57600
           |             |   0101£º1200     1001£º19200   1101£º115200
           |             |   0110£º2400     1010£º38400   
-----------+-------------+--------------------------------------------
b8    		 |   ±¨Í·      |  0£ºÎÞ(MODBUS)£»1:ÓÐ[½öRS,D8124³õÊ¼Öµ£ºSTX(02H)]
-----------+-------------+--------------------------------------------
b9    		 |   ±¨Î²      |  0:ÎÞ(MODBUS); 1:ÓÐ[½öRS,D8125³õÊ¼Öµ£ºETX(03H)]
-----------+-------------+--------------------------------------------
b10~b12 	 |             |  Ô¤Áô
-----------+-------------+--------------------------------------------
b13	       |   ºÍÐ£Ñé    |   0£º²»¸½¼Ó(MODBUS)£»1£º¸½¼Ó(RS)
-----------+-------------+--------------------------------------------
b14	       |   Ð­Òé      |   0£º×¨ÓÃÐ­Òé(MODBUS)£»1£ºÎÞÐ­Òé(RS)
-----------+-------------+--------------------------------------------
b15	       | ½öMODBUS    |   0£º´Ó»ú  1£ºÖ÷»ú			 
----------------------------------------------------------------------

¾ÙÀý£ºD8120 = 0X4096           Í¨Ñ¶²¨ÌØÂÊÊÇ19200
*********************************************************************************/

#include "stm32f10x.h"
#include "stm32f10x_flash.h"
#include <stdio.h>
#include "PLC_Dialogue.h"
#include "PLC_IO.h"
#include "plc_conf.h"
#include "bsp_timer.h"

const u8   plc_programCodeBuf[34000] __at (PLC_RAM_ADDR)={						
//FLASHÆðÊ¼µØÖ·ÎªPLCÐÅÏ¢**************************×îÇ°µÄ0X02±íÊ¾PLCÎª16KµÄ³ÌÐò²½,ÃÜÂëÇøÓòºÍ²îÊýÇøÓò****************
0x10,0x00,0xD8,0xBA,0x00,0x00,0x00,0x00,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,
0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,
0x20,0x20,0xF4,0x09,0xFF,0x0B,0xF4,0x01,0xE7,0x03,0x64,0x0E,0xC7,0x0E,0xDC,0x0E,0xFF,0x0E,0x90,0x01,0xFE,0x03,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x83,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0X0F,0X00,//½áÊøÖ¸Áî
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,};
	

//D8000 = 200;		   		  É¨ÃèÊ±¼ä
//D8001 = 0X5EF6;	        ÐÍºÅ°æ±¾ 
//D8101 = 0X5EF6;	        ÐÍºÅ°æ±¾ 
//D8002 = 10;			   		  ÄÚ´æÈÝÁ¿ 16K
//D8102 = 10;			   		  ÄÚ´æÈÝÁ¿ 16000²
//D8003 = 0x0010; 	   	  ´æ´¢ÀàÐÍ: PLCÄÚÖÃ´æ´¢
//D8006                   CPUµç³ØµçÑ¹
//D8010 = 10;			   	    É¨Ãèµ±Ç°Öµ
//D8011 = 20;			   	    É¨Ãè×îÐ¡Ê±¼ä(0.1MS)
//D8012 = 140;			   	  É¨Ãè×î³¤Ê±¼ä(0.1MS) 

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
static u8 Flag_Uart_Send=1;             //·¢ËÍ±êÖ¾Î»
u16 rx_count,tx_count;                  //Êý¾Ý¼ÆËã       



char tx_data[PROGRAM_BUF_SIZE];         //·¢ËÍ»º´æ
char rx_data[PROGRAM_BUF_SIZE];         //½ÓÊÕ»º´æ

u16 prog_address,data_address;          //¼ÆËãÊý¾Ý²Ù×÷ÆðÊ¼µØÖ·»º´æ


u16 plc_16BitBuf[PLC_16BIT_BUF_SIZE] __at (RAM_ADDR);   //PLC_RAMÔËÐÐÊý¾Ý

// 0x200066B0~0x2000D200 // 26KB
// ´«ÈË¼Ç£¬20170424ÐÞ¸Ä
u8  step_status[1000];  
u8  step_address[2000]; //Ð´²½×´Ì¬Îª0ÉÏÉýÑÓÓëÏÂ½µÑØÊ¹ÓÃ¹²2K×Ö½Ú1600bit

u8  data_size,block_contol[2];													
extern u8  edit_prog;
extern void RTC_Set(u16 syear,u8 smon,u8 sday,u8 hour,u8 min,u8 sec);	
												
u8 Send_out;
u8 Write_Pro_flag = 0;


void data_init(void)//D8000~D8126³õÊ¼»¯
{                                                 
	u16 temp;                                       
	u16 temp_address;                                
	
	prog_address=0x52;                                
  for(temp=0;temp<126;temp++)                      
  {                                                                   
       temp_address=0x0C00;                        
	   plc_16BitBuf[temp_address+temp]=special_d[temp]; //´«ÈëÏµÍ³FLASH±¸·ÝµÄÓÃ»§Êý¾Ý
  }                                                //
	// D8000
	plc_16BitBuf[0x2000] =plc_programCodeBuf[prog_address] *256; // È¡×Ö½Ú¸ßÎ»                      
	plc_16BitBuf[0x2000] |=plc_programCodeBuf[prog_address+1]; // È¡×Ö½ÚµÍÎ»
	
	PLC_RW_RAM_8BIT(0x01E0) =0x09; // ÁîM8000 M8003ÖÃON
	
	block_contol[0]=200; // ·ÀÖ¹Ð´²ÎÊýÊ±³ÌÐò¶ªÊ§
	block_contol[1]=200;                            
	
#if DEBUG
	  #warning "MCºÍMCRÖ¸Áî--data_init";
//	  MC.MC_SFR = 0;
//	  MC.MC_Flg = 0;
//	  MC.PLC_MC_BIT = 0;
#endif
	
}                                                  

void write_block(u16 number)                                                  // Ð´ÈëFLASH
{ 
	u16 temp,wait_write,appoint_address;
	if(number<17)			                                                          // Ð´Èë²ÎÊýµÄ¿é±ØÐè10¿é
	{
		FLASH_Unlock();		                                                        // flash¹Ø±Õ±£»¤
		FLASH_ErasePage(PLC_RAM_ADDR+number*0x800);                               // ²Á³ýÒ»¿éÊý¾ÝÕ¼ÓÃ2K
		for(temp=0;temp<1024;temp++)	                                            // ²Ù×÷Îª16bit,Ö»ÐèÒª1024´Î³ÌÐò²Ù×÷Íê±Ï
		{
			appoint_address =PLC_RAM_ADDR + number*0x800 + temp*2;                  // ÆðÊ¼µØÖ·¼ÓÉÏ¿éµØÖ·ÔÙ¼ÓÉÏ¿éµÄÐ¡µØÖ·,µÈÓÚÄ¿±êÎ»ÖÃ 
			wait_write =progWriteBuf[temp*2] + progWriteBuf[temp*2+1]*0X100; // Ð´Èë16bitÖÁflash
			FLASH_ProgramHalfWord(appoint_address,wait_write);                      // µÈ´ý³ÌÐòÐ´Èë½áÊø
		}
		FLASH_Lock();	                                                            // ½áÊø³ÌÐòÐ´Èë¿ªÆôflash±£»¤
	}
}

void backup_block(u16 number)		                                              // ³ÌÐò¿é±¸·Ý,Ä¿µÄÔÚÐ´³ÌÐòÖ®Ç°½øÇ°Ãæ³ÌÐò±¸·Ý
{
	u16 temp,appoint_address;
	if(number<17)
	{
		for(temp=0;temp<2048;temp++)
		{
			appoint_address =number*0x800 + temp;                                    // ÆðÊ¼µØÖ·¼ÓÉÏ¿éµØÖ·ÔÙ¼ÓÉÏ¿éµÄÐ¡µØÖ· 
			progWriteBuf[temp] =plc_programCodeBuf[appoint_address];	           // ½«³ÌÐò±¸·Ý³öÀ´
		}
	}
}

//=======================================================================================================
// º¯ÊýÃû³Æ: ErasurePLC
// ¹¦ÄÜÃèÊö£ºPLC²Á³ýFLASH¿Õ¼ä
// Êä¡¡Èë:  mode Ä£Ê½       
// Êä¡¡³ö:  void     
// È«¾Ö±äÁ¿:  
// µ÷ÓÃÄ£¿é: 
// ×÷¡¡Õß:  ´«ÈË¼Ç
// ÈÕ¡¡ÆÚ:  2014Äê5ÔÂ18ÈÕ
// ±¸  ×¢:  
//-------------------------------------------------------------------------------------------------------
// ÐÞ¸ÄÈË:
// ÈÕ¡¡ÆÚ:
// ±¸  ×¢: 
//-------------------------------------------------------------------------------------------------------
//=======================================================================================================
void ErasurePLC(u8 mode)
{
	u16 temp=0,Erasure_plc_16BitBuf;
	/*******************************************PLC´æ´¢ÄÚ´æÇåÀí	************************************************/
	if(mode==1)
	{
		backup_block(0);
		progWriteBuf[92]=0x0f;                                                         //¸³Öµ
		progWriteBuf[93]=0x00;                                                         //¸³Öµ
		for(temp=94;temp<2048;temp++)                                                       //´Ó0x5E
		{ 
			progWriteBuf[temp]=0xffff;                                                    //
		}
		FLASH_Unlock();                                                                     //flash¹Ø±Õ±£»¤                     
		FLASH_ErasePage(PLC_RAM_ADDR+0x800);                                            
		write_block(0);                                                                     //±¸·ÝµÚÒ»¿éflash
		for(temp=1;temp<10;temp++)                                                          //²Á³ý10¿é
		FLASH_ErasePage(PLC_RAM_ADDR+temp*0x800);                                           //²Á³ýflash
		FLASH_Lock();                                                                       //½áÊø³ÌÐòÐ´Èë¿ªÆôflash±£»¤
	}
/*******************************************PLCÇåÀíÊý¾ÝÔª¼þÎ»*********************************************/
   if(mode==2)
	 {
	   for(Erasure_plc_16BitBuf=0x4000;Erasure_plc_16BitBuf<0x7E7E;Erasure_plc_16BitBuf+=2)           // Çå³ýD0000-D7999
       plc_16BitBuf[Erasure_plc_16BitBuf]=0x00;
	 }
/*******************************************PLCÇåÀíÎ»Ôª¼þ**************************************************/
	 if(mode==3)
	 {
	      for(Erasure_plc_16BitBuf=0x0000;Erasure_plc_16BitBuf<0x00BE;Erasure_plc_16BitBuf+=2)	    // Çå³ýM0000-M3071
        plc_16BitBuf[Erasure_plc_16BitBuf]=0x00;
	 }
   tx_data[1]=0x06,tx_count=1,Send_out=5;                                                 // Çå³ýÍê±Ï±¨¸æÉÏÎ»»ú
}
				
/*******************************************************************************
º¯Êý¹¦ÄÜ£º¼ÆËãÐ£ÑéºÍ 
±¸×¢£º 20171102£¬´«ÈË¼ÇÓÅ»¯
*******************************************************************************/
u8 CheckSum(char * pBuf)//¼ÆËã½ÓÊÕÇøºÍÐ£Ñé
{ 
	u16 i; 
	u8 sum;
	sum=0;		// Çë³ýºÍ¼ÇËãÆ÷
	pBuf +=3;	// ¼ÆËãºÍ´ÓµÚÈýÎ»¿ªÊ¼
	for(i=3;i<(rx_count-1);i++) // ¼ÆËãºÍ
	{ 
		sum += *pBuf; // ¿ªÊ¼Ïà¼Ó
		pBuf++;				// Ö¸Õë¼ÓÒ»
	}
	return sum;			// Êý¾ÝÕý³£	
}

/*******************************************************************************
º¯ÊýÃû³Æ£ºvoid switch_read_data(void)  
º¯Êý¹¦ÄÜ£º×ª»»ASCIIÂëÎªHEXÂë£¬Õ¼ÓÃÊý¾Ý·¢ËÍ¼Ä´æÆ÷         
³ö¿Ú²ÎÊý£ºÎÞ
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

void setup_HL(void)	                     //¸ßµÍÎ»½»»»ÔÙ×ª»» £¬Ð¡¶Ë×ª»»
{                                                                     
	u8 temp;                                                            
	temp=tx_data[3];				               //µØÖ·¸ßÎ»ËÍÈë16Î»Êý¾ÝÇø     
	prog_address=temp*0x100+tx_data[2];    //¼ÆËã³ÌÐò²Ù×÷ÆðÊ¼µØÖ·       
}	                                                                     

void setup_LH(void)	                     //Õý³£µØÖ·×ª»»
{ 
	u8 temp;
	temp=tx_data[3];				              //µØÖ·¸ßÎ»ËÍÈë16Î»Êý¾ÝÇø
	data_address=temp*0x100+tx_data[4];  //¼ÆËãÊý¾Ý²Ù×÷ÆðÊ¼µØÖ·
}

typedef union                           
{
	int data;
	char data1[2];
} usart_data;

// ´«ÈË¼Ç£¬20161122×¢ÊÍ£¬//¶ÁÈ¡PLCÐÍºÅ£¬Ö¸Áî¡°30¡±
void read_plc_tyte(u8 addr)                                  
{
	u16 temp;
	u8 temp_sum; 
	usart_data plc_type;
	
	plc_type.data=special_d[addr];                             //PLCÐÍºÅ
	tx_data[1]=0x02;                                           //±¨ÎÄ¿ªÊ¼	02
	temp_sum=0;
	for(temp=0;temp<data_size;temp++)
	{ 
		tx_data[temp*2+2]=Ascll[plc_type.data1[temp]/0x10]; //È¡×Ö½Ú¸ßÎ»
		tx_data[temp*2+3]=Ascll[plc_type.data1[temp]%0x10]; //È¡×Ö½ÚµÍÎ»
		temp_sum+=tx_data[temp*2+2]+tx_data[temp*2+3];
	}
	tx_data[temp*2+2]=0x03;                                     //±¨ÎÄ½áÊø	03
	temp_sum+=0x03;
	tx_data[temp*2+3]=Ascll[temp_sum/0x10];
	tx_data[temp*2+4]=Ascll[temp_sum%0x10]; 
	tx_count=temp*2+4;
}


/*******************************************************************************
º¯ÊýÃû³Æ£PPLC_Comm_Byte
º¯Êý¹¦ÄÜ£ºÍ¨ÐÅ×Ö½ÚµØÖ·ÖØ¶¨Òå¼°Ö´ÐÐ
Èë¿Ú²ÎÊý£ºCmdÃüÁî          
³ö¿Ú²ÎÊý£ºÓ³ÉäµÄÊµ¼ÊµØÖ·(16BITµØÖ·)
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
	else if(comm_add>=0x0200&&comm_add<=0x021F) // T00-T255  OVER´¥µã
			return comm_add-0x0140;
	else if(comm_add>=0x0500&&comm_add<=0x051F) // T00-T255  Enable ÏßÈ¦
			return comm_add-0x0240; 
	else if(comm_add>=0x01E0&&comm_add<=0x01FF) // C00-C255  OVER´¥µã
			return comm_add-0x0020;
	else if(comm_add<=0x00BF) // M0000-M1535 
			return comm_add+0x0100;
	else if(comm_add>=0x00C0&&comm_add<=0x017F) // M1536-M3071
			return comm_add+0x0440;
	else if(comm_add>=0x01C0&&comm_add<=0x01DF) // M8000-M8255 
			return comm_add+0x0020;
	else // ÎÞÐ§µØÖ·
			return comm_add;                                              
}

/*******************************************************************************
º¯ÊýÃû³Æ£ºPLC_Com_BIT
º¯Êý¹¦ÄÜ£ºÍ¨ÐÅÎ»µØÖ·ÖØ¶¨Òå¼°Ö´ÐÐ
Èë¿Ú²ÎÊý£ºCmdÃüÁî          
³ö¿Ú²ÎÊý£ºÓ³ÉäµÄÊµ¼ÊµØÖ·(BITµØÖ·)
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
 else if((addr>=0x9800)&&(addr<=0x99FF)) // T00-T255 Enable ÏßÈ¦ *
	 return (addr-0X7200); 
 else if((addr>=0x9700)&&(addr<=0x97FF)) // C00-C255 Enable ÏßÈ¦ *
	 return (addr-0X7200); 
 else if((addr>=0x6300)&&(addr<=0x64FF)) // T00-T255 OVER   ´¥µã *
	 return (addr-0X5D00); 
 else if((addr>=0x6200)&&(addr<=0x62FF)) // C00-C255	OVER   ´¥µã	*
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
		return (addr-0X1200); //T00-T255 Enable ÏßÈ¦
	else if((addr>=0x1000)&&(addr<=0x10FF)) 
		return (addr-0X0A00); //T00-T255 OVER   ´¥µã
	else if((addr>=0x0E00)&&(addr<=0x0EFF)) 
		return (addr+0X0100); //M8000-M8255
	else if((addr>=0x0F00)&&(addr<=0x0FFF)) 
		return (addr-0X0100); //C00-C255 OVER´¥µã
	else  return addr;
}


//=======================================================================================================
// º¯ÊýÃû³Æ:  void READ_data(void)
// ¹¦ÄÜÃèÊö£º ¶ÁÊý¾Ý  X,Y,M,S,T,C,D   
// Êä¡¡Èë:   void  
// Êä¡¡³ö:   void   
// È«¾Ö±äÁ¿:  
// µ÷ÓÃÄ£¿é: 
// ×÷¡¡Õß:  ´«ÈË¼Ç
// ÈÕ¡¡ÆÚ:  2014Äê6ÔÂ10ÈÕ
// ±¸  ×¢:  
//-------------------------------------------------------------------------------------------------------
// ÐÞ¸ÄÈË:
// ÈÕ¡¡ÆÚ:
// ±¸  ×¢:  ´«ÈË¼Ç£¬20161122×¢ÊÍ£¬PLC_Comm_Byte»ñÈ¡FX0N±à³Ì¿ÚÍ¨Ñ¶·ÃÎÊÊ±µÄÄÚ²¿ÈíÔª¼þµØÖ·£¬S X Y T M C M8£¬ ¡¾£¿ T C16 C32 D8 D D1000 ¡¿
//=======================================================================================================
void read_other_data(void)	                                       //Ö¸Áî¡°30¡±
{
	u16 i;
	u8 temp_sum;
	tx_data[1]=0x02;                                                 //±¨ÎÄ¿ªÊ¼
	temp_sum=0;
	for(i=0;i<data_size;i++)
	{ 
		tx_data[i*2 + 2]=Ascll[PLC_RW_RAM_8BIT(PLC_Comm_Byte(i + prog_address))/0x10];//È¡×Ö½Ú¸ßÎ»
		tx_data[i*2 + 3]=Ascll[PLC_RW_RAM_8BIT(PLC_Comm_Byte(i + prog_address))%0x10];//È¡×Ö½ÚµÍÎ»
		temp_sum+=tx_data[i*2+2]+tx_data[i*2+3];
	}
	tx_data[i*2 + 2]=0x03;                                          //±¨ÎÄ½áÊø	03
	temp_sum+=0x03;
	tx_data[i*2 + 3]=Ascll[temp_sum/0x10];
	tx_data[i*2 + 4]=Ascll[temp_sum%0x10]; 
	tx_count=i*2+4;
}

//=======================================================================================================
// º¯ÊýÃû³Æ:  void READ_data(void)
// ¹¦ÄÜÃèÊö£º Ð´Êý¾Ý  X,Y,M,S,T,C,D   
// Êä¡¡Èë:   void  
// Êä¡¡³ö:   void   
// È«¾Ö±äÁ¿:  
// µ÷ÓÃÄ£¿é: 
// ×÷¡¡Õß:  ´«ÈË¼Ç
// ÈÕ¡¡ÆÚ:  2014Äê6ÔÂ10ÈÕ
// ±¸  ×¢:  
//-------------------------------------------------------------------------------------------------------
// ÐÞ¸ÄÈË:
// ÈÕ¡¡ÆÚ:
// ±¸  ×¢: 
//=======================================================================================================
void PC_WRITE_byte(void)                       //Ð´×Ö
{ 
	u16 temp;
	prog_address=tx_data[2]*0x100+tx_data[3]+4;//¼ÆËãÊý¾Ý²Ù×÷ÆðÊ¼µØÖ·
	for(temp=0;temp<data_size;temp++)
	{
		PLC_RW_RAM_8BIT(PLC_Comm_Byte(temp+prog_address))=tx_data[5+temp];
	}
	tx_data[1]=0x06,tx_count=1;                //±¨¸æÉÏÎ»»ú
}

//=======================================================================================================
// º¯ÊýÃû³Æ:  void FORCE_ON_data(void)   
// ¹¦ÄÜÃèÊö£º FORCE ON  X,Y,M,S,T,C
// Êä¡¡Èë:   void  
// Êä¡¡³ö:   void   
// È«¾Ö±äÁ¿:  
// µ÷ÓÃÄ£¿é: 
// ×÷¡¡Õß:  ´«ÈË¼Ç
// ÈÕ¡¡ÆÚ:  2014Äê6ÔÂ10ÈÕ
// ±¸  ×¢:  
//-------------------------------------------------------------------------------------------------------
// ÐÞ¸ÄÈË:
// ÈÕ¡¡ÆÚ:
// ±¸  ×¢: 
//=======================================================================================================
void PC_FORCE_ON(void)	                                //Ç¿ÖÆ  38 ON
{ 
	PLC_BIT_ON(PLC_Com_BIT(tx_data[2]*0x100+tx_data[3]));//¼ÆËãÊý¾Ý²Ù×÷ÆðÊ¼µØÖ·
	tx_data[1]=0x06,tx_count=1;  
}

//======================================================================================================
// º¯ÊýÃû³Æ:  void FORCE_ON_data(void)   
// ¹¦ÄÜÃèÊö£º FORCE OFF  X,Y,M,S,T,C
// Êä¡¡Èë:   void  
// Êä¡¡³ö:   void   
// È«¾Ö±äÁ¿:  
// µ÷ÓÃÄ£¿é: 
// ×÷¡¡Õß:  ´«ÈË¼Ç
// ÈÕ¡¡ÆÚ:  2014Äê6ÔÂ10ÈÕ
// ±¸  ×¢:  
//------------------------------------------------------------------------------------------------------
// ÐÞ¸ÄÈË:
// ÈÕ¡¡ÆÚ:
// ±¸  ×¢: 
//=======================================================================================================
void PC_FORCE_OFF(void)	                      //Ç¿ÖÆ  37  OFF 
{ 
	PLC_BIT_OFF(PLC_Com_BIT(tx_data[2]*0x100+tx_data[3]));//¼ÆËãÊý¾Ý²Ù×÷ÆðÊ¼µØÖ·
	tx_data[1]=0x06,tx_count=1;   
}

void PC_READ_byte(void)	                        //¶Á×Ö
{
	prog_address=tx_data[2]*0x100+tx_data[3];     //¼ÆËãÊý¾Ý²Ù×÷ÆðÊ¼µØÖ·
	switch(prog_address)
	{ 
		case 0x0ECA: read_plc_tyte(101);  break;  //¶ÁPLC TYPE      ´«ÈË¼Ç£¬20161122×¢ÊÍ£¬D8101
		case 0x0E02: read_plc_tyte(1);    break;  //¶ÁPLC TYPE      ´«ÈË¼Ç£¬20161122×¢ÊÍ£¬D8001
		default: read_other_data();       break;
	}
}


void EPC_FORCE_ON(void)	        //Ê¹ÓÃÀ©Õ¹¹¦ÄÜ"E"Ç¿ÖÆON
{  
   switch(prog_address)
   {
			case 0x6023: PLC_RW_RAM_8BIT(0X01E0)=0x09;  break;  // Ô¶³Ì²Ù×÷ÇëÇóÊÇ·ñ¿ÉÒÔ½øÐÐ
	    case 0x6024: // Ô¶³Ì²Ù×÷ÐèÒªÔËÐÐ
			{
				PLC_RW_RAM_8BIT(0X01E0)=0x09;  
				Write_Pro_flag = 0;  
			  break;  
			}
	    case 0x6025: PLC_RW_RAM_8BIT(0X01E0)=0x0A;  break;  // Ô¶³Ì²Ù×÷ÐèÒªÍ£Ö¹
      default:     PLC_BIT_ON(PLC_Com_BIT(prog_address));break;  // ÆäËü²Ù×÷ÇøÓò
	 }
   tx_data[1]=0x06,tx_count=1;
}

void EPC_FORCE_OFF(void)	  //Ê¹ÓÃÀ©Õ¹¹¦ÄÜ"E"Ç¿ÖÆOFF 
{ 
  PLC_BIT_OFF(PLC_Com_BIT(prog_address));
  tx_data[1]=0x06,tx_count=1;
}  

void PC_READ_Parameter(void)                                  //¶ÁÅäÖÃ	  E00
{
  u16 temp,temp_bit,temp_addr,mov_bit,temp1;
  u8 temp_sum;
  u8 send,monitor,monitor1,monitor2; 
  tx_data[1]=0x02;                                             //±¨ÎÄ¿ªÊ¼
  temp_sum=0;
  prog_address=PLC_Comm_Byte(tx_data[3]*0x100+tx_data[4]);       //¼ÆËãÊý¾Ý²Ù×÷ÆðÊ¼µØÖ·
  if((prog_address==0x1790)||(prog_address==0x17D0))             //ÇëÇó¶Á¼à¿ØÊý¾ÝÇø0X1790 Óë 0X17D0µØÖ·
  {
		  if(prog_address==0x1790)
      {
   	     monitor1=PLC_RAM8(0x20002400);	                         //¶Á³öÐèÒª¼à¿Ø×ÖÊýÁ¿
	       for(temp1=monitor=0;monitor<monitor1;monitor++)         //¶Á¼à¿Ø×ÖÊý¾Ý
	       { 
					  temp_bit=PLC_Comm_Byte(plc_16BitBuf[0X0A02+monitor])/2;
					  if(0x7FE0==temp_bit)
						{ 
							monitor++;
						  temp_bit=plc_16BitBuf[0X0A02+monitor];
							plc_16BitBuf[0x0BC8+temp1]=plc_16BitBuf[temp_bit];temp1++;
						}
						else
				    plc_16BitBuf[0x0BC8+temp1]=plc_16BitBuf[temp_bit];temp1++;//½«ÐèÒªµÄÊý¾Ý´«µ½»º´æ		
						
					  if((temp_bit>=0x600)&&(temp_bit<=0x66E))             //Ö÷ÒªÊÇC200-C255ÒÔºóµÄµØÖ·ÊÇ32Î»µÄ 
					  {   
						  plc_16BitBuf[0x1790/2+temp1]=plc_16BitBuf[temp_bit+1];   //½«ÐèÒªµÄÊý¾Ý´«µ½»º´æ
						  temp1++;
					  }							
		     }
	       monitor2=PLC_RAM8(0x20002402);                          //¶Á³öÐèÒª¼à¿ØÎ»ÊýÁ¿
				 temp1=temp1+16;                                         //
	       for(monitor1=0;monitor1<monitor2;monitor1++)            //¶Á¼à¿ØÎ»Êý¾Ý
	       {  
						temp_addr=PLC_Com_BIT(plc_16BitBuf[0x0A02+monitor+monitor1]);
		        temp_bit=plc_16BitBuf[temp_addr/0x10];
		        mov_bit = temp_addr%0x10;
		        if((temp_bit&(1<<mov_bit))==(1<<mov_bit))
		        plc_16BitBuf[0x0BC8+temp1+monitor1/0x10]|=1<<(monitor1%0x10);//´®¿Ú»º´æÊý¾Ý	
			      else plc_16BitBuf[0x0BC8+temp1+monitor1/0x10]&=~(1<<(monitor1%0x10));	      					
		     }
	     }
   }
	 for(temp=0;temp<data_size;temp++)	    //¶ÁRAM
   { 
			send=PLC_RW_RAM_8BIT(prog_address+temp);
	    tx_data[temp*2+2]=Ascll[send/0x10]; //È¡×Ö½Ú¸ßÎ»
      tx_data[temp*2+3]=Ascll[send%0x10]; //È¡×Ö½ÚµÍÎ»
	    temp_sum+=tx_data[temp*2+2]+tx_data[temp*2+3];
   }
   tx_data[temp*2+2]=0x03;    //
   temp_sum+=0x03;
   tx_data[temp*2+3]=Ascll[temp_sum/0x10];
   tx_data[temp*2+4]=Ascll[temp_sum%0x10]; 
   tx_count=temp*2+4;
}

void PC_WRITE_Parameter(void)                //Ð´ÅäÖÃ	  E10
{  
	u16 temp;
	prog_address=tx_data[3]*0x100+tx_data[4];  //¼ÆËãÊý¾Ý²Ù×÷ÆðÊ¼µØÖ·
	for(temp=0;temp<data_size;temp++)	         //write  RAM
	{
	    PLC_RW_RAM_8BIT(PLC_Comm_Byte(prog_address+temp))=tx_data[6+temp];
	}
	tx_data[1]=0x06,tx_count=1; //±¨¸æÉÏÎ»»ú
}
 

void PC_READ_PORG(void)	          //¶Á³ÌÐòE01
{
	u16 temp;
	u8 temp_sum; 
	
	if(0x805C < data_address)
	{
		tx_data[1]=0x15;
		tx_count=1; 
		return;
	}	
	
	tx_data[1]=0x02;                 //±¨ÎÄ¿ªÊ¼
	temp_sum=0;
	data_address-=0x8000;	           //¶ÁFLASH µØÖ·¼õ0x8000µÈÓÚÊµ¼ÊÎ»ÖÃ
	for(temp=0;temp<data_size;temp++)
	{ 
		tx_data[temp*2+2]=Ascll[plc_programCodeBuf[data_address+temp]/0x10]; //È¡×Ö½Ú¸ßÎ»
		tx_data[temp*2+3]=Ascll[plc_programCodeBuf[data_address+temp]%0x10]; //È¡×Ö½ÚµÍÎ»
		temp_sum+=tx_data[temp*2+2]+tx_data[temp*2+3];
	}
	tx_data[temp*2+2]=0x03;  
	temp_sum+=0x03;
	tx_data[temp*2+3]=Ascll[temp_sum/0x10];
	tx_data[temp*2+4]=Ascll[temp_sum%0x10]; 
	tx_count=temp*2+4;
}

//  ÌÝÐÎÍ¼ÏÂÔØ
void PC_WRITE_PORG(void)	                           //Ð´³ÌÐò E11 
{  
	 u16 i;
   prog_address=tx_data[3]*0x100+tx_data[4];         //¼ÆËãÊý¾Ý²Ù×÷ÆðÊ¼µØÖ·
	 
	 PLC_RW_RAM_8BIT(0X01E0)=0x0A; 
	 Write_Pro_flag = 1;			   //·ÀÖ¹ÔÚÏÂÔØ¹ý³ÌÖÐ³ÌÐòÔËÐÐ ADD ´«ÈË¼Ç
	
	 edit_prog=0;                                      //°Ñ³ÌÐò±à¼­Çå³ý,ÒòÎªÐ´³ÌÐòÊ±¿ÉÄÜ´æÔÚPµØÖ··¢Éú±ä»¯£¬PLCÓ¦¼ÆËãPµØÖ·
	 prog_address-=0x8000;                             //µØÖ·¼õ0X8000µÈÓÚÊµ¼ÊÎ»ÖÃ
	 for(i=0;i<data_size;i++)
	 {
		    block_contol[0]=(prog_address+i)/0x800;	 //Ã¿Ò»¿éÕ¼ÓÃµÄµØÖ· 0X800£½2K ×Ö½Ú
	      if(block_contol[0]==block_contol[1])			   //ÊÇ·ñÐèÒªÌø¿é£¬
	      {
			    progWriteBuf[(prog_address+i)-block_contol[0]*0x800]=tx_data[6+i];  //½«Êý¾ÝÐ´Èë»º´æÖÐ
	      }
		    else							                           //ÐèÒªÌø¿é´¦Àí
		    {
			    write_block(block_contol[1]);              //½«Ç°Ò»¿éÐ´Èëµ½FLASH
		      backup_block(block_contol[0]);             //±¸·ÝÐèÒªÐ´µÄFLASH¿é
		      block_contol[1]=block_contol[0];
		      progWriteBuf[(prog_address+i)-block_contol[0]*0x800]=tx_data[6+i];
		    }
   }
   tx_data[1]=0x06,tx_count=1;
}

static u16 find_data(u16 addr,u16 find_data)             //²éÕÒÊý¾ÝµØÖ·£¬²¢·µ»ØÕÒµ½µÄÊý¾ÝµØÖ·
{
	 u8 find_ok,data_H,data_L;
   find_ok=5;
   data_H=find_data/0x100;
   data_L=find_data%0x100;
	 addr-=0x8000;
   do{
	      if((plc_programCodeBuf[addr]==data_L)&&(plc_programCodeBuf[addr+1]==data_H))
	      find_ok=0;		                                   //ÕÒµ½ÐèÒªµÄÖ¸Áî
	      else
	      addr+=2;
		    if(addr>(0xFD5C-0x8000))
		    find_ok=1;                                       //ÔÚÓÐÐ§µÄ·¶Î§ÄÚÃ»ÓÐÕÒµ½ENDÖ¸Áî
	   }while(find_ok>3);
	 
	return addr+0X8000;
}
 
static void find_data_address(void)       //²éÕÒÉÏÎ»»úÐèÒªµÄÖ¸ÁîµØÖ·
{ 
	   u8 temp_sum,data_H,data_L;  
		 data_L=tx_data[5];	           //ÐèÒª²éÕÒÊý¾ÝµÄÄÚÈÝµÍÎ»	
     data_H=tx_data[6];            //ÐèÒª²éÕÒÊý¾ÝµÄÄÚÈÝ¸ßÎ»
	   data_address=find_data(data_address,data_H*0X100+data_L);	//¶ÁFLASH
	   tx_data[1]=0x02;              //±¨ÎÄ¿ªÊ¼
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
  end_addr=find_data(addr+0x8000,0x000f)+mov_addr-0x8000; //ÕÒ³öENDÖ¸ÁîËùÔÚÎ»ÖÃ£¬ÔÙ¼ÓÉÏÒÆ¶¯Á¿£¬µÈÓÚ×îµ½ENDËùÔÚÎ»ÖÃ
  start_addr=end_addr;
  addr_mov=addr;   
  if(addr>0x5B)            //µ±Ç°ÒÆ¶¯ÊÇ·ñ´óÓÚ0²½
	{
		  addr_mov-=0X5C;	     //¼õÈ¥²ÎÊýµØÖ·
	    end_addr-=0x5C;
	    addr_mov/=2;		     //Ò»²½Õ¼ÓÃÁ½¸ö×Ö½Ú¡¡³ý2ÎªÖ¸µ½ÆðÊ¼×Ö½Ú
	    end_addr/=2;
		  addr_mov/=8;			   //Ö¸ÁîÏòÃ¿Ò»²½µÄ×³Ì¬µØÖ·
	    end_addr/=8;
	    offset/=2;
	    mov_byte=offset/8;
	    offset%=8;
 	    while(!(end_addr==addr_mov))	                               //END address and curennt addr  ÊÇ·ñÏàµÈ	 ,²»ÏàµÈÐèÒªÑ­»·
	    {
	     temp=step_address[end_addr]*0x100+step_address[end_addr-1]; //µ÷³öÇ°16Î»µÄ×³Ì¬Öµ£¬ÎªºóÃæÐèÒªÒÆ¶¯
		   temp<<=offset;                                              //ÏòºóÒÆ¶¯Æ«ÒÆµÄµØÖ·ÓàÊý
		   step_address[end_addr+mov_byte]=temp/0x100;		             //¸½Öµµ½Ä¿±êµØÖ·
		   end_addr--;							                                   //µØÖ·ÏòÇ°ÒÆ8²½
		  }
      temp=step_address[end_addr]*0x100+step_address[end_addr-1];	 //µ÷³öÇ°16Î»µÄ×³Ì¬Öµ£¬ÎªºóÃæÐèÒªÒÆ¶¯
      temp<<=offset;                                               //ÏòºóÒÆ¶¯Æ«ÒÆµÄµØÖ·ÓàÊý
      step_address[end_addr+mov_byte]=temp/0x100;		               //¸½Öµµ½Ä¿±êµØÖ·	    
	}
	end_addr=start_addr;   
	temp=start_addr;
do{
     if((end_addr/0x800)==(addr/0x800))                              //Ä¿±êÒÆ¶¯Î»ÖÃÓëÊµ¼ÊÒÆ¶¯Î»ÖÃµ½ÁËÒ»¸ö¿éÂð
     start_addr=addr%0x800;			                                     //Èç¹ûÒÑµ½ÁËÒ»¸ö¿éÄÇÃ´ÆðÊ¼µØÖ·ÎªÄ¿±êÎ»ÖÃ¿ªÄ£
	   else
     start_addr=0;					                                         //Ã»µ½Ôò´Ó0¿ªÊ¼ÒÆ¶¯
     if((temp/0x800)==(end_addr/0x800))                              //ENDËùÔÚµÄµØÖ·£¬¸úÕýÔÚÒÆ¶¯µÄµØÖ·ÊÇÔÚÒ»¸ö¿éÂð£¬Èç¹ûÔÚÒ»¸ö¿é£¬ÔòÐèÒª½øÐÐ½áÊø¶È¶È´¦Àí
     temp1=end_addr%0x800+1; 
	   else
	   temp1=2048;					                                           //Èç¹ûÃ»ÓÐÔÚÒ»¸ö¿éÖ±½Óµ½Ò»¸ö¿éµÄ½áÊøÎ»ÖÃ2048
	   backup_block(end_addr/0x800);
	   for(temp2=start_addr;temp2<temp1+1;temp2++)
	   {
		   backup_addr=(end_addr/0x800)*0x800+temp2-mov_addr;            //µØÖ·ÏòÇ°Æ«ÒÆÐèÒªÒÆ¶¯µÄÁ¿
	     progWriteBuf[temp2]=plc_programCodeBuf[backup_addr];			  	       //±¸·ÝµØÖ·
	   }
	   write_block(end_addr/0x800);                                    //½«ÒÆ¶¯¹ýµÄÊý¾ÝÐ´µ½FLASH
	   end_addr-=(temp1-start_addr);                                   //Ëã³öÏÖÔÚÒÆ¶¯ºóµÄÎ»ÖÃ  
	 }while(end_addr>addr+mov_addr);   
}


void online_write_data(void)                             //ÔÚÏßÐ´³ÌÐò
{  
	 static u16 temp,Size;
   signed short temp1,temp2;
   temp1=tx_data[6];                                      //´æÈëµÄÊÇÔÚ¶àÉÙ¸öÊý¾ÝÐ´
	 temp2=tx_data[8];                                      //ÐèÒªÐ´Èë¶àÉÙ¸ö×Ö½Ú
	 temp2-=temp1;                                          //¼ÆËãÒ»¿éµØÖ· 
	 if(temp2>0)                                            //Èç¹ûÐ´ÈëµÄÊý³¤¶È´óÓÚÊµ¼ÊµÄÊý¾Ý³¤¶È£¬ÔòÐèÒªÅ²¶¯flashµØÖ·
	 {
	   mov_flash(data_address-0x8000,temp2); //ÐèÒªÅ²¶¯flashµÄµØÖ·£¬ÒÔ¼°Å²¶¯µÄ³¤¶È
	 }                
   edit_prog=0;                                           //°Ñ³ÌÐò±à¼­Çå³ý,ÒòÎªÐ´³ÌÐòÊ±¿ÉÄÜ´æÔÚPµØÖ··¢Éú±ä»¯£¬PLCÓ¦¼ÆËãPµØÖ· 
	 
	 block_contol[0]=100;
	 block_contol[1]=100;
     prog_address=(tx_data[3]*0x100+tx_data[4])-0x8000;     //¼ÆËãÊý¾Ý²Ù×÷ÆðÊ¼µØÖ·
     data_size=tx_data[8];
	 for(temp=0;temp<data_size;temp++)
	 {
		     block_contol[0]=(prog_address+temp)/0x800;	      //Ã¿Ò»¿éÕ¼ÓÃµÄµØÖ· 0X800£½2K ×Ö½Ú
	       if(block_contol[0]==block_contol[1])			        //ÊÇ·ñÐèÒªÌø¿é´¦Àí
	       {progWriteBuf[(prog_address+temp)-block_contol[0]*0x800]=tx_data[9+temp];}  //½«Êý¾ÝÐ´Èë»º´æÖÐ
		     else							                                //ÐèÒªÌø¿é´¦Àí
		     {
			    write_block(block_contol[1]);                   //½«Ç°Ò»¿éÐ´Èëµ½FLASH
		      backup_block(block_contol[0]);                  //±¸·ÝÐèÒªÐ´µÄFLASH¿é
		      block_contol[1]=block_contol[0];
		      progWriteBuf[(prog_address+temp)-block_contol[0]*0x800]=tx_data[9+temp];
		     }
	  }
	  write_block(block_contol[0]);                         //½«Ç°Ò»¿éÐ´Èëµ½FLASH  °Ñ´®¿ÚÏÂ·¢³ÌÐòÐ´Èëplc_programCodeBuf 
    if(temp2<0)	                                          //É¾³ý³ÌÐòÊýÁ¿
		{    
			   temp2=0-temp2;                                   //¼ÆËãÉ¾³ý³ÌÐòÊýÁ¿
			   Size=find_data(0x8000,0x000f)-prog_address;      //°Ñ ENDµØÖ·-ÆðÊ¼²Ù×÷µØÖ·=²Ù×÷ÊýÁ¿
		   	 for(;temp<Size;temp++)                           //temp ±£ÁôÉÏ Ð´Èë×´Ì¬
	       {
		        block_contol[0]=(prog_address+temp)/0x800;	  //Ã¿Ò»¿éÕ¼ÓÃµÄµØÖ· 0X800£½2K ×Ö½Ú
	          if(block_contol[0]==block_contol[1])			    //ÊÇ·ñÐèÒªÌø¿é´¦Àí
	          {
			       progWriteBuf[(prog_address+temp)-block_contol[0]*0x800]=plc_programCodeBuf[prog_address+temp+temp2];  //½«Êý¾ÝÐ´Èë»º´æÖÐ
	          }
		        else							                            //ÐèÒªÌø¿é´¦Àí
		        {
			        write_block(block_contol[1]);               //½«Ç°Ò»¿éÐ´Èëµ½FLASH
		          backup_block(block_contol[0]);              //±¸·ÝÐèÒªÐ´µÄFLASH¿é
		          block_contol[1]=block_contol[0];            //
		          progWriteBuf[(prog_address+temp)-block_contol[0]*0x800]=plc_programCodeBuf[prog_address+temp+temp2];
		        }
	       }   
				 write_block(block_contol[0]);                    //½«Ç°Ò»¿éÐ´Èëµ½FLASH   
		}
    tx_data[1]=0x06,tx_count=1;
}

void all_flash_unlock(void)                               //FLASHÈ«²¿½âËø
{
  block_contol[1]=200;
  block_contol[0]=200;
  tx_data[1]=0x06,tx_count=1;
}

void all_flash_lock(void) 	                                  //FLASHÈ«²¿¼ÓËø
{
  write_block(block_contol[1]);                               //¼ÓËøÖ®Ç°°ÑÐèÒªÐ´µÄÊý¾ÝÐ´µ½FLASH
  block_contol[1]=200;
  block_contol[0]=200;
  FLASH_Lock();
  tx_data[1]=0x06,tx_count=1;
}

void PC_OPTION_PROG(void)                                      //À©Õ¹¹¦ÄÜ "E" ´úÂë
{ 	
	 u16 temp;
	 if((rx_count==11)&&((rx_data[4]==0x37)||(rx_data[4]==0x38)))//ÊÇ·ñÎªÇ¿ÖÆ¹¦ÄÜ 
	 {	
         prog_address=hex[rx_data[5]]*0x10+hex[rx_data[6]]+hex[rx_data[7]]*0x1000+hex[rx_data[8]]*0x100;
	     if(rx_data[4]==0x37) 
		   EPC_FORCE_ON();  
		   else
		   EPC_FORCE_OFF();
		}
		else
	  {
			 setup_LH();			                                      //µ÷ÓÃËã³öµØÖ·º¯Êý
	     temp=tx_data[2];
	     switch(temp) 
		   { 
				 case 0x00: PC_READ_Parameter();  break;               //¶ÁÅäÖÃ E00
			case 0x10: //Ð´ÅäÖÃ E10
			{
				// ´«ÈË¼Ç£¬20180524£¬ÐÂÔögxwork2Ö±½ÓÉèÖÃÊ±ÖÓ£¬Ó¦¸ÃÐèÒªÌØÊâ´¦Àí£¿
				// E10 0E1A[d8013] 0E24[d8018] 0010[·Ö] 0011[Ê±] 0018[ÈÕ] 0005[ÔÂ] 00 E207[Äê] 04[ÐÇÆÚ] 00
				
				PC_WRITE_Parameter(); 				
			  break;               
			}
			case 0x01: // ¶Á³ÌÐò E01
			{
				PC_READ_PORG();       
			  break; 
      }				
			   case 0x11: PC_WRITE_PORG();      break;               //Ð´³ÌÐò E11 
			   case 0x77: all_flash_unlock();   break;               //Ê¹ÓÃEÖ¸Áî½øÐÐÐ´³ÌÐòÐ´ÇëÇó77
			   case 0x87: all_flash_lock();     break;               //Ê¹ÓÃEÖ¸Áî½øÐÐÐ´³ÌÐò½áÊøÇëÇó87
			   case 0x41: find_data_address();  break;               //²éÕÒENDÖ¸ÁîµØÖ·	
				 case 0x61: all_flash_unlock();   break;               //PLC´æ´¢ÄÚ´æÇåÀí Ó¦ÎªÏÂ·¢ÁË¶à´Î ÎÒÔÚÕâFLASHÈ«²¿½âËø
				 case 0x60: ErasurePLC(1);        break;               //PLC´æ´¢ÄÚ´æÇåÀí	
				 case 0x63: ErasurePLC(2);        break;               //PLCÇåÀíÊý¾ÝÔª¼þÎ»	
				 case 0x62: ErasurePLC(3);        break;               //PLCÇåÀíÎ»Ôª¼þ	
			   default: tx_data[1]=0x15,tx_count=1; break;           //Óöµ½²»Ö§³ÖµÄÃüÁî
			 } 
		}
}

void READ_plc_programCodeBuf()                              //¶Á16Î»Êý¾Ý°üÀ¨Ö»¶Á
{
  u8 temp_sum; 
  tx_data[1]=0x02;                               //±¨ÎÄ¿ªÊ¼
  temp_sum=0;
	tx_data[2]=Ascll[plc_programCodeBuf[0]/0x10];             //È¡×Ö½Ú¸ßÎ»
  tx_data[3]=Ascll[plc_programCodeBuf[0]%0x10];             //È¡×Ö½ÚµÍÎ»
	tx_data[4]=Ascll[plc_programCodeBuf[1]/0x10];             //È¡×Ö½Ú¸ßÎ»
  tx_data[5]=Ascll[plc_programCodeBuf[1]%0x10];             //È¡×Ö½ÚµÍÎ»
	temp_sum+=tx_data[2]+tx_data[3]+tx_data[4]+tx_data[5];
  tx_data[6]=0x03;                               // ½áÊø
  temp_sum+=0x03;
  tx_data[7]=Ascll[temp_sum/0x10];
  tx_data[8]=Ascll[temp_sum%0x10]; 
  tx_count=8;
}

void PLC_1BIT_State()                                           //¶ÁÈ¡ PLC¹¤×÷×´Ì¬  Æô¶¯ 
{
	u8 temp_sum;
	tx_data[1]=0x02;
	temp_sum=0;
	if(PLC_BIT_TEST(PLC_Com_BIT(tx_data[5]*0X100+tx_data[4])))    //¶ÁÈ¡Î»¼Ä´æÆ÷µØÖ·
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

void PLC_Download(void)  //²é¿´¼Ä´æÆ÷ ×´Ì¬
{
	 u16 temp;
	 temp=tx_data[3];                              //¼ÆËãÊý¾Ý²Ù×÷ÆðÊ¼µØÖ·
	 switch(temp) 
	 { 
		  case 0x11: READ_plc_programCodeBuf();          break;  //¶ÁÈÝÁ¿	
		  case 0x10: PLC_1BIT_State();        break;  //¶ÁÈ¡PLCÔËÐÐ×´Ì¬
		  default: tx_data[1]=0x15,tx_count=1;break;  //Óöµ½²»Ö§³ÖµÄÃüÁî
	 } 
}

void PLC_Upload(void)           //²é¿´³ÌÐòÌÝÐÎÍ¼½áÊøµØÖ·
{
	u16 *PLC_Addr,temp;
	u8 temp_sum,data_H,data_L;
	temp=0X805A;                  //
  PLC_Addr=((u16*)(0x800605A));
  do{PLC_Addr++;temp+=2;}
	while(!(*PLC_Addr==0x000f));  //Èç¹ûEND ¾Í0X000F	 
	tx_data[1]=0x02;              //±¨ÎÄ¿ªÊ¼
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

void PLC_E_Expand(void)                             //À©Õ¹¹¦ÄÜ "F" ÉÏ´«³ÌÐòºÍÏÂÔØ³ÌÐòµ÷ÓÃ
{ 	
	 u16 temp;
	 temp=rx_data[4];                                 //¼ÆËãÊý¾Ý²Ù×÷ÆðÊ¼µØÖ·
	 setup_LH();			                                //µ÷ÓÃËã³öµØÖ·º¯Êý
	 switch(temp) 
	 { 
		 case 0X35: PLC_Download();           break;    //ÏÂÔØ ³ÌÐòÊ±µ÷ÓÃ
		 case 0X38: PLC_Upload();             break;    //²éÕÒENDÖ¸ÁîµØÖ·
		 case 0x43: online_write_data();      break;    //ÔÚÏßÐ´³ÌÐò	
		 default: tx_data[1]=0x06,tx_count=1; break;    //Óöµ½²»Ö§³ÖµÄÃüÁî
	 } 
}

void find_end(void)		   //²éÕÒ³ÌÐòÖÐÊÇ·ñ´æÔÚENDÖ¸Áî£¬
{
	if(rx_count==13)
			tx_data[1]=0x06,tx_count=1;  
	else
			tx_data[1]=0x06,tx_count=1; 
}

void Process_switch(void)
{            
	   u8 temp;
     switch_read_data();                                          //°ÑµÚÈýÎ»¿ªÊ¼µÄASCIIÂë×ª»»³ÉHEX	£¬µØÖ·ÎªÊý¾Ý·¢ËÍÇø
		 temp=rx_data[3];
		 switch(temp) 
		 { 
					case 0x30: data_size=tx_data[4],PC_READ_byte();  break; //´úÈëÇëÇó¾ÝÊý³¤¶ÈÎ»¡°tx_data[4]¡±    ¶ÁÊý¾Ý
			    case 0x31: data_size=tx_data[4];PC_WRITE_byte(); break; //´úÈëÇëÇó¾ÝÊý³¤¶ÈÎ»¡°tx_data[4]¡±    Ð´Êý¾Ý
			    case 0x34: find_end();                           break; //²éÕÒÖ¸Áî£¬Èç²éÕÒµ½ÓÐÊý¾ÝÔò·µ»Ø6
			    case 0x37: setup_HL(),PC_FORCE_ON();             break; //PLC	Æô¶¯ Ô¶³Ì ¡°0x37¡±
			    case 0x38: setup_HL(),PC_FORCE_OFF();            break; //PLC Í£Ö¹ Ô¶³Ì ¡°0x38¡±
			    case 0x42: all_flash_lock();                     break; //Ð´²ÎÊý½áÊøÃüÁî
			    case 0x46: PLC_E_Expand();                       break; //Í¨Ñ¶F¹¦ÄÜÖ¸Áî 
			    case 0x45: data_size=tx_data[5],PC_OPTION_PROG();break; //Í¨Ñ¶E¹¦ÄÜÖ¸Áî 
			    default:	                                       break;
		}                                                                                                                        
	   if((tx_count==0)&&(rx_count==0))	                            //return error code for 0x15
	 tx_data[1]=0x15,tx_count=1;			 		
	rx_count=0;Send_out=1;                                     //±¨¸æÉÏÎ»»ú	
}		

void usart(u32 DEFAULT_BAUD)
{
	  USART_InitTypeDef USART_InitStructure;                          //´®¿ÚÅäÖÃ¼Ä´æÆ÷	
	  USART_InitStructure.USART_BaudRate = DEFAULT_BAUD;              //ÉèÖÃ²¨ÌØÂÊ
	  USART_InitStructure.USART_WordLength = USART_WordLength_8b;     //8Î»Êý¾ÝÎ»
	  USART_InitStructure.USART_StopBits = USART_StopBits_1;          //Ò»Î»Í£Ö¹Î»
	  USART_InitStructure.USART_Parity = USART_Parity_Even;           //Ð£ÑéÎ» 
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1,&USART_InitStructure);                        //³õÊ¼»¯´®¿Ú   
}  

//---------------------´®¿Ú¹¦ÄÜÅäÖÃ---------------------
void USART1_Configuration(void) 
{
	DMA_InitTypeDef  DMA_InitStructure;                           //DMA³õÊ¼»¯½á¹¹Ìå
	GPIO_InitTypeDef GPIO_InitStructure;                          //´®¿ÚÒý½ÅÅäÖÃ¼Ä´æÆ÷
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 , ENABLE);       //´ò¿ª´®¿Ú¶ÔÓ¦µÄÍâÉèÊ±ÖÓ  
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);            //Æô¶¯DMAÊ±ÖÓ
	DMA_DeInit(DMA1_Channel4);                                    //DMA1Í¨µÀ4ÅäÖÃ
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART1->DR);//ÍâÉèµØÖ·
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)tx_data+1;        //ÄÚ´æµØÖ·
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;            //dma´«Êä·½Ïòµ¥Ïò
	DMA_InitStructure.DMA_BufferSize = PROGRAM_BUF_SIZE;                       //ÉèÖÃDMAÔÚ´«ÊäÊ±»º³åÇøµÄ³¤¶È
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//ÉèÖÃDMAµÄÍâÉèµÝÔöÄ£Ê½£¬Ò»¸öÍâÉè
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;       //ÉèÖÃDMAµÄÄÚ´æµÝÔöÄ£Ê½
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//ÍâÉèÊý¾Ý×Ö³¤
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;//ÄÚ´æÊý¾Ý×Ö³¤
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                 //ÉèÖÃDMAµÄ´«ÊäÄ£Ê½
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;           //ÉèÖÃDMAµÄÓÅÏÈ¼¶±ð
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                  //ÉèÖÃDMAµÄ2¸ömemoryÖÐµÄ±äÁ¿»¥Ïà·ÃÎÊ
	DMA_Init(DMA1_Channel4,&DMA_InitStructure);
	DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,ENABLE);
	usart(19200);                                //³õÊ¼»¯²ÎÊý 
	//TXE·¢ËÍÖÐ¶Ï,TC´«ÊäÍê³ÉÖÐ¶Ï,RXNE½ÓÊÕÖÐ¶Ï,PEÆæÅ¼´íÎóÖÐ¶Ï,¿ÉÒÔÊÇ¶à¸ö   
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);  
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);//²ÉÓÃDMA·½Ê½·¢ËÍ
	USART_Cmd(USART1, ENABLE);                  //Æô¶¯´®¿Ú    
	
	//*********************´®¿Ú1µÄ¹Ü½Å³õÊ¼»¯ ****************************************   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;                       //¹Ü½Å9  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;               //Ñ¡ÔñGPIOÏìÓ¦ËÙ¶È  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                 //¸´ÓÃÍÆÍìÊä³ö  
	GPIO_Init(GPIOA, &GPIO_InitStructure);                          //TX³õÊ¼»¯  
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;                      //¹Ü½Å10  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;           //¸¡¿ÕÊäÈë  
	GPIO_Init(GPIOA, &GPIO_InitStructure);                          //RX³õÊ¼»¯                                                      
}

void DMA1_Channel4_IRQHandler(void)                 //´®¿Ú1DMA·½Ê½·¢ËÍÖÐ¶Ï
{
	DMA1->IFCR |= DMA1_FLAG_TC4;                      //Çå³ý±êÖ¾Î»
	DMA1_Channel4->CCR &= 0xFFFE;                     //¹Ø±ÕDMA
	Flag_Uart_Send = 1;                               //ÔÊÐíÔÙ´Î·¢ËÍ
}

void  TX_Process(void) // ·¢ËÍ´®¿ÚÊý¾Ý
{  
	if(Flag_Uart_Send)
	{
		Send_out=Flag_Uart_Send = 0;
		DMA1_Channel4->CNDTR = tx_count;// ÉèÖÃ´«ÊäÊý¾Ý³¤¶È
		DMA1_Channel4->CCR |= 0x0001;   // ´ò¿ªDMA
	}
}


void RX_Process(void)                                //½ÓÊÕ´®¿ÚÊý¾Ý
{   
	static u8 sum,f=1;
	rx_data[0]=0x7f&USART1->DR;

	if(rx_data[0]==0X05)		                            //ÉÏÎ»»úÌá³öÍ¨Ñ¶ÇëÇó
	{
		rx_count=0;
		
		tx_data[1]=0x06;
		tx_count=1;
		TX_Process(); //±¨¸æÉÏÎ»»ú²¢ÇÒ·µ»Ø0X06Ó¦´ð	
		
		 gCommLedFlashFlg =2; // ÉÁË¸
	} 
	else if(rx_data[0]==0X02)	                          //±¨ÎÄ¿ªÊ¼
	{
		rx_count=0x01;
	}
	else if(rx_count==0)		
	{
		if(f==1)
		{
			usart(19200);
			rx_count=0,tx_data[1]=0x06,tx_count=1,TX_Process();//±¨¸æÉÏÎ»»ú²¢ÇÒ·µ»Ø0X06Ó¦´ð	 
			f=0;
		}
		else if(f==0)
		{
			usart(9600);
			rx_count=0,tx_data[1]=0x06,tx_count=1,TX_Process();//±¨¸æÉÏÎ»»ú²¢ÇÒ·µ»Ø0X06Ó¦´ð	 	 
			f=1;
		}		
	}
	if(rx_count>0)		                              //È·ÈÏ±¨ÎÄ¿ªÊ¼
	{
		rx_count++; 
		rx_data[rx_count]=rx_data[0];
		if(rx_count > PROGRAM_BUF_SIZE)	             //¶ÁÈ¡Êý¾ÝÎó²î´óÓÚ610
		{
			tx_count=0;
			rx_count=0;
		}
		if((rx_count>3)&&(rx_data[rx_count-2]==0x03))	 //Êý¾ÝÊÇ·ñ´«ËÍ½áÊø
		{ 
			sum = CheckSum(rx_data);            
			if((rx_data[rx_count-1] == Ascll[sum/0x10])&&(rx_data[rx_count] == Ascll[sum%0x10]))// ¼ÆËãÊý¾ÝºÍ×´Ì¬ Êý¾ÝÊÇ·ñÕý³£
			{
				Process_switch(); 
			}
			else 
			{
				tx_data[1]=0x15;
				tx_count=1;
				TX_Process();     //±¨¸æÉÏÎ»»ú²¢ÇÒ»Ø´ðÊý¾ÝÒì³£·µÖµ0X15
			}
			
			gCommLedFlashFlg =2;  // ÉÁË¸
		}
	}	   
}

void USART1_IRQHandler(void)                         //ÖÐ¶Ïµ÷ÓÃ
{ 
	if(USART_GetITStatus(USART1,USART_IT_RXNE)==SET)  //½ÓÊÕÖÐ¶Ï
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
		RX_Process();
	}
	//Òç³ö-Èç¹û·¢ÉúÒç³öÐèÒªÏÈ¶ÁSR,ÔÙ¶ÁDR¼Ä´æÆ÷Ôò¿ÉÇå³ý²»¶ÏÈëÖÐ¶ÏµÄÎÊÌâ
	if(USART_GetFlagStatus(USART1,USART_FLAG_ORE)==SET)
	{
		USART_ClearFlag(USART1,USART_FLAG_ORE);         //¶ÁSRÆäÊµ¾ÍÊÇÇå³ý±êÖ¾
		USART_ReceiveData(USART1);                      //¶ÁDR
	}
	if(USART_GetITStatus(USART1, USART_IT_TXE)==SET)
	{
		USART_ClearITPendingBit(USART1,USART_IT_TXE);
		USART_ITConfig(USART1,USART_IT_TXE,DISABLE);
	}
	if(USART_GetITStatus(USART1, USART_IT_TC)==SET)  //·¢ËÍÖÐ¶Ï
	{
	  USART_ClearITPendingBit(USART1,USART_IT_TC);
	}
}



