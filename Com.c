 /********************************************************/
// CPU��Ҫ��STM32F103--RAM�ڴ治С��64K	Flash�ڴ治С��128K
// ����������STM32F103RDT6��VET6����ͨ��
// �༭���ڣ�20150909
// editor by ���˼�
// ���꣺https://shop178536326.taobao.com
/********************************************************
PLC��ص�����Ĵ���
ר�ø����̵���           ����
M8126                   ȫ�ֱ�־
M8127                   ͨѶ���������ź�
M8128                   ������־ 
M8129                   ͨѶ�����л�        
   
ר�����ݼĴ���           ����
D8000 = 200;		   		  ɨ��ʱ��
D8001 = 0X5EF6;	        �ͺŰ汾 FX2N(C)
D8101 = 0X5EF6;	        �ͺŰ汾 FX2N(C)
D8002 = 8;			   		  �ڴ�����
D8102 = 8;			   		  �ڴ�����
D8003 = 0x0010; 	   	  �洢����: PLC���ô洢
D8006                   CPU��ص�ѹ
D8010 = 10;			   	    ɨ�赱ǰֵ
D8011 = 20;			   	    ɨ����Сʱ��(0.1MS)
D8012 = 140;			   	  ɨ���ʱ��(0.1MS) 
D6030 D6031 D6032 D6033       ��ģ��������
D8080 D8081                   ��ģ�����



D8120 = 0X4096                ͨѶ��ʽ
D8121                         ��վ�ţ����16����
D8127                         �������ݵ��׵�ַ
D8128                         ����������
D8129                         ����ͨѶ��ʱʱ��ȷ��ֵ
D8000                         ���Ź�         
D8019                         ��Ӧ����
D8018                         ��Ӧ���
D8017                         ��Ӧ�·�
D8016                         ��Ӧ����
D8015                         ��ӦСʱ
D8014                         ��Ӧ����
D8013                         ��Ӧ��


ͨѶ��ʽ��⣨D8120��
----------------------------------------------------------------------
λ��	     |   �� ��	   |          ����      
-----------+-------------+--------------------------------------------
b0	       |  ���ݳ���	 |   0�� 7λ   1�� 8λ
-----------+-------------+--------------------------------------------
b2b1	     |  У�鷽ʽ   |   00����У��  01����У��  11��żУ��
-----------+-------------+--------------------------------------------
b3	       |   ֹͣλ	   |   0�� 1λ   1�� 2λ
-----------+-------------+--------------------------------------------
           |             |   0001��300      0111��4800    1011��56000
b7b6b5b4   |   ������	   |   0100��600      1000��9600    1100��57600
           |             |   0101��1200     1001��19200   1101��115200
           |             |   0110��2400     1010��38400   
-----------+-------------+--------------------------------------------
b8    		 |   ��ͷ      |  0����(MODBUS)��1:��[��RS,D8124��ʼֵ��STX(02H)]
-----------+-------------+--------------------------------------------
b9    		 |   ��β      |  0:��(MODBUS); 1:��[��RS,D8125��ʼֵ��ETX(03H)]
-----------+-------------+--------------------------------------------
b10~b12 	 |             |  Ԥ��
-----------+-------------+--------------------------------------------
b13	       |   ��У��    |   0��������(MODBUS)��1������(RS)
-----------+-------------+--------------------------------------------
b14	       |   Э��      |   0��ר��Э��(MODBUS)��1����Э��(RS)
-----------+-------------+--------------------------------------------
b15	       | ��MODBUS    |   0���ӻ�  1������			 
----------------------------------------------------------------------

������D8120 = 0X4096           ͨѶ��������19200
*********************************************************************************/

#include "stm32f10x.h"
#include "stm32f10x_flash.h"
#include <stdio.h>
#include "PLC_Dialogue.h"
#include "PLC_IO.h"
#include "plc_conf.h"
#include "bsp_timer.h"

const u8   plc_programCodeBuf[34000] __at (PLC_RAM_ADDR)={						
//FLASH��ʼ��ַΪPLC��Ϣ**************************��ǰ��0X02��ʾPLCΪ16K�ĳ���,��������Ͳ�������****************
0x10,0x00,0xD8,0xBA,0x00,0x00,0x00,0x00,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,
0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,
0x20,0x20,0xF4,0x09,0xFF,0x0B,0xF4,0x01,0xE7,0x03,0x64,0x0E,0xC7,0x0E,0xDC,0x0E,0xFF,0x0E,0x90,0x01,0xFE,0x03,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x83,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0X0F,0X00,//����ָ��
0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,};
	

//D8000 = 200;		   		  ɨ��ʱ��
//D8001 = 0X5EF6;	        �ͺŰ汾 
//D8101 = 0X5EF6;	        �ͺŰ汾 
//D8002 = 10;			   		  �ڴ����� 16K
//D8102 = 10;			   		  �ڴ����� 16000�
//D8003 = 0x0010; 	   	  �洢����: PLC���ô洢
//D8006                   CPU��ص�ѹ
//D8010 = 10;			   	    ɨ�赱ǰֵ
//D8011 = 20;			   	    ɨ����Сʱ��(0.1MS)
//D8012 = 140;			   	  ɨ���ʱ��(0.1MS) 

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
static u8 Flag_Uart_Send=1;             //���ͱ�־λ
u16 rx_count,tx_count;                  //���ݼ���       



char tx_data[PROGRAM_BUF_SIZE];         //���ͻ���
char rx_data[PROGRAM_BUF_SIZE];         //���ջ���

u16 prog_address,data_address;          //�������ݲ�����ʼ��ַ����


u16 plc_16BitBuf[PLC_16BIT_BUF_SIZE] __at (RAM_ADDR);   //PLC_RAM��������

// 0x200066B0~0x2000D200 // 26KB
// ���˼ǣ�20170424�޸�
u8  step_status[1000];  
u8  step_address[2000]; //д��״̬Ϊ0���������½���ʹ�ù�2K�ֽ�1600bit

u8  data_size,block_contol[2];													
extern u8  edit_prog;
extern void RTC_Set(u16 syear,u8 smon,u8 sday,u8 hour,u8 min,u8 sec);	
												
u8 Send_out;
u8 Write_Pro_flag = 0;


void data_init(void)//D8000~D8126��ʼ��
{                                                 
	u16 temp;                                       
	u16 temp_address;                                
	
	prog_address=0x52;                                
  for(temp=0;temp<126;temp++)                      
  {                                                                   
       temp_address=0x0C00;                        
	   plc_16BitBuf[temp_address+temp]=special_d[temp]; //����ϵͳFLASH���ݵ��û�����
  }                                                //
	// D8000
	plc_16BitBuf[0x2000] =plc_programCodeBuf[prog_address] *256; // ȡ�ֽڸ�λ                      
	plc_16BitBuf[0x2000] |=plc_programCodeBuf[prog_address+1]; // ȡ�ֽڵ�λ
	
	PLC_RW_RAM_8BIT(0x01E0) =0x09; // ��M8000 M8003��ON
	
	block_contol[0]=200; // ��ֹд����ʱ����ʧ
	block_contol[1]=200;                            
	
#if DEBUG
	  #warning "MC��MCRָ��--data_init";
//	  MC.MC_SFR = 0;
//	  MC.MC_Flg = 0;
//	  MC.PLC_MC_BIT = 0;
#endif
	
}                                                  

void write_block(u16 number)                                                  // д��FLASH
{ 
	u16 temp,wait_write,appoint_address;
	if(number<17)			                                                          // д������Ŀ����10��
	{
		FLASH_Unlock();		                                                        // flash�رձ���
		FLASH_ErasePage(PLC_RAM_ADDR+number*0x800);                               // ����һ������ռ��2K
		for(temp=0;temp<1024;temp++)	                                            // ����Ϊ16bit,ֻ��Ҫ1024�γ���������
		{
			appoint_address =PLC_RAM_ADDR + number*0x800 + temp*2;                  // ��ʼ��ַ���Ͽ��ַ�ټ��Ͽ��С��ַ,����Ŀ��λ�� 
			wait_write =progWriteBuf[temp*2] + progWriteBuf[temp*2+1]*0X100; // д��16bit��flash
			FLASH_ProgramHalfWord(appoint_address,wait_write);                      // �ȴ�����д�����
		}
		FLASH_Lock();	                                                            // ��������д�뿪��flash����
	}
}

void backup_block(u16 number)		                                              // ����鱸��,Ŀ����д����֮ǰ��ǰ����򱸷�
{
	u16 temp,appoint_address;
	if(number<17)
	{
		for(temp=0;temp<2048;temp++)
		{
			appoint_address =number*0x800 + temp;                                    // ��ʼ��ַ���Ͽ��ַ�ټ��Ͽ��С��ַ 
			progWriteBuf[temp] =plc_programCodeBuf[appoint_address];	           // �����򱸷ݳ���
		}
	}
}

//=======================================================================================================
// ��������: ErasurePLC
// ����������PLC����FLASH�ռ�
// �䡡��:  mode ģʽ       
// �䡡��:  void     
// ȫ�ֱ���:  
// ����ģ��: 
// ������:  ���˼�
// �ա���:  2014��5��18��
// ��  ע:  
//-------------------------------------------------------------------------------------------------------
// �޸���:
// �ա���:
// ��  ע: 
//-------------------------------------------------------------------------------------------------------
//=======================================================================================================
void ErasurePLC(u8 mode)
{
	u16 temp=0,Erasure_plc_16BitBuf;
	/*******************************************PLC�洢�ڴ�����	************************************************/
	if(mode==1)
	{
		backup_block(0);
		progWriteBuf[92]=0x0f;                                                         //��ֵ
		progWriteBuf[93]=0x00;                                                         //��ֵ
		for(temp=94;temp<2048;temp++)                                                       //��0x5E
		{ 
			progWriteBuf[temp]=0xffff;                                                    //
		}
		FLASH_Unlock();                                                                     //flash�رձ���                     
		FLASH_ErasePage(PLC_RAM_ADDR+0x800);                                            
		write_block(0);                                                                     //���ݵ�һ��flash
		for(temp=1;temp<10;temp++)                                                          //����10��
		FLASH_ErasePage(PLC_RAM_ADDR+temp*0x800);                                           //����flash
		FLASH_Lock();                                                                       //��������д�뿪��flash����
	}
/*******************************************PLC��������Ԫ��λ*********************************************/
   if(mode==2)
	 {
	   for(Erasure_plc_16BitBuf=0x4000;Erasure_plc_16BitBuf<0x7E7E;Erasure_plc_16BitBuf+=2)           // ���D0000-D7999
       plc_16BitBuf[Erasure_plc_16BitBuf]=0x00;
	 }
/*******************************************PLC����λԪ��**************************************************/
	 if(mode==3)
	 {
	      for(Erasure_plc_16BitBuf=0x0000;Erasure_plc_16BitBuf<0x00BE;Erasure_plc_16BitBuf+=2)	    // ���M0000-M3071
        plc_16BitBuf[Erasure_plc_16BitBuf]=0x00;
	 }
   tx_data[1]=0x06,tx_count=1,Send_out=5;                                                 // �����ϱ�����λ��
}
				
/*******************************************************************************
�������ܣ�����У��� 
��ע�� 20171102�����˼��Ż�
*******************************************************************************/
u8 CheckSum(char * pBuf)//�����������У��
{ 
	u16 i; 
	u8 sum;
	sum=0;		// ����ͼ�����
	pBuf +=3;	// ����ʹӵ���λ��ʼ
	for(i=3;i<(rx_count-1);i++) // �����
	{ 
		sum += *pBuf; // ��ʼ���
		pBuf++;				// ָ���һ
	}
	return sum;			// ��������	
}

/*******************************************************************************
�������ƣ�void switch_read_data(void)  
�������ܣ�ת��ASCII��ΪHEX�룬ռ�����ݷ��ͼĴ���         
���ڲ�������
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

void setup_HL(void)	                     //�ߵ�λ������ת�� ��С��ת��
{                                                                     
	u8 temp;                                                            
	temp=tx_data[3];				               //��ַ��λ����16λ������     
	prog_address=temp*0x100+tx_data[2];    //������������ʼ��ַ       
}	                                                                     

void setup_LH(void)	                     //������ַת��
{ 
	u8 temp;
	temp=tx_data[3];				              //��ַ��λ����16λ������
	data_address=temp*0x100+tx_data[4];  //�������ݲ�����ʼ��ַ
}

typedef union                           
{
	int data;
	char data1[2];
} usart_data;

// ���˼ǣ�20161122ע�ͣ�//��ȡPLC�ͺţ�ָ�30��
void read_plc_tyte(u8 addr)                                  
{
	u16 temp;
	u8 temp_sum; 
	usart_data plc_type;
	
	plc_type.data=special_d[addr];                             //PLC�ͺ�
	tx_data[1]=0x02;                                           //���Ŀ�ʼ	02
	temp_sum=0;
	for(temp=0;temp<data_size;temp++)
	{ 
		tx_data[temp*2+2]=Ascll[plc_type.data1[temp]/0x10]; //ȡ�ֽڸ�λ
		tx_data[temp*2+3]=Ascll[plc_type.data1[temp]%0x10]; //ȡ�ֽڵ�λ
		temp_sum+=tx_data[temp*2+2]+tx_data[temp*2+3];
	}
	tx_data[temp*2+2]=0x03;                                     //���Ľ���	03
	temp_sum+=0x03;
	tx_data[temp*2+3]=Ascll[temp_sum/0x10];
	tx_data[temp*2+4]=Ascll[temp_sum%0x10]; 
	tx_count=temp*2+4;
}


/*******************************************************************************
�������ƣPPLC_Comm_Byte
�������ܣ�ͨ���ֽڵ�ַ�ض��弰ִ��
��ڲ�����Cmd����          
���ڲ�����ӳ���ʵ�ʵ�ַ(16BIT��ַ)
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
	else if(comm_add>=0x0200&&comm_add<=0x021F) // T00-T255  OVER����
			return comm_add-0x0140;
	else if(comm_add>=0x0500&&comm_add<=0x051F) // T00-T255  Enable ��Ȧ
			return comm_add-0x0240; 
	else if(comm_add>=0x01E0&&comm_add<=0x01FF) // C00-C255  OVER����
			return comm_add-0x0020;
	else if(comm_add<=0x00BF) // M0000-M1535 
			return comm_add+0x0100;
	else if(comm_add>=0x00C0&&comm_add<=0x017F) // M1536-M3071
			return comm_add+0x0440;
	else if(comm_add>=0x01C0&&comm_add<=0x01DF) // M8000-M8255 
			return comm_add+0x0020;
	else // ��Ч��ַ
			return comm_add;                                              
}

/*******************************************************************************
�������ƣ�PLC_Com_BIT
�������ܣ�ͨ��λ��ַ�ض��弰ִ��
��ڲ�����Cmd����          
���ڲ�����ӳ���ʵ�ʵ�ַ(BIT��ַ)
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
 else if((addr>=0x9800)&&(addr<=0x99FF)) // T00-T255 Enable ��Ȧ *
	 return (addr-0X7200); 
 else if((addr>=0x9700)&&(addr<=0x97FF)) // C00-C255 Enable ��Ȧ *
	 return (addr-0X7200); 
 else if((addr>=0x6300)&&(addr<=0x64FF)) // T00-T255 OVER   ���� *
	 return (addr-0X5D00); 
 else if((addr>=0x6200)&&(addr<=0x62FF)) // C00-C255	OVER   ����	*
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
		return (addr-0X1200); //T00-T255 Enable ��Ȧ
	else if((addr>=0x1000)&&(addr<=0x10FF)) 
		return (addr-0X0A00); //T00-T255 OVER   ����
	else if((addr>=0x0E00)&&(addr<=0x0EFF)) 
		return (addr+0X0100); //M8000-M8255
	else if((addr>=0x0F00)&&(addr<=0x0FFF)) 
		return (addr-0X0100); //C00-C255 OVER����
	else  return addr;
}


//=======================================================================================================
// ��������:  void READ_data(void)
// ���������� ������  X,Y,M,S,T,C,D   
// �䡡��:   void  
// �䡡��:   void   
// ȫ�ֱ���:  
// ����ģ��: 
// ������:  ���˼�
// �ա���:  2014��6��10��
// ��  ע:  
//-------------------------------------------------------------------------------------------------------
// �޸���:
// �ա���:
// ��  ע:  ���˼ǣ�20161122ע�ͣ�PLC_Comm_Byte��ȡFX0N��̿�ͨѶ����ʱ���ڲ���Ԫ����ַ��S X Y T M C M8�� ���� T C16 C32 D8 D D1000 ��
//=======================================================================================================
void read_other_data(void)	                                       //ָ�30��
{
	u16 i;
	u8 temp_sum;
	tx_data[1]=0x02;                                                 //���Ŀ�ʼ
	temp_sum=0;
	for(i=0;i<data_size;i++)
	{ 
		tx_data[i*2 + 2]=Ascll[PLC_RW_RAM_8BIT(PLC_Comm_Byte(i + prog_address))/0x10];//ȡ�ֽڸ�λ
		tx_data[i*2 + 3]=Ascll[PLC_RW_RAM_8BIT(PLC_Comm_Byte(i + prog_address))%0x10];//ȡ�ֽڵ�λ
		temp_sum+=tx_data[i*2+2]+tx_data[i*2+3];
	}
	tx_data[i*2 + 2]=0x03;                                          //���Ľ���	03
	temp_sum+=0x03;
	tx_data[i*2 + 3]=Ascll[temp_sum/0x10];
	tx_data[i*2 + 4]=Ascll[temp_sum%0x10]; 
	tx_count=i*2+4;
}

//=======================================================================================================
// ��������:  void READ_data(void)
// ���������� д����  X,Y,M,S,T,C,D   
// �䡡��:   void  
// �䡡��:   void   
// ȫ�ֱ���:  
// ����ģ��: 
// ������:  ���˼�
// �ա���:  2014��6��10��
// ��  ע:  
//-------------------------------------------------------------------------------------------------------
// �޸���:
// �ա���:
// ��  ע: 
//=======================================================================================================
void PC_WRITE_byte(void)                       //д��
{ 
	u16 temp;
	prog_address=tx_data[2]*0x100+tx_data[3]+4;//�������ݲ�����ʼ��ַ
	for(temp=0;temp<data_size;temp++)
	{
		PLC_RW_RAM_8BIT(PLC_Comm_Byte(temp+prog_address))=tx_data[5+temp];
	}
	tx_data[1]=0x06,tx_count=1;                //������λ��
}

//=======================================================================================================
// ��������:  void FORCE_ON_data(void)   
// ���������� FORCE ON  X,Y,M,S,T,C
// �䡡��:   void  
// �䡡��:   void   
// ȫ�ֱ���:  
// ����ģ��: 
// ������:  ���˼�
// �ա���:  2014��6��10��
// ��  ע:  
//-------------------------------------------------------------------------------------------------------
// �޸���:
// �ա���:
// ��  ע: 
//=======================================================================================================
void PC_FORCE_ON(void)	                                //ǿ��  38 ON
{ 
	PLC_BIT_ON(PLC_Com_BIT(tx_data[2]*0x100+tx_data[3]));//�������ݲ�����ʼ��ַ
	tx_data[1]=0x06,tx_count=1;  
}

//======================================================================================================
// ��������:  void FORCE_ON_data(void)   
// ���������� FORCE OFF  X,Y,M,S,T,C
// �䡡��:   void  
// �䡡��:   void   
// ȫ�ֱ���:  
// ����ģ��: 
// ������:  ���˼�
// �ա���:  2014��6��10��
// ��  ע:  
//------------------------------------------------------------------------------------------------------
// �޸���:
// �ա���:
// ��  ע: 
//=======================================================================================================
void PC_FORCE_OFF(void)	                      //ǿ��  37  OFF 
{ 
	PLC_BIT_OFF(PLC_Com_BIT(tx_data[2]*0x100+tx_data[3]));//�������ݲ�����ʼ��ַ
	tx_data[1]=0x06,tx_count=1;   
}

void PC_READ_byte(void)	                        //����
{
	prog_address=tx_data[2]*0x100+tx_data[3];     //�������ݲ�����ʼ��ַ
	switch(prog_address)
	{ 
		case 0x0ECA: read_plc_tyte(101);  break;  //��PLC TYPE      ���˼ǣ�20161122ע�ͣ�D8101
		case 0x0E02: read_plc_tyte(1);    break;  //��PLC TYPE      ���˼ǣ�20161122ע�ͣ�D8001
		default: read_other_data();       break;
	}
}


void EPC_FORCE_ON(void)	        //ʹ����չ����"E"ǿ��ON
{  
   switch(prog_address)
   {
			case 0x6023: PLC_RW_RAM_8BIT(0X01E0)=0x09;  break;  // Զ�̲��������Ƿ���Խ���
	    case 0x6024: // Զ�̲�����Ҫ����
			{
				PLC_RW_RAM_8BIT(0X01E0)=0x09;  
				Write_Pro_flag = 0;  
			  break;  
			}
	    case 0x6025: PLC_RW_RAM_8BIT(0X01E0)=0x0A;  break;  // Զ�̲�����Ҫֹͣ
      default:     PLC_BIT_ON(PLC_Com_BIT(prog_address));break;  // ������������
	 }
   tx_data[1]=0x06,tx_count=1;
}

void EPC_FORCE_OFF(void)	  //ʹ����չ����"E"ǿ��OFF 
{ 
  PLC_BIT_OFF(PLC_Com_BIT(prog_address));
  tx_data[1]=0x06,tx_count=1;
}  

void PC_READ_Parameter(void)                                  //������	  E00
{
  u16 temp,temp_bit,temp_addr,mov_bit,temp1;
  u8 temp_sum;
  u8 send,monitor,monitor1,monitor2; 
  tx_data[1]=0x02;                                             //���Ŀ�ʼ
  temp_sum=0;
  prog_address=PLC_Comm_Byte(tx_data[3]*0x100+tx_data[4]);       //�������ݲ�����ʼ��ַ
  if((prog_address==0x1790)||(prog_address==0x17D0))             //��������������0X1790 �� 0X17D0��ַ
  {
		  if(prog_address==0x1790)
      {
   	     monitor1=PLC_RAM8(0x20002400);	                         //������Ҫ���������
	       for(temp1=monitor=0;monitor<monitor1;monitor++)         //�����������
	       { 
					  temp_bit=PLC_Comm_Byte(plc_16BitBuf[0X0A02+monitor])/2;
					  if(0x7FE0==temp_bit)
						{ 
							monitor++;
						  temp_bit=plc_16BitBuf[0X0A02+monitor];
							plc_16BitBuf[0x0BC8+temp1]=plc_16BitBuf[temp_bit];temp1++;
						}
						else
				    plc_16BitBuf[0x0BC8+temp1]=plc_16BitBuf[temp_bit];temp1++;//����Ҫ�����ݴ�������		
						
					  if((temp_bit>=0x600)&&(temp_bit<=0x66E))             //��Ҫ��C200-C255�Ժ�ĵ�ַ��32λ�� 
					  {   
						  plc_16BitBuf[0x1790/2+temp1]=plc_16BitBuf[temp_bit+1];   //����Ҫ�����ݴ�������
						  temp1++;
					  }							
		     }
	       monitor2=PLC_RAM8(0x20002402);                          //������Ҫ���λ����
				 temp1=temp1+16;                                         //
	       for(monitor1=0;monitor1<monitor2;monitor1++)            //�����λ����
	       {  
						temp_addr=PLC_Com_BIT(plc_16BitBuf[0x0A02+monitor+monitor1]);
		        temp_bit=plc_16BitBuf[temp_addr/0x10];
		        mov_bit = temp_addr%0x10;
		        if((temp_bit&(1<<mov_bit))==(1<<mov_bit))
		        plc_16BitBuf[0x0BC8+temp1+monitor1/0x10]|=1<<(monitor1%0x10);//���ڻ�������	
			      else plc_16BitBuf[0x0BC8+temp1+monitor1/0x10]&=~(1<<(monitor1%0x10));	      					
		     }
	     }
   }
	 for(temp=0;temp<data_size;temp++)	    //��RAM
   { 
			send=PLC_RW_RAM_8BIT(prog_address+temp);
	    tx_data[temp*2+2]=Ascll[send/0x10]; //ȡ�ֽڸ�λ
      tx_data[temp*2+3]=Ascll[send%0x10]; //ȡ�ֽڵ�λ
	    temp_sum+=tx_data[temp*2+2]+tx_data[temp*2+3];
   }
   tx_data[temp*2+2]=0x03;    //
   temp_sum+=0x03;
   tx_data[temp*2+3]=Ascll[temp_sum/0x10];
   tx_data[temp*2+4]=Ascll[temp_sum%0x10]; 
   tx_count=temp*2+4;
}

void PC_WRITE_Parameter(void)                //д����	  E10
{  
	u16 temp;
	prog_address=tx_data[3]*0x100+tx_data[4];  //�������ݲ�����ʼ��ַ
	for(temp=0;temp<data_size;temp++)	         //write  RAM
	{
	    PLC_RW_RAM_8BIT(PLC_Comm_Byte(prog_address+temp))=tx_data[6+temp];
	}
	tx_data[1]=0x06,tx_count=1; //������λ��
}
 

void PC_READ_PORG(void)	          //������E01
{
	u16 temp;
	u8 temp_sum; 
	
	if(0x805C < data_address)
	{
		tx_data[1]=0x15;
		tx_count=1; 
		return;
	}	
	
	tx_data[1]=0x02;                 //���Ŀ�ʼ
	temp_sum=0;
	data_address-=0x8000;	           //��FLASH ��ַ��0x8000����ʵ��λ��
	for(temp=0;temp<data_size;temp++)
	{ 
		tx_data[temp*2+2]=Ascll[plc_programCodeBuf[data_address+temp]/0x10]; //ȡ�ֽڸ�λ
		tx_data[temp*2+3]=Ascll[plc_programCodeBuf[data_address+temp]%0x10]; //ȡ�ֽڵ�λ
		temp_sum+=tx_data[temp*2+2]+tx_data[temp*2+3];
	}
	tx_data[temp*2+2]=0x03;  
	temp_sum+=0x03;
	tx_data[temp*2+3]=Ascll[temp_sum/0x10];
	tx_data[temp*2+4]=Ascll[temp_sum%0x10]; 
	tx_count=temp*2+4;
}

//  ����ͼ����
void PC_WRITE_PORG(void)	                           //д���� E11 
{  
	 u16 i;
   prog_address=tx_data[3]*0x100+tx_data[4];         //�������ݲ�����ʼ��ַ
	 
	 PLC_RW_RAM_8BIT(0X01E0)=0x0A; 
	 Write_Pro_flag = 1;			   //��ֹ�����ع����г������� ADD ���˼�
	
	 edit_prog=0;                                      //�ѳ���༭���,��Ϊд����ʱ���ܴ���P��ַ�����仯��PLCӦ����P��ַ
	 prog_address-=0x8000;                             //��ַ��0X8000����ʵ��λ��
	 for(i=0;i<data_size;i++)
	 {
		    block_contol[0]=(prog_address+i)/0x800;	 //ÿһ��ռ�õĵ�ַ 0X800��2K �ֽ�
	      if(block_contol[0]==block_contol[1])			   //�Ƿ���Ҫ���飬
	      {
			    progWriteBuf[(prog_address+i)-block_contol[0]*0x800]=tx_data[6+i];  //������д�뻺����
	      }
		    else							                           //��Ҫ���鴦��
		    {
			    write_block(block_contol[1]);              //��ǰһ��д�뵽FLASH
		      backup_block(block_contol[0]);             //������Ҫд��FLASH��
		      block_contol[1]=block_contol[0];
		      progWriteBuf[(prog_address+i)-block_contol[0]*0x800]=tx_data[6+i];
		    }
   }
   tx_data[1]=0x06,tx_count=1;
}

static u16 find_data(u16 addr,u16 find_data)             //�������ݵ�ַ���������ҵ������ݵ�ַ
{
	 u8 find_ok,data_H,data_L;
   find_ok=5;
   data_H=find_data/0x100;
   data_L=find_data%0x100;
	 addr-=0x8000;
   do{
	      if((plc_programCodeBuf[addr]==data_L)&&(plc_programCodeBuf[addr+1]==data_H))
	      find_ok=0;		                                   //�ҵ���Ҫ��ָ��
	      else
	      addr+=2;
		    if(addr>(0xFD5C-0x8000))
		    find_ok=1;                                       //����Ч�ķ�Χ��û���ҵ�ENDָ��
	   }while(find_ok>3);
	 
	return addr+0X8000;
}
 
static void find_data_address(void)       //������λ����Ҫ��ָ���ַ
{ 
	   u8 temp_sum,data_H,data_L;  
		 data_L=tx_data[5];	           //��Ҫ�������ݵ����ݵ�λ	
     data_H=tx_data[6];            //��Ҫ�������ݵ����ݸ�λ
	   data_address=find_data(data_address,data_H*0X100+data_L);	//��FLASH
	   tx_data[1]=0x02;              //���Ŀ�ʼ
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
  end_addr=find_data(addr+0x8000,0x000f)+mov_addr-0x8000; //�ҳ�ENDָ������λ�ã��ټ����ƶ����������END����λ��
  start_addr=end_addr;
  addr_mov=addr;   
  if(addr>0x5B)            //��ǰ�ƶ��Ƿ����0��
	{
		  addr_mov-=0X5C;	     //��ȥ������ַ
	    end_addr-=0x5C;
	    addr_mov/=2;		     //һ��ռ�������ֽڡ���2Ϊָ����ʼ�ֽ�
	    end_addr/=2;
		  addr_mov/=8;			   //ָ����ÿһ����׳̬��ַ
	    end_addr/=8;
	    offset/=2;
	    mov_byte=offset/8;
	    offset%=8;
 	    while(!(end_addr==addr_mov))	                               //END address and curennt addr  �Ƿ����	 ,�������Ҫѭ��
	    {
	     temp=step_address[end_addr]*0x100+step_address[end_addr-1]; //����ǰ16λ��׳ֵ̬��Ϊ������Ҫ�ƶ�
		   temp<<=offset;                                              //����ƶ�ƫ�Ƶĵ�ַ����
		   step_address[end_addr+mov_byte]=temp/0x100;		             //��ֵ��Ŀ���ַ
		   end_addr--;							                                   //��ַ��ǰ��8��
		  }
      temp=step_address[end_addr]*0x100+step_address[end_addr-1];	 //����ǰ16λ��׳ֵ̬��Ϊ������Ҫ�ƶ�
      temp<<=offset;                                               //����ƶ�ƫ�Ƶĵ�ַ����
      step_address[end_addr+mov_byte]=temp/0x100;		               //��ֵ��Ŀ���ַ	    
	}
	end_addr=start_addr;   
	temp=start_addr;
do{
     if((end_addr/0x800)==(addr/0x800))                              //Ŀ���ƶ�λ����ʵ���ƶ�λ�õ���һ������
     start_addr=addr%0x800;			                                     //����ѵ���һ������ô��ʼ��ַΪĿ��λ�ÿ�ģ
	   else
     start_addr=0;					                                         //û�����0��ʼ�ƶ�
     if((temp/0x800)==(end_addr/0x800))                              //END���ڵĵ�ַ���������ƶ��ĵ�ַ����һ�����������һ���飬����Ҫ���н����ȶȴ���
     temp1=end_addr%0x800+1; 
	   else
	   temp1=2048;					                                           //���û����һ����ֱ�ӵ�һ����Ľ���λ��2048
	   backup_block(end_addr/0x800);
	   for(temp2=start_addr;temp2<temp1+1;temp2++)
	   {
		   backup_addr=(end_addr/0x800)*0x800+temp2-mov_addr;            //��ַ��ǰƫ����Ҫ�ƶ�����
	     progWriteBuf[temp2]=plc_programCodeBuf[backup_addr];			  	       //���ݵ�ַ
	   }
	   write_block(end_addr/0x800);                                    //���ƶ���������д��FLASH
	   end_addr-=(temp1-start_addr);                                   //��������ƶ����λ��  
	 }while(end_addr>addr+mov_addr);   
}


void online_write_data(void)                             //����д����
{  
	 static u16 temp,Size;
   signed short temp1,temp2;
   temp1=tx_data[6];                                      //��������ڶ��ٸ�����д
	 temp2=tx_data[8];                                      //��Ҫд����ٸ��ֽ�
	 temp2-=temp1;                                          //����һ���ַ 
	 if(temp2>0)                                            //���д��������ȴ���ʵ�ʵ����ݳ��ȣ�����ҪŲ��flash��ַ
	 {
	   mov_flash(data_address-0x8000,temp2); //��ҪŲ��flash�ĵ�ַ���Լ�Ų���ĳ���
	 }                
   edit_prog=0;                                           //�ѳ���༭���,��Ϊд����ʱ���ܴ���P��ַ�����仯��PLCӦ����P��ַ 
	 
	 block_contol[0]=100;
	 block_contol[1]=100;
     prog_address=(tx_data[3]*0x100+tx_data[4])-0x8000;     //�������ݲ�����ʼ��ַ
     data_size=tx_data[8];
	 for(temp=0;temp<data_size;temp++)
	 {
		     block_contol[0]=(prog_address+temp)/0x800;	      //ÿһ��ռ�õĵ�ַ 0X800��2K �ֽ�
	       if(block_contol[0]==block_contol[1])			        //�Ƿ���Ҫ���鴦��
	       {progWriteBuf[(prog_address+temp)-block_contol[0]*0x800]=tx_data[9+temp];}  //������д�뻺����
		     else							                                //��Ҫ���鴦��
		     {
			    write_block(block_contol[1]);                   //��ǰһ��д�뵽FLASH
		      backup_block(block_contol[0]);                  //������Ҫд��FLASH��
		      block_contol[1]=block_contol[0];
		      progWriteBuf[(prog_address+temp)-block_contol[0]*0x800]=tx_data[9+temp];
		     }
	  }
	  write_block(block_contol[0]);                         //��ǰһ��д�뵽FLASH  �Ѵ����·�����д��plc_programCodeBuf 
    if(temp2<0)	                                          //ɾ����������
		{    
			   temp2=0-temp2;                                   //����ɾ����������
			   Size=find_data(0x8000,0x000f)-prog_address;      //�� END��ַ-��ʼ������ַ=��������
		   	 for(;temp<Size;temp++)                           //temp ������ д��״̬
	       {
		        block_contol[0]=(prog_address+temp)/0x800;	  //ÿһ��ռ�õĵ�ַ 0X800��2K �ֽ�
	          if(block_contol[0]==block_contol[1])			    //�Ƿ���Ҫ���鴦��
	          {
			       progWriteBuf[(prog_address+temp)-block_contol[0]*0x800]=plc_programCodeBuf[prog_address+temp+temp2];  //������д�뻺����
	          }
		        else							                            //��Ҫ���鴦��
		        {
			        write_block(block_contol[1]);               //��ǰһ��д�뵽FLASH
		          backup_block(block_contol[0]);              //������Ҫд��FLASH��
		          block_contol[1]=block_contol[0];            //
		          progWriteBuf[(prog_address+temp)-block_contol[0]*0x800]=plc_programCodeBuf[prog_address+temp+temp2];
		        }
	       }   
				 write_block(block_contol[0]);                    //��ǰһ��д�뵽FLASH   
		}
    tx_data[1]=0x06,tx_count=1;
}

void all_flash_unlock(void)                               //FLASHȫ������
{
  block_contol[1]=200;
  block_contol[0]=200;
  tx_data[1]=0x06,tx_count=1;
}

void all_flash_lock(void) 	                                  //FLASHȫ������
{
  write_block(block_contol[1]);                               //����֮ǰ����Ҫд������д��FLASH
  block_contol[1]=200;
  block_contol[0]=200;
  FLASH_Lock();
  tx_data[1]=0x06,tx_count=1;
}

void PC_OPTION_PROG(void)                                      //��չ���� "E" ����
{ 	
	 u16 temp;
	 if((rx_count==11)&&((rx_data[4]==0x37)||(rx_data[4]==0x38)))//�Ƿ�Ϊǿ�ƹ��� 
	 {	
         prog_address=hex[rx_data[5]]*0x10+hex[rx_data[6]]+hex[rx_data[7]]*0x1000+hex[rx_data[8]]*0x100;
	     if(rx_data[4]==0x37) 
		   EPC_FORCE_ON();  
		   else
		   EPC_FORCE_OFF();
		}
		else
	  {
			 setup_LH();			                                      //���������ַ����
	     temp=tx_data[2];
	     switch(temp) 
		   { 
				 case 0x00: PC_READ_Parameter();  break;               //������ E00
			case 0x10: //д���� E10
			{
				// ���˼ǣ�20180524������gxwork2ֱ������ʱ�ӣ�Ӧ����Ҫ���⴦����
				// E10 0E1A[d8013] 0E24[d8018] 0010[��] 0011[ʱ] 0018[��] 0005[��] 00 E207[��] 04[����] 00
				
				PC_WRITE_Parameter(); 				
			  break;               
			}
			case 0x01: // ������ E01
			{
				PC_READ_PORG();       
			  break; 
      }				
			   case 0x11: PC_WRITE_PORG();      break;               //д���� E11 
			   case 0x77: all_flash_unlock();   break;               //ʹ��Eָ�����д����д����77
			   case 0x87: all_flash_lock();     break;               //ʹ��Eָ�����д�����������87
			   case 0x41: find_data_address();  break;               //����ENDָ���ַ	
				 case 0x61: all_flash_unlock();   break;               //PLC�洢�ڴ����� ӦΪ�·��˶�� ������FLASHȫ������
				 case 0x60: ErasurePLC(1);        break;               //PLC�洢�ڴ�����	
				 case 0x63: ErasurePLC(2);        break;               //PLC��������Ԫ��λ	
				 case 0x62: ErasurePLC(3);        break;               //PLC����λԪ��	
			   default: tx_data[1]=0x15,tx_count=1; break;           //������֧�ֵ�����
			 } 
		}
}

void READ_plc_programCodeBuf()                              //��16λ���ݰ���ֻ��
{
  u8 temp_sum; 
  tx_data[1]=0x02;                               //���Ŀ�ʼ
  temp_sum=0;
	tx_data[2]=Ascll[plc_programCodeBuf[0]/0x10];             //ȡ�ֽڸ�λ
  tx_data[3]=Ascll[plc_programCodeBuf[0]%0x10];             //ȡ�ֽڵ�λ
	tx_data[4]=Ascll[plc_programCodeBuf[1]/0x10];             //ȡ�ֽڸ�λ
  tx_data[5]=Ascll[plc_programCodeBuf[1]%0x10];             //ȡ�ֽڵ�λ
	temp_sum+=tx_data[2]+tx_data[3]+tx_data[4]+tx_data[5];
  tx_data[6]=0x03;                               // ����
  temp_sum+=0x03;
  tx_data[7]=Ascll[temp_sum/0x10];
  tx_data[8]=Ascll[temp_sum%0x10]; 
  tx_count=8;
}

void PLC_1BIT_State()                                           //��ȡ PLC����״̬  ���� 
{
	u8 temp_sum;
	tx_data[1]=0x02;
	temp_sum=0;
	if(PLC_BIT_TEST(PLC_Com_BIT(tx_data[5]*0X100+tx_data[4])))    //��ȡλ�Ĵ�����ַ
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

void PLC_Download(void)  //�鿴�Ĵ��� ״̬
{
	 u16 temp;
	 temp=tx_data[3];                              //�������ݲ�����ʼ��ַ
	 switch(temp) 
	 { 
		  case 0x11: READ_plc_programCodeBuf();          break;  //������	
		  case 0x10: PLC_1BIT_State();        break;  //��ȡPLC����״̬
		  default: tx_data[1]=0x15,tx_count=1;break;  //������֧�ֵ�����
	 } 
}

void PLC_Upload(void)           //�鿴��������ͼ������ַ
{
	u16 *PLC_Addr,temp;
	u8 temp_sum,data_H,data_L;
	temp=0X805A;                  //
  PLC_Addr=((u16*)(0x800605A));
  do{PLC_Addr++;temp+=2;}
	while(!(*PLC_Addr==0x000f));  //���END ��0X000F	 
	tx_data[1]=0x02;              //���Ŀ�ʼ
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

void PLC_E_Expand(void)                             //��չ���� "F" �ϴ���������س������
{ 	
	 u16 temp;
	 temp=rx_data[4];                                 //�������ݲ�����ʼ��ַ
	 setup_LH();			                                //���������ַ����
	 switch(temp) 
	 { 
		 case 0X35: PLC_Download();           break;    //���� ����ʱ����
		 case 0X38: PLC_Upload();             break;    //����ENDָ���ַ
		 case 0x43: online_write_data();      break;    //����д����	
		 default: tx_data[1]=0x06,tx_count=1; break;    //������֧�ֵ�����
	 } 
}

void find_end(void)		   //���ҳ������Ƿ����ENDָ�
{
	if(rx_count==13)
			tx_data[1]=0x06,tx_count=1;  
	else
			tx_data[1]=0x06,tx_count=1; 
}

void Process_switch(void)
{            
	   u8 temp;
     switch_read_data();                                          //�ѵ���λ��ʼ��ASCII��ת����HEX	����ַΪ���ݷ�����
		 temp=rx_data[3];
		 switch(temp) 
		 { 
					case 0x30: data_size=tx_data[4],PC_READ_byte();  break; //���������������λ��tx_data[4]��    ������
			    case 0x31: data_size=tx_data[4];PC_WRITE_byte(); break; //���������������λ��tx_data[4]��    д����
			    case 0x34: find_end();                           break; //����ָ�����ҵ��������򷵻�6
			    case 0x37: setup_HL(),PC_FORCE_ON();             break; //PLC	���� Զ�� ��0x37��
			    case 0x38: setup_HL(),PC_FORCE_OFF();            break; //PLC ֹͣ Զ�� ��0x38��
			    case 0x42: all_flash_lock();                     break; //д������������
			    case 0x46: PLC_E_Expand();                       break; //ͨѶF����ָ�� 
			    case 0x45: data_size=tx_data[5],PC_OPTION_PROG();break; //ͨѶE����ָ�� 
			    default:	                                       break;
		}                                                                                                                        
	   if((tx_count==0)&&(rx_count==0))	                            //return error code for 0x15
	 tx_data[1]=0x15,tx_count=1;			 		
	rx_count=0;Send_out=1;                                     //������λ��	
}		

void usart(u32 DEFAULT_BAUD)
{
	  USART_InitTypeDef USART_InitStructure;                          //�������üĴ���	
	  USART_InitStructure.USART_BaudRate = DEFAULT_BAUD;              //���ò�����
	  USART_InitStructure.USART_WordLength = USART_WordLength_8b;     //8λ����λ
	  USART_InitStructure.USART_StopBits = USART_StopBits_1;          //һλֹͣλ
	  USART_InitStructure.USART_Parity = USART_Parity_Even;           //У��λ 
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1,&USART_InitStructure);                        //��ʼ������   
}  

//---------------------���ڹ�������---------------------
void USART1_Configuration(void) 
{
	DMA_InitTypeDef  DMA_InitStructure;                           //DMA��ʼ���ṹ��
	GPIO_InitTypeDef GPIO_InitStructure;                          //�����������üĴ���
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 , ENABLE);       //�򿪴��ڶ�Ӧ������ʱ��  
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);            //����DMAʱ��
	DMA_DeInit(DMA1_Channel4);                                    //DMA1ͨ��4����
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART1->DR);//�����ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)tx_data+1;        //�ڴ��ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;            //dma���䷽����
	DMA_InitStructure.DMA_BufferSize = PROGRAM_BUF_SIZE;                       //����DMA�ڴ���ʱ�������ĳ���
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//����DMA���������ģʽ��һ������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;       //����DMA���ڴ����ģʽ
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//���������ֳ�
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;//�ڴ������ֳ�
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                 //����DMA�Ĵ���ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;           //����DMA�����ȼ���
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                  //����DMA��2��memory�еı����������
	DMA_Init(DMA1_Channel4,&DMA_InitStructure);
	DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,ENABLE);
	usart(19200);                                //��ʼ������ 
	//TXE�����ж�,TC��������ж�,RXNE�����ж�,PE��ż�����ж�,�����Ƕ��   
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);  
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);//����DMA��ʽ����
	USART_Cmd(USART1, ENABLE);                  //��������    
	
	//*********************����1�Ĺܽų�ʼ�� ****************************************   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;                       //�ܽ�9  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;               //ѡ��GPIO��Ӧ�ٶ�  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                 //�����������  
	GPIO_Init(GPIOA, &GPIO_InitStructure);                          //TX��ʼ��  
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;                      //�ܽ�10  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;           //��������  
	GPIO_Init(GPIOA, &GPIO_InitStructure);                          //RX��ʼ��                                                      
}

void DMA1_Channel4_IRQHandler(void)                 //����1DMA��ʽ�����ж�
{
	DMA1->IFCR |= DMA1_FLAG_TC4;                      //�����־λ
	DMA1_Channel4->CCR &= 0xFFFE;                     //�ر�DMA
	Flag_Uart_Send = 1;                               //�����ٴη���
}

void  TX_Process(void) // ���ʹ�������
{  
	if(Flag_Uart_Send)
	{
		Send_out=Flag_Uart_Send = 0;
		DMA1_Channel4->CNDTR = tx_count;// ���ô������ݳ���
		DMA1_Channel4->CCR |= 0x0001;   // ��DMA
	}
}


void RX_Process(void)                                //���մ�������
{   
	static u8 sum,f=1;
	rx_data[0]=0x7f&USART1->DR;

	if(rx_data[0]==0X05)		                            //��λ�����ͨѶ����
	{
		rx_count=0;
		
		tx_data[1]=0x06;
		tx_count=1;
		TX_Process(); //������λ�����ҷ���0X06Ӧ��	
		
		 gCommLedFlashFlg =2; // ��˸
	} 
	else if(rx_data[0]==0X02)	                          //���Ŀ�ʼ
	{
		rx_count=0x01;
	}
	else if(rx_count==0)		
	{
		if(f==1)
		{
			usart(19200);
			rx_count=0,tx_data[1]=0x06,tx_count=1,TX_Process();//������λ�����ҷ���0X06Ӧ��	 
			f=0;
		}
		else if(f==0)
		{
			usart(9600);
			rx_count=0,tx_data[1]=0x06,tx_count=1,TX_Process();//������λ�����ҷ���0X06Ӧ��	 	 
			f=1;
		}		
	}
	if(rx_count>0)		                              //ȷ�ϱ��Ŀ�ʼ
	{
		rx_count++; 
		rx_data[rx_count]=rx_data[0];
		if(rx_count > PROGRAM_BUF_SIZE)	             //��ȡ����������610
		{
			tx_count=0;
			rx_count=0;
		}
		if((rx_count>3)&&(rx_data[rx_count-2]==0x03))	 //�����Ƿ��ͽ���
		{ 
			sum = CheckSum(rx_data);            
			if((rx_data[rx_count-1] == Ascll[sum/0x10])&&(rx_data[rx_count] == Ascll[sum%0x10]))// �������ݺ�״̬ �����Ƿ�����
			{
				Process_switch(); 
			}
			else 
			{
				tx_data[1]=0x15;
				tx_count=1;
				TX_Process();     //������λ�����һش������쳣��ֵ0X15
			}
			
			gCommLedFlashFlg =2;  // ��˸
		}
	}	   
}

void USART1_IRQHandler(void)                         //�жϵ���
{ 
	if(USART_GetITStatus(USART1,USART_IT_RXNE)==SET)  //�����ж�
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
		RX_Process();
	}
	//���-������������Ҫ�ȶ�SR,�ٶ�DR�Ĵ����������������жϵ�����
	if(USART_GetFlagStatus(USART1,USART_FLAG_ORE)==SET)
	{
		USART_ClearFlag(USART1,USART_FLAG_ORE);         //��SR��ʵ���������־
		USART_ReceiveData(USART1);                      //��DR
	}
	if(USART_GetITStatus(USART1, USART_IT_TXE)==SET)
	{
		USART_ClearITPendingBit(USART1,USART_IT_TXE);
		USART_ITConfig(USART1,USART_IT_TXE,DISABLE);
	}
	if(USART_GetITStatus(USART1, USART_IT_TC)==SET)  //�����ж�
	{
	  USART_ClearITPendingBit(USART1,USART_IT_TC);
	}
}


