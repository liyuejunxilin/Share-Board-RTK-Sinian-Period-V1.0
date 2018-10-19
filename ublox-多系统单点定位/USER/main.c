#include "sys.h"
#include "delay.h" 
#include "led.h"  
#include "usart.h"
#include "usart3.h"
#include "rtklib.h"
/******************************************************************
  Module Name    :               
  File Name      : main.c	            
  Author   	     : LHC         
  Create Date  	 : 2018/10/18         
  Version   	   : Share-Board/RTK Sinian Period V1.0                
  Function       : u-blox M8T+STM32F7单点定位算法
  Description  	 : 无     
  Support     	 : QQ:459757196              
******************************************************************/

unsigned char Soluion_GGA[150];
unsigned char Soluion_RMC[150];
unsigned char Soluion_GSV[150];
unsigned char Soluion_GSA[150];
strsvr_t svr;

const prcopt_t default_opt={ /* defaults processing options */
    PMODE_SINGLE,0,1,SYS_GPS|SYS_CMP,   /* mode,soltype,nf,navsys */
    15.0*D2R,           								/* elmin*/
//		{{1,1},{20.0,20.0,0.0,0.0,0.0,0.0,0.0,0.0,20.0}},						/* SNR mask*/
		{{1,1},{40.0,35.0,30.0,25.0,20.0,20.0,20.0,20.0,20.0}},				/* SNR mask*/
    0,ARMODE_FIXHOLD,1,5,0,10,               /* sateph,modear,glomodear,maxout,minlock,minfix */
    IONOOPT_BRDC,TROPOPT_SAAS,0,0,           /* estion,esttrop,dynamics,tidecorr */
    1,0,0,0,0,                  /* niter,codesmooth,intpref,sbascorr,sbassatsel */
    0,0,                        /* rovpos,refpos */
    100.0,              					/* eratio[] */
    {100.0,0.003,0.003,0.0,1.0}, /* err[] */
    {30.0,0.03,0.3},            /* std[] */
    {1E-4,1E-3,1E-4,1E-1,1E-2}, /* prn[] */
    5E-12,                      /* sclkstab */
    {3.0,0.9999,0.20},          /* thresar */
    0.0,0.0,0.05,               /* elmaskar,almaskhold,thresslip */
    30.0,30.0,30.0,             /* maxtdif,maxinno,maxgdop */
    {0},{0},{10.780175707,106.660899381,31.5523}                /* baseline,ru,rb */
//    {"",""},                    /* anttype */
//    {{0}},{{0}},{0}             /* antdel,pcv,exsats */
};

int main(void)
{ 
	double ep[6];
	u16 t;
	u16 len;
	u8 led=0;
	gtime_t time;
	Stm32_Clock_Init(432,25,2,9);//设置时钟,216Mhz
  delay_init(216);		//延时初始化 
	uart_init(108,256000);	//串口初始化为115200
	usart3_init(54,115200);
	LED_Init();		  		//初始化与LED连接的硬件接口   
	rtkinit(&svr.rtk,&default_opt);//配置初始化
  init_raw(&svr.raw[0]);
	svr.stream[0].type=STR_SERIAL;
	svr.conv[0]->itype=STRFMT_UBX;
	while(1)
	{
		if(USART3_RX_STA&0x8000)
		{		
			if(led)
				LED0(0);
			else
				LED0(1);
				led=~led;
			len=USART3_RX_STA&0X7FFF;	//得到数据长度
			for(t=0;t<len;t++)
			input_raw(&svr.raw[0],svr.conv[0]->itype,USART3_RX_BUF[t]);
			 USART3_RX_STA=0;
			if(ublox_raw_flag==1)
			{
				
				time=gpst2utc(svr.raw[0].time);
				if (time.sec>=0.995) {time.time++; time.sec=0.0;}
				time2epoch(time,ep);
				
				
				rtkpos(&svr.rtk,svr.raw[0].obs.data,svr.raw[0].obs.n,&svr.raw[0].nav);
//				outsol(Soluion,&svr.rtk.sol,svr.rtk.rb);
//				printf("GPGGA,%s\r\n",Soluion);
				outnmea_gga(Soluion_GGA,&svr.rtk.sol);
				outnmea_rmc(Soluion_RMC,&svr.rtk.sol);
				outnmea_gsa(Soluion_GSA,&svr.rtk.sol,svr.rtk.ssat);
				outnmea_gsv(Soluion_GSV,&svr.rtk.sol,svr.rtk.ssat);
				printf("%s%s%s%s\r\n",Soluion_GGA,Soluion_RMC,Soluion_GSA,Soluion_GSV);
				printf("ERROR,%s\r\n",svr.rtk.errbuf);
				
				
//				printf("SATNUM=%02d,%02.0f%02.0f%05.2f,\r\n",svr.raw[0].obs.n,svr.raw[0].time.time,svr.raw[0].time.sec);
				printf("SATNUM=%02d,%02.0f%02.0f%05.2f,%02.0f%02.0f%02d*FF\r\n",svr.raw[0].obs.n,ep[3],ep[4],ep[5],ep[2],ep[1],(int)ep[0]%100);
//				for(t=0;t<svr.raw[0].obs.n;t++)
//				{
//					printf("GPSSYS=%d*EE\r\n",svr.raw[0].obs.data[t].rcv);
//					if(svr.raw[0].obs.data[t].sat<33)
//					printf("GPSSYS=G%02d,%06lf,%06lf,%06lf,%03d*EE\r\n",svr.raw[0].obs.data[t].sat,svr.raw[0].obs.data[t].L[0],svr.raw[0].obs.data[t].P[0],svr.raw[0].obs.data[t].D[0],svr.raw[0].obs.data[t].SNR[0]);
//					else if(svr.raw[0].obs.data[t].sat>=33&&svr.raw[0].obs.data[t].sat<57)
//					printf("GLOSYS=R%02d,%06lf,%06lf,%06lf,%03d*EE\r\n",svr.raw[0].obs.data[t].sat-32,svr.raw[0].obs.data[t].L[0],svr.raw[0].obs.data[t].P[0],svr.raw[0].obs.data[t].D[0],svr.raw[0].obs.data[t].SNR[0]);
//					else if(svr.raw[0].obs.data[t].sat>=57&&svr.raw[0].obs.data[t].sat<84)
//					printf("GALSYS=E%02d,%06lf,%06lf,%06lf,%03d*EE\r\n",svr.raw[0].obs.data[t].sat-56,svr.raw[0].obs.data[t].L[0],svr.raw[0].obs.data[t].P[0],svr.raw[0].obs.data[t].D[0],svr.raw[0].obs.data[t].SNR[0]);
//					else if(svr.raw[0].obs.data[t].sat>=84&&svr.raw[0].obs.data[t].sat<87)
//					printf("QZSSYS=J%02d,%06lf,%06lf,%06lf,%03d*EE\r\n",svr.raw[0].obs.data[t].sat-83,svr.raw[0].obs.data[t].L[0],svr.raw[0].obs.data[t].P[0],svr.raw[0].obs.data[t].D[0],svr.raw[0].obs.data[t].SNR[0]);
//					else if(svr.raw[0].obs.data[t].sat>=87&&svr.raw[0].obs.data[t].sat<122)
//					printf("BDSSYS=C%02d,%06lf,%06lf,%06lf,%03d*EE\r\n",svr.raw[0].obs.data[t].sat-86,svr.raw[0].obs.data[t].L[0],svr.raw[0].obs.data[t].P[0],svr.raw[0].obs.data[t].D[0],svr.raw[0].obs.data[t].SNR[0]);
//					else if(svr.raw[0].obs.data[t].sat>=122&&svr.raw[0].obs.data[t].sat<144)
//					printf("SBASYS=%02d,%06lf,%06lf,%06lf,%03d*EE\r\n",svr.raw[0].obs.data[t].sat-2,svr.raw[0].obs.data[t].L[0],svr.raw[0].obs.data[t].P[0],svr.raw[0].obs.data[t].D[0],svr.raw[0].obs.data[t].SNR[0]);
//						printf("week sec:%.3lf,%.8lf\r\n",svr.raw[0].nav.eph[svr.raw[0].obs.data[t].sat-1].A,svr.raw[0].nav.eph[svr.raw[0].obs.data[t].sat-1].e);
//				}
			 ublox_raw_flag=0;
			 ublox_eph_flag=0;
			}
		}
	}
}


