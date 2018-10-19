#include <stdio.h>
#include <math.h>
#include "rtklib.h"

#define SQRT(x)    ((x)<0.0?0.0:sqrt(x))
#define KNOT2M     0.514444444  /* m/knot */
static const int solq_nmea[]={  /* nmea quality flags to rtklib sol quality */
    /* nmea 0183 v.2.3 quality flags: */
    /*  0=invalid, 1=gps fix (sps), 2=dgps fix, 3=pps fix, 4=rtk, 5=float rtk */
    /*  6=estimated (dead reckoning), 7=manual input, 8=simulation */
    
    SOLQ_NONE ,SOLQ_SINGLE, SOLQ_DGPS, SOLQ_PPP , SOLQ_FIX,
    SOLQ_FLOAT,SOLQ_DR    , SOLQ_NONE, SOLQ_NONE, SOLQ_NONE
};
static double sqvar(double covar)
{
    return covar<0.0?-sqrt(-covar):sqrt(covar);
}
extern void outsol(char* res, const sol_t *sol, const double *rb)
{
	gtime_t time; 
	double ep[6],pos[3],Qecef[9],Qenu[9];
//#ifdef SOLF_ENU
//	double e[3],enu[3],rr[3],eQ[9];
//#endif	
	
	time=sol->time;
	time2epoch(time,ep);
	
#ifdef SOLF_LLH	
	ecef2pos(sol->rr,pos);
	Qecef[0]=sol->qr[0];//xx
	Qecef[4]=sol->qr[1];//yy
	Qecef[8]=sol->qr[2];//zz
	Qecef[1]=Qecef[3]=sol->qr[3];//xy
	Qecef[5]=Qecef[7]=sol->qr[4];//yz
	Qecef[2]=Qecef[6]=sol->qr[5];//zx
	covenu(pos,Qecef,Qenu);
	
	res+=sprintf(res,"%04.0f/%02.0f/%02.0f %02.0f:%02.0f:%06.3f %14.9f %14.9f %10.4f %3d %3d %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %6.2f %6.1f",
	ep[0],ep[1],ep[2],ep[3],ep[4],ep[5],//time yy/mm/dd hh:mm:ss.ssss
	pos[0]*R2D,pos[1]*R2D,pos[2],
	sol->stat,sol->ns,
	SQRT(Qenu[4]),SQRT(Qenu[0]),SQRT(Qenu[8]),
	sqvar(Qenu[1]),sqvar(Qenu[2]),sqvar(Qenu[5]),
	sol->age,sol->ratio);
#elif defined SOLF_ENU
	ecef2pos(sol->rb,pos);
	Qecef[0]=sol->qr[0];//xx
	Qecef[4]=sol->qr[1];//yy
	Qecef[8]=sol->qr[2];//zz
	Qecef[1]=Qecef[3]=sol->qr[3];//xy
	Qecef[5]=Qecef[7]=sol->qr[4];//yz
	Qecef[2]=Qecef[6]=sol->qr[5];//zx
	xyz2enu(pos,e);
	for (i=0;i<3;i++)
		rr[i]=sol->rr[i]-rb[i];
	matmul("NN",3,1,3,1.0,e,rr,0.0,enu);
	matmul("NN",3,3,3,1.0,e,Qecef,0.0,eQ);
  matmul("NT",3,3,3,1.0,eQ,e,0.0,Qenu);	
	
	res+=sprintf(res,"%04.0f/%02.0f/%02.0f %02.0f:%02.0f:%06.3f %14.4f %14.4f %14.4f %3d %3d %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %6.2f %6.1f",
	ep[0],ep[1],ep[2],ep[3],ep[4],ep[5],//time yy/mm/dd hh:mm:ss.ssss
	enu[0],enu[1],enu[2],
	sol->state,sol->ns,
	SQRT(Qenu[4]),SQRT(Qenu[0]),SQRT(Qenu[8]),
	sqvar(Qenu[1]),sqvar(Qenu[2]),sqvar(Qenu[5]),
	sol->age,sol->ratio);
#endif

//#ifdef ENCODER
//	res+=sprintf(res," %7d",sol->encoder);
//#endif

//#ifdef TIME_MEASURE
//	res+=sprintf(res," %4d",sol->processTime);
//#endif

	res[0]='\n';		
	
}
/* output solution in the form of nmea RMC sentence --------------------------*/
extern int outnmea_rmc(unsigned char *buff, const sol_t *sol)
{
    static double dirp=0.0;
    gtime_t time;
    double ep[6],pos[3],enuv[3],dms1[3],dms2[3],vel,dir,amag=0.0;
    char *p=(char *)buff,*q,sum,*emag="E";
    
    //trace(3,"outnmea_rmc:\n");
    
    if (sol->stat<=SOLQ_NONE) {
        p+=sprintf(p,"$GPRMC,,,,,,,,,,,,");
        for (q=(char *)buff+1,sum=0;*q;q++) sum^=*q;
        p+=sprintf(p,"*%02X%c%c",sum,0x0D,0x0A);
        return p-(char *)buff;
    }
    time=gpst2utc(sol->time);
    if (time.sec>=0.995) {time.time++; time.sec=0.0;}
    time2epoch(time,ep);
    ecef2pos(sol->rr,pos);
    ecef2enu(pos,sol->rr+3,enuv);
    vel=norm(enuv,3);
    if (vel>=1.0) {
        dir=atan2(enuv[0],enuv[1])*R2D;
        if (dir<0.0) dir+=360.0;
        dirp=dir;
    }
    else {
        dir=dirp;
    }
    deg2dms(fabs(pos[0])*R2D,dms1,7);
    deg2dms(fabs(pos[1])*R2D,dms2,7);
    p+=sprintf(p,"$GPRMC,%02.0f%02.0f%05.2f,A,%02.0f%010.7f,%s,%03.0f%010.7f,%s,%4.2f,%4.2f,%02.0f%02.0f%02d,%.1f,%s,%s",
               ep[3],ep[4],ep[5],dms1[0],dms1[1]+dms1[2]/60.0,pos[0]>=0?"N":"S",
               dms2[0],dms2[1]+dms2[2]/60.0,pos[1]>=0?"E":"W",vel/KNOT2M,dir,
               ep[2],ep[1],(int)ep[0]%100,amag,emag,
               sol->stat==SOLQ_DGPS||sol->stat==SOLQ_FLOAT||sol->stat==SOLQ_FIX?"D":"A");
    for (q=(char *)buff+1,sum=0;*q;q++) sum^=*q; /* check-sum */
    p+=sprintf(p,"*%02X%c%c",sum,0x0D,0x0A);
    return p-(char *)buff;
}
/* output solution in the form of nmea GGA sentence --------------------------*/
extern int outnmea_gga(unsigned char *buff, const sol_t *sol)
{
    gtime_t time;
    double h,ep[6],pos[3],dms1[3],dms2[3],dop=1.0;
    int solq;
    char *p=(char *)buff,*q,sum;
    
    //trace(3,"outnmea_gga:\n");
    
    if (sol->stat<=SOLQ_NONE) {
        p+=sprintf(p,"$GPGGA,,,,,,,,,,,,,,");
        for (q=(char *)buff+1,sum=0;*q;q++) sum^=*q;
        p+=sprintf(p,"*%02X%c%c",sum,0x0D,0x0A);
        return p-(char *)buff;
    }
    for (solq=0;solq<8;solq++) if (solq_nmea[solq]==sol->stat) break;
    if (solq>=8) solq=0;
    time=gpst2utc(sol->time);
    if (time.sec>=0.995) {time.time++; time.sec=0.0;}
    time2epoch(time,ep);
    ecef2pos(sol->rr,pos);
    h=geoidh(pos);
    deg2dms(fabs(pos[0])*R2D,dms1,7);
    deg2dms(fabs(pos[1])*R2D,dms2,7);
    p+=sprintf(p,"$GPGGA,%02.0f%02.0f%05.2f,%02.0f%010.7f,%s,%03.0f%010.7f,%s,%d,%02d,%.1f,%.3f,M,%.3f,M,%.1f,",
               ep[3],ep[4],ep[5],dms1[0],dms1[1]+dms1[2]/60.0,pos[0]>=0?"N":"S",
               dms2[0],dms2[1]+dms2[2]/60.0,pos[1]>=0?"E":"W",solq,
               sol->ns,dop,pos[2]-h,h,sol->age);
    for (q=(char *)buff+1,sum=0;*q;q++) sum^=*q; /* check-sum */
    p+=sprintf(p,"*%02X%c%c",sum,0x0D,0x0A);
    return p-(char *)buff;
}
/* output solution in the form of nmea GSA sentences -------------------------*/
extern int outnmea_gsa(unsigned char *buff, const sol_t *sol,
                       const ssat_t *ssat)
{
    double azel[MAXSAT*2],dop[4];
    int i,sat,sys,nsat,prn[MAXSAT];
    char *p=(char *)buff,*q,*s,sum;
    
//    trace(3,"outnmea_gsa:\n");
    
    if (sol->stat<=SOLQ_NONE) {
        p+=sprintf(p,"$GPGSA,A,1,,,,,,,,,,,,,,,");
        for (q=(char *)buff+1,sum=0;*q;q++) sum^=*q;
        p+=sprintf(p,"*%02X%c%c",sum,0x0D,0x0A);
        return p-(char *)buff;
    }
    /* GPGSA: gps/sbas */
    for (sat=1,nsat=0;sat<=MAXSAT&&nsat<12;sat++) {
        if (!ssat[sat-1].vs||ssat[sat-1].azel[1]<=0.0) continue;
        sys=satsys(sat,prn+nsat);
        if (sys!=SYS_GPS&&sys!=SYS_SBS) continue;
        if (sys==SYS_SBS) prn[nsat]+=33-MINPRNSBS;
        for (i=0;i<2;i++) azel[i+nsat*2]=ssat[sat-1].azel[i];
        nsat++;
    }
    if (nsat>0) {
        s=p;
        p+=sprintf(p,"$GPGSA,A,%d",sol->stat<=0?1:3);
        for (i=0;i<12;i++) {
            if (i<nsat) p+=sprintf(p,",%02d",prn[i]);
            else        p+=sprintf(p,",");
        }
        dops(nsat,azel,0.0,dop);
        p+=sprintf(p,",%3.1f,%3.1f,%3.1f,1",dop[1],dop[2],dop[3]);
        for (q=s+1,sum=0;*q;q++) sum^=*q; /* check-sum */
        p+=sprintf(p,"*%02X%c%c",sum,0x0D,0x0A);
    }
    /* GLGSA: glonass */
    for (sat=1,nsat=0;sat<=MAXSAT&&nsat<12;sat++) {
        if (!ssat[sat-1].vs||ssat[sat-1].azel[1]<=0.0) continue;
        if (satsys(sat,prn+nsat)!=SYS_GLO) continue;
        for (i=0;i<2;i++) azel[i+nsat*2]=ssat[sat-1].azel[i];
        nsat++;
    }
    if (nsat>0) {
        s=p;
        p+=sprintf(p,"$GLGSA,A,%d",sol->stat<=0?1:3);
        for (i=0;i<12;i++) {
            if (i<nsat) p+=sprintf(p,",%02d",prn[i]+64);
            else        p+=sprintf(p,",");
        }
        dops(nsat,azel,0.0,dop);
        p+=sprintf(p,",%3.1f,%3.1f,%3.1f,2",dop[1],dop[2],dop[3]);
        for (q=s+1,sum=0;*q;q++) sum^=*q; /* check-sum */
        p+=sprintf(p,"*%02X%c%c",sum,0x0D,0x0A);
    }
    /* GAGSA: galileo */
    for (sat=1,nsat=0;sat<=MAXSAT&&nsat<12;sat++) {
        if (!ssat[sat-1].vs||ssat[sat-1].azel[1]<=0.0) continue;
        if (satsys(sat,prn+nsat)!=SYS_GAL) continue;
        for (i=0;i<2;i++) azel[i+nsat*2]=ssat[sat-1].azel[i];
        nsat++;
    }
    if (nsat>0) {
        s=p;
        p+=sprintf(p,"$GAGSA,A,%d",sol->stat<=0?1:3);
        for (i=0;i<12;i++) {
            if (i<nsat) p+=sprintf(p,",%02d",prn[i]);
            else        p+=sprintf(p,",");
        }
        dops(nsat,azel,0.0,dop);
        p+=sprintf(p,",%3.1f,%3.1f,%3.1f,3",dop[1],dop[2],dop[3]);
        for (q=s+1,sum=0;*q;q++) sum^=*q; /* check-sum */
        p+=sprintf(p,"*%02X%c%c",sum,0x0D,0x0A);
    }
    return p-(char *)buff;
}
/* output solution in the form of nmea GSV sentence --------------------------*/
extern int outnmea_gsv(unsigned char *buff, const sol_t *sol,
                       const ssat_t *ssat)
{
    double az,el,snr;
    int i,j,k,n,sat,prn,sys,nmsg,sats[MAXSAT];
    char *p=(char *)buff,*q,*s,sum;
    
//    trace(3,"outnmea_gsv:\n");
    
    if (sol->stat<=SOLQ_NONE) {
        p+=sprintf(p,"$GPGSV,1,1,0,,,,,,,,,,,,,,,,");
        for (q=(char *)buff+1,sum=0;*q;q++) sum^=*q;
        p+=sprintf(p,"*%02X%c%c",sum,0x0D,0x0A);
        return p-(char *)buff;
    }
    /* GPGSV: gps/sbas */
    for (sat=1,n=0;sat<MAXSAT&&n<12;sat++) {
        sys=satsys(sat,&prn);
        if (sys!=SYS_GPS&&sys!=SYS_SBS) continue;
        if (ssat[sat-1].vs&&ssat[sat-1].azel[1]>0.0) sats[n++]=sat;
    }
    nmsg=n<=0?0:(n-1)/4+1;
    
    for (i=k=0;i<nmsg;i++) {
        s=p;
        p+=sprintf(p,"$GPGSV,%d,%d,%02d",nmsg,i+1,n);
        
        for (j=0;j<4;j++,k++) {
            if (k<n) {
                if (satsys(sats[k],&prn)==SYS_SBS) prn+=33-MINPRNSBS;
                az =ssat[sats[k]-1].azel[0]*R2D; if (az<0.0) az+=360.0;
                el =ssat[sats[k]-1].azel[1]*R2D;
                snr=ssat[sats[k]-1].snr[0]*0.25;
                p+=sprintf(p,",%02d,%02.0f,%03.0f,%02.0f",prn,el,az,snr);
            }
            else p+=sprintf(p,",,,,");
        }
        p+=sprintf(p,",1"); /* L1C/A */
        for (q=s+1,sum=0;*q;q++) sum^=*q; /* check-sum */
        p+=sprintf(p,"*%02X%c%c",sum,0x0D,0x0A);
    }
    /* GLGSV: glonass */
    for (sat=1,n=0;sat<MAXSAT&&n<12;sat++) {
        if (satsys(sat,&prn)!=SYS_GLO) continue;
        if (ssat[sat-1].vs&&ssat[sat-1].azel[1]>0.0) sats[n++]=sat;
    }
    nmsg=n<=0?0:(n-1)/4+1;
    
    for (i=k=0;i<nmsg;i++) {
        s=p;
        p+=sprintf(p,"$GLGSV,%d,%d,%02d",nmsg,i+1,n);
        
        for (j=0;j<4;j++,k++) {
            if (k<n) {
                satsys(sats[k],&prn); prn+=64; /* 65-99 */
                az =ssat[sats[k]-1].azel[0]*R2D; if (az<0.0) az+=360.0;
                el =ssat[sats[k]-1].azel[1]*R2D;
                snr=ssat[sats[k]-1].snr[0]*0.25;
                p+=sprintf(p,",%02d,%02.0f,%03.0f,%02.0f",prn,el,az,snr);
            }
            else p+=sprintf(p,",,,,");
        }
        p+=sprintf(p,",1"); /* L1C/A */
        for (q=s+1,sum=0;*q;q++) sum^=*q; /* check-sum */
        p+=sprintf(p,"*%02X%c%c",sum,0x0D,0x0A);
    }
    /* GAGSV: galileo */
    for (sat=1,n=0;sat<MAXSAT&&n<12;sat++) {
        if (satsys(sat,&prn)!=SYS_GAL) continue;
        if (ssat[sat-1].vs&&ssat[sat-1].azel[1]>0.0) sats[n++]=sat;
    }
    nmsg=n<=0?0:(n-1)/4+1;
    
    for (i=k=0;i<nmsg;i++) {
        s=p;
        p+=sprintf(p,"$GAGSV,%d,%d,%02d",nmsg,i+1,n);
        
        for (j=0;j<4;j++,k++) {
            if (k<n) {
                satsys(sats[k],&prn); /* 1-36 */
                az =ssat[sats[k]-1].azel[0]*R2D; if (az<0.0) az+=360.0;
                el =ssat[sats[k]-1].azel[1]*R2D;
                snr=ssat[sats[k]-1].snr[0]*0.25;
                p+=sprintf(p,",%02d,%02.0f,%03.0f,%02.0f",prn,el,az,snr);
            }
            else p+=sprintf(p,",,,,");
        }
        p+=sprintf(p,",7"); /* L1BC */
        for (q=s+1,sum=0;*q;q++) sum^=*q; /* check-sum */
        p+=sprintf(p,"*%02X%c%c",sum,0x0D,0x0A);
    }
    return p-(char *)buff;
}
