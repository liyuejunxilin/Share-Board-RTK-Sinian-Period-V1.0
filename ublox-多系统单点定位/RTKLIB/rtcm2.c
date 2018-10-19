#include "rtklib.h"

/* decode rtcm ver.2 message -------------------------------------------------*/
extern int decode_rtcm2(rtcm_t *rtcm)
{
    double zcnt;
    int staid,seqno,stah,ret=0,type=getbitu(rtcm->buff,8,6);
    
//    trace(3,"decode_rtcm2: type=%2d len=%3d\n",type,rtcm->len);
    
    if ((zcnt=getbitu(rtcm->buff,24,13)*0.6)>=3600.0) {
//        trace(2,"rtcm2 modified z-count error: zcnt=%.1f\n",zcnt);
        return -1;
    }
    adjhour(rtcm,zcnt);
    staid=getbitu(rtcm->buff,14,10);
    seqno=getbitu(rtcm->buff,37, 3);
    stah =getbitu(rtcm->buff,45, 3);
//    if (seqno-rtcm->seqno!=1&&seqno-rtcm->seqno!=-7) 
//			{
//        trace(2,"rtcm2 message outage: seqno=%d->%d\n",rtcm->seqno,seqno);
//      }
    rtcm->seqno=seqno;
    rtcm->stah =stah;
    
    if (rtcm->outtype) {
        sprintf(rtcm->msgtype,"RTCM %2d (%4d) zcnt=%7.1f staid=%3d seqno=%d",
                type,rtcm->len,zcnt,staid,seqno);
    }
    if (type==3||type==22||type==23||type==24) {
//        if (rtcm->staid!=0&&staid!=rtcm->staid)
//					{
//           trace(2,"rtcm2 station id changed: %d->%d\n",rtcm->staid,staid);
//         }
        rtcm->staid=staid;
    }
    if (rtcm->staid!=0&&staid!=rtcm->staid) {
//        trace(2,"rtcm2 station id invalid: %d %d\n",staid,rtcm->staid);
        return -1;
    }
		//暂时禁用
    switch (type) {
        case  1: ret=decode_type1 (rtcm); break;
        case  3: ret=decode_type3 (rtcm); break;
        case  9: ret=decode_type1 (rtcm); break;
        case 14: ret=decode_type14(rtcm); break;
        case 16: ret=decode_type16(rtcm); break;
        case 17: ret=decode_type17(rtcm); break;
        case 18: ret=decode_type18(rtcm); break;
        case 19: ret=decode_type19(rtcm); break;
        case 22: ret=decode_type22(rtcm); break;
        case 23: ret=decode_type23(rtcm); break; /* not supported */
        case 24: ret=decode_type24(rtcm); break; /* not supported */
        case 31: ret=decode_type31(rtcm); break; /* not supported */
        case 32: ret=decode_type32(rtcm); break; /* not supported */
        case 34: ret=decode_type34(rtcm); break; /* not supported */
        case 36: ret=decode_type36(rtcm); break; /* not supported */
        case 37: ret=decode_type37(rtcm); break; /* not supported */
        case 59: ret=decode_type59(rtcm); break; /* not supported */
    }
    if (ret>=0) {
        if (1<=type&&type<=99) rtcm->nmsg2[type]++; else rtcm->nmsg2[0]++;
    }
    return ret;
}

/* adjust hourly rollover of rtcm 2 time -------------------------------------*/
static void adjhour(rtcm_t *rtcm, double zcnt)
{
    double tow,hour,sec;
    int week;
 //暂时禁用
    /* if no time, get cpu time */
    if (rtcm->time.time==0) rtcm->time=utc2gpst(timeget());
    tow=time2gpst(rtcm->time,&week);
    hour=floor(tow/3600.0);
    sec=tow-hour*3600.0;
    if      (zcnt<sec-1800.0) zcnt+=3600.0;
    else if (zcnt>sec+1800.0) zcnt-=3600.0;
    rtcm->time=gpst2time(week,hour*3600+zcnt);
	
}
