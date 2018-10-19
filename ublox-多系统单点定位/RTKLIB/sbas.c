/*------------------------------------------------------------------------------
* sbas.c : sbas functions
*
*          Copyright (C) 2007-2011 by T.TAKASU, All rights reserved.
*
* option : -DRRCENA  enable rrc correction
*          
* references :
*     [1] RTCA/DO-229C, Minimum operational performanc standards for global
*         positioning system/wide area augmentation system airborne equipment,
*         RTCA inc, November 28, 2001
*     [2] IS-QZSS v.1.1, Quasi-Zenith Satellite System Navigation Service
*         Interface Specification for QZSS, Japan Aerospace Exploration Agency,
*         July 31, 2009
*
* version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
* history : 2007/10/14 1.0  new
*           2009/01/24 1.1  modify sbspntpos() api
*                           improve fast/ion correction update
*           2009/04/08 1.2  move function crc24q() to rcvlog.c
*                           support glonass, galileo and qzss
*           2009/06/08 1.3  modify sbsupdatestat()
*                           delete sbssatpos()
*           2009/12/12 1.4  support glonass
*           2010/01/22 1.5  support ems (egnos message service) format
*           2010/06/10 1.6  added api:
*                               sbssatcorr(),sbstropcorr(),sbsioncorr(),
*                               sbsupdatecorr()
*                           changed api:
*                               sbsreadmsgt(),sbsreadmsg()
*                           deleted api:
*                               sbspntpos(),sbsupdatestat()
*           2010/08/16 1.7  not reject udre==14 or give==15 correction message
*                           (2.4.0_p4)
*           2011/01/15 1.8  use api ionppp()
*                           add prn mask of qzss for qzss L1SAIF
*-----------------------------------------------------------------------------*/
#include "rtklib.h"

/* decode sbas message ---------------------------------------------------------
* decode sbas message frame words and check crc
* args   : gtime_t time     I   reception time
*          int    prn       I   sbas satellite prn number
*          unsigned int *word I message frame words (24bit x 10)
*          sbsmsg_t *sbsmsg O   sbas message
* return : status (1:ok,0:crc error)
*-----------------------------------------------------------------------------*/
extern int sbsdecodemsg(gtime_t time, int prn, const unsigned int *words,
                        sbsmsg_t *sbsmsg)
{
    int i,j;
    unsigned char f[29];
    double tow;
    
//    printf("sbsdecodemsg: prn=%d\n",prn);
    
    if (time.time==0) return 0;
    tow=time2gpst(time,&sbsmsg->week);
    sbsmsg->tow=(int)(tow+DTTOL);
    sbsmsg->prn=prn;
    for (i=0;i<7;i++) for (j=0;j<4;j++) {
        sbsmsg->msg[i*4+j]=(unsigned char)(words[i]>>((3-j)*8));
    }
    sbsmsg->msg[28]=(unsigned char)(words[7]>>18)&0xC0;
    for (i=28;i>0;i--) f[i]=(sbsmsg->msg[i]>>6)+(sbsmsg->msg[i-1]<<2);
    f[0]=sbsmsg->msg[0]>>6;
    ublox_eph_flag=1;
    return crc24q(f,29)==(words[7]&0xFFFFFF); /* check crc */
}

