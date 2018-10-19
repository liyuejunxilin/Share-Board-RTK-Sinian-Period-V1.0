#include "rtklib.h"

/* convert stearm ------------------------------------------------------------*/
extern void strconv(stream_t *str,strconv_t *conv, unsigned char *buff, int n)
{
    int i,ret;

    for (i=0;i<n;i++) {
        
        /* input rtcm 2 messages */
        if (conv->itype==STRFMT_RTCM2) {
            ret=input_rtcm2(&conv->rtcm,buff[i]);
//            rtcm2rtcm(&conv->out,&conv->rtcm,ret,conv->stasel);
        }
        /* input rtcm 3 messages */
        else if (conv->itype==STRFMT_RTCM3) {
            ret=input_rtcm3(&conv->rtcm,buff[i]);
//            rtcm2rtcm(&conv->out,&conv->rtcm,ret,conv->stasel);
        }
        /* input receiver raw messages ，其他接收机类型文件*/
//        else {
//            ret=input_raw(&conv->raw,conv->itype,buff[i]);
//            raw2rtcm(&conv->out,&conv->raw,ret);
//        }
        /* write obs and nav data messages to stream */
//        switch (ret) {
//            case 1: write_obs(conv->out.time,str,conv); break;
//            case 2: write_nav(conv->out.time,str,conv); break;
//        }
    }
    /* write cyclic nav data and station info messages to stream */
//    write_nav_cycle(str,conv);
//    write_sta_cycle(str,conv);
}

/* copy received data from receiver rtcm to rtcm -----------------------------*/
static void rtcm2rtcm(rtcm_t *out, const rtcm_t *rtcm, int ret, int stasel)
{
    int i,sat,prn;
    
    out->time=rtcm->time;
    
    if (!stasel) out->staid=rtcm->staid;
    
    if (ret==1) {
        for (i=0;i<rtcm->obs.n;i++) {
            out->obs.data[i]=rtcm->obs.data[i];
        }
        out->obs.n=rtcm->obs.n;
    }
    else if (ret==2) {
        sat=rtcm->ephsat;
        switch (satsys(sat,&prn)) {
            case SYS_GLO: out->nav.geph[prn-1]=rtcm->nav.geph[prn-1]; break;
            case SYS_GPS:
            case SYS_GAL:
            case SYS_QZS:
            case SYS_CMP: out->nav.eph [sat-1]=rtcm->nav.eph [sat-1]; break;
        }
        out->ephsat=sat;
    }
    else if (ret==5) {
        if (!stasel) out->sta=rtcm->sta;
    }
    else if (ret==9) {
        matcpy(out->nav.utc_gps,rtcm->nav.utc_gps,4,1);
        matcpy(out->nav.utc_glo,rtcm->nav.utc_glo,4,1);
        matcpy(out->nav.utc_gal,rtcm->nav.utc_gal,4,1);
        matcpy(out->nav.utc_qzs,rtcm->nav.utc_qzs,4,1);
        matcpy(out->nav.ion_gps,rtcm->nav.ion_gps,8,1);
        matcpy(out->nav.ion_gal,rtcm->nav.ion_gal,4,1);
        matcpy(out->nav.ion_qzs,rtcm->nav.ion_qzs,8,1);
        out->nav.leaps=rtcm->nav.leaps;
    }
}

/* copy received data from receiver raw to rtcm ------------------------------*/
static void raw2rtcm(rtcm_t *out, const raw_t *raw, int ret)
{
    int i,sat,prn;
    
    out->time=raw->time;
    
    if (ret==1) {
        for (i=0;i<raw->obs.n;i++) {
            out->obs.data[i]=raw->obs.data[i];
        }
        out->obs.n=raw->obs.n;
    }
    else if (ret==2) {
        sat=raw->ephsat;
        switch (satsys(sat,&prn)) {
            case SYS_GLO: out->nav.geph[prn-1]=raw->nav.geph[prn-1]; break;
            case SYS_GPS:
            case SYS_GAL:
            case SYS_QZS:
            case SYS_CMP: out->nav.eph [sat-1]=raw->nav.eph [sat-1]; break;
        }
        out->ephsat=sat;
    }
    else if (ret==9) {
        matcpy(out->nav.utc_gps,raw->nav.utc_gps,4,1);
        matcpy(out->nav.utc_glo,raw->nav.utc_glo,4,1);
        matcpy(out->nav.utc_gal,raw->nav.utc_gal,4,1);
        matcpy(out->nav.utc_qzs,raw->nav.utc_qzs,4,1);
        matcpy(out->nav.ion_gps,raw->nav.ion_gps,8,1);
        matcpy(out->nav.ion_gal,raw->nav.ion_gal,4,1);
        matcpy(out->nav.ion_qzs,raw->nav.ion_qzs,8,1);
        out->nav.leaps=raw->nav.leaps;
    }
}

