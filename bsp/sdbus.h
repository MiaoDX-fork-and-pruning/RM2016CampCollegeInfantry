#ifndef _SDBUS_H_
#define _SDBUS_H_
#include <stdint.h>

typedef struct{
double xf;
double xtr;
double xrr;
	
double w1;
double w2;
double w3;
double w4;
}SDBUS;

typedef struct{
double kp;
double ki;
double kd;
}SPID;

extern SDBUS sdbus;
extern SPID spid;

void SDBUS_Enc(const SDBUS* sdbus,unsigned char* sdbuf);
void SDBUS_Dec(SDBUS* sdbus,const unsigned char* sdbuf);
void SDBUS_Reset(SDBUS* psdbus);

void SPID_Dec(SPID* spid,const unsigned char* spbuf);
void SPID_POS_Dec(SPID* spid,const unsigned char* spbuf);

#endif
