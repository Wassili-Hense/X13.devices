#include "../config.h"
#include "thermostat.h"

#define FAN_ON		PORTD5
#define MOTOR_MINUS PORTD4
#define MOTOR_PLUS	PORTD6
#define MOTOR_OVR	PORTD7

static uint8_t tsWrite(subidx_t * pSubidx, uint8_t Len, uint8_t *pBuf);
//static uint8_t tsPoolAct(subidx_t * pSubidx, uint8_t sleep);
//static uint8_t tsReadAct(subidx_t * pSubidx, uint8_t *pLen, uint8_t *pBuf);

enum {
  TS_STATE_PREINIT=0,
  TS_STATE_INIT,
  TS_STATE_REF_NEG,
  TS_STATE_REF_POS,
  TS_STATE_WAIT,
  TS_STATE_RUN_NEG,
  TS_STATE_RUN_ZERO,
  TS_STATE_RUN_POS,
} tsSTATE_E;
static uint8_t tsState=TS_STATE_PREINIT;
static uint16_t tsTCnt;
static int16_t tsCalibrate;
static int16_t tsPosCur;
//static int16_t tsPosCurS;
static int16_t tsPosExp;
static uint8_t  tsPrcntExp;


void tsClean(){
}
void tsConfig(void){
  indextable_t * pIndex;
  pIndex = getFreeIdxOD();
  if(pIndex!=NULL){
    pIndex->cbRead  =  NULL;
    pIndex->cbWrite =  &tsWrite;
    pIndex->cbPoll  =  NULL;
    pIndex->sidx.Place = objUsrExt;
    pIndex->sidx.Type =  objUInt8;
    pIndex->sidx.Base = 26;
  }
  
  //pIndex = getFreeIdxOD();
  //if(pIndex!=NULL){
  //pIndex->cbRead  =  &tsReadAct;
  //pIndex->cbWrite =  NULL;
  //pIndex->cbPoll  =  &tsPoolAct;
  //pIndex->sidx.Place = objUsrExt;
  //pIndex->sidx.Type =  objInt16;
  //pIndex->sidx.Base = 4;
  //}
  
  DDRD |= (1<<MOTOR_MINUS) | (1<<MOTOR_PLUS);
  PORTD &= ~((1<<MOTOR_MINUS) | (1<<MOTOR_PLUS));
}
void tsTick(){
  switch(tsState){
  case TS_STATE_PREINIT:
    tsTCnt=192;
    tsPosCur=-1;
    tsState=TS_STATE_INIT;
    tsCalibrate=0;
    break;
  case TS_STATE_INIT:
    if(--tsTCnt==0){
      tsState=TS_STATE_REF_NEG;
      tsTCnt=32;
      PORTD|=1<<MOTOR_PLUS;
    }
    break;
  case TS_STATE_REF_NEG:
    if(tsTCnt>0){
      tsTCnt--;
    } else if((PIND & (1<<MOTOR_OVR))!=0){
      PORTD&=~(1<<MOTOR_PLUS);
      PORTD|=1<<MOTOR_MINUS;
      tsState=TS_STATE_REF_POS;
    }
    break;
  case TS_STATE_REF_POS:
    tsTCnt++;
    if(tsTCnt>32 && (PIND & (1<<MOTOR_OVR))!=0){
      PORTD&=~(1<<MOTOR_MINUS);
      tsState=TS_STATE_WAIT;
      tsCalibrate=tsTCnt;
      tsPosCur=0;
      tsPosExp=0;
      {
        uint32_t tmp=tsCalibrate;
        tmp*=tsPrcntExp;
        tsPosExp=(int16_t)(tmp>>8);
      }
    }
    break;
  case TS_STATE_WAIT:
    if(tsPosExp-16>tsPosCur){
      tsTCnt=32;
      PORTD &= ~(1<<MOTOR_MINUS);
      PORTD|=1<<MOTOR_PLUS;
      tsState=TS_STATE_RUN_POS;
      } else if(tsPosExp+16<tsPosCur){
      tsTCnt=32;
      PORTD &= ~(1<<MOTOR_PLUS);
      PORTD|=1<<MOTOR_MINUS;
      if(tsPosExp<(tsCalibrate>>3)){
        tsState=TS_STATE_RUN_ZERO;
        }else{
        tsState=TS_STATE_RUN_NEG;
      }
    }
    break;
  case TS_STATE_RUN_ZERO:
    tsPosCur--;
    if(tsTCnt>0){
      tsTCnt--;
      } else if((PIND & (1<<MOTOR_OVR))!=0){
      PORTD &= ~((1<<MOTOR_MINUS) | (1<<MOTOR_PLUS));
      tsPosCur=0;
      tsState=TS_STATE_WAIT;
    }
    break;
  case TS_STATE_RUN_NEG:
    tsPosCur--;
    if(tsTCnt>0){
      tsTCnt--;
      } else if((PIND & (1<<MOTOR_OVR))!=0){
      PORTD &= ~((1<<MOTOR_MINUS) | (1<<MOTOR_PLUS));
      tsPosCur=0;
      tsState=TS_STATE_WAIT;
    }
    if(tsPosCur<=tsPosExp){
      PORTD &= ~((1<<MOTOR_MINUS) | (1<<MOTOR_PLUS));
      tsState=TS_STATE_WAIT;
    }
    break;
  case TS_STATE_RUN_POS:
    tsPosCur++;
    if(tsTCnt>0){
      tsTCnt--;
      } else if((PIND & (1<<MOTOR_OVR))!=0){
      PORTD &= ~((1<<MOTOR_MINUS) | (1<<MOTOR_PLUS));
      tsPosCur=tsCalibrate;
      tsState=TS_STATE_WAIT;
    }
    if(tsPosCur>=tsPosExp){
      PORTD &= ~((1<<MOTOR_MINUS) | (1<<MOTOR_PLUS));
      tsState=TS_STATE_WAIT;
    }
    break;
  }
}


static uint8_t tsWrite(subidx_t * pSubidx, uint8_t Len, uint8_t *pBuf){
  tsPrcntExp=pBuf[0];
  if(tsCalibrate!=0){
    uint32_t tmp=tsCalibrate;
    tmp*=tsPrcntExp;
    tsPosExp=(int16_t)(tmp>>8);
  }
  return MQTTS_RET_ACCEPTED;
}
//static uint8_t tsPoolAct(subidx_t * pSubidx, uint8_t sleep){
//return (tsPosCur!=tsPosCurS)?1:0;
//}
//static uint8_t tsReadAct(subidx_t * pSubidx, uint8_t *pLen, uint8_t *pBuf){
//*pLen = 2;
//*(int16_t *)pBuf = tsPosCur;
//tsPosCurS=tsPosCur;
//return MQTTS_RET_ACCEPTED;
//}
