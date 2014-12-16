#include "../config.h"
#include "../ext.h"
#include "thermostat.h"

#define MOTOR_MINUS PORTD4
#define FAN_ON		PORTD5
#define MOTOR_OVR	PORTD6
#define MOTOR_PLUS	PORTD7

extern int16_t ain_act_val[];
extern uint8_t dio_status[];
static uint8_t tsWrite(subidx_t * pSubidx, uint8_t Len, uint8_t *pBuf);
static uint8_t tsPoolAct(subidx_t * pSubidx, uint8_t sleep);
static uint8_t tsReadAct(subidx_t * pSubidx, uint8_t *pLen, uint8_t *pBuf);

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
static uint8_t tsState;
static int32_t tsTCnt;
static int32_t tsCalibrate;
static int32_t tsPosCur;
static int32_t tsRefPath;
static int32_t tsPosExp;
static uint16_t tsPosExpS;
static int16_t tsPosTolerance;

static int16_t tsTust;
static int16_t tsPidPrevAct;
static int32_t tsPidCnt;
static int32_t tsPidIVal;

void tsConfig(void){
  DDRD &= ~(1<<MOTOR_OVR);
  DDRD |= (1<<MOTOR_MINUS) | (1<<MOTOR_PLUS);
  PORTD &= ~((1<<MOTOR_OVR) | (1<<MOTOR_MINUS) | (1<<MOTOR_PLUS));

  tsTust = 13700; // 23*32768/55;
  tsPosTolerance=128;
  
  indextable_t * pIndex;
  pIndex = getFreeIdxOD();    // In27
  if(pIndex!=NULL){
    pIndex->sidx.Place = objDin;
    pIndex->sidx.Type =  objPinNPN;
    pIndex->sidx.Base = 27;
    extRegisterOD(pIndex);
  }
  

  pIndex = getFreeIdxOD();    // Ai6
  if(pIndex!=NULL){
    pIndex->sidx.Place = objAin;
    pIndex->sidx.Type =  objArefInt1;
    pIndex->sidx.Base = 6;
    extRegisterOD(pIndex);
  }
  
  pIndex = getFreeIdxOD();  // Tust/55*32768
  if(pIndex!=NULL){       
    pIndex->cbRead  =  NULL;
    pIndex->cbWrite =  &tsWrite;
    pIndex->cbPoll  =  NULL;
    pIndex->sidx.Place = objUsrExt;
    pIndex->sidx.Type =  objInt16;
    pIndex->sidx.Base = 35;
  }
  
  pIndex = getFreeIdxOD();  // PosExp
  if(pIndex!=NULL){
    pIndex->cbRead  =  &tsReadAct;
    pIndex->cbWrite =  NULL;
    pIndex->cbPoll  =  &tsPoolAct;
    pIndex->sidx.Place = objUsrExt;
    pIndex->sidx.Type =  objUInt16;
    pIndex->sidx.Base = 36;
  }

  tsState=TS_STATE_PREINIT;
}
void tsTick(){
  // ain_act_val[6]
  // dio_status[1] & 8

  if((dio_status[1] & 8)!=0 && tsPidCnt>-80000L){  // Door is open
    tsPidCnt=-100000L; // 1000 sec
    tsPosExp=0;
    tsPidPrevAct=ain_act_val[6];
  } else {
    tsPidCnt++;
  }
  if(tsPidCnt>=0 && (tsPidCnt&0xFF)==0){
    int16_t tAct=ain_act_val[6];
    int16_t e=tsTust-tAct;
    tsPidIVal+=e/2;
    if(tsPidIVal>=(tsCalibrate*1024)){
      tsPidIVal=(tsCalibrate*1024)-1;
    } else if(tsPidIVal<0){
      tsPidIVal=0;
    }
    if(tsPidCnt>13567){  // 135.67 sec
      int32_t tmp;
      tmp=tsPidIVal/1024+e/2+(tsPidPrevAct-tAct)/3;
      tsPidPrevAct=tAct;
      tsPidCnt=0;
      if(tmp>=tsCalibrate){
        tmp=tsCalibrate-1;
      } else if(tmp<0){
        tmp=0;
      }
      tsPosExp=(tsPosExp+tmp)/2;
      if(tsPosTolerance>8){
        tsPosTolerance/=2;
      } else{
        tsPosTolerance=8;
      }
    }
  }
  switch(tsState){
    case TS_STATE_PREINIT:
    tsTCnt=256;
    tsPosCur=-1;
    tsState=TS_STATE_INIT;
    tsCalibrate=0;
    break;
    case TS_STATE_INIT:
    if(--tsTCnt==0){
      tsState=TS_STATE_REF_NEG;
      tsTCnt=48;
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
    if(tsTCnt>48 && (PIND & (1<<MOTOR_OVR))!=0){
      PORTD&=~(1<<MOTOR_MINUS);
      tsState=TS_STATE_WAIT;
      tsCalibrate=tsTCnt;
      
      tsPosCur=0;
      tsRefPath=0;
      tsPidCnt=0;
      
      tsPidIVal=tsCalibrate*256;  // initial value=1/4
      tsPosExp=tsPidIVal/1024;
      tsPidPrevAct=ain_act_val[6];
    }
    break;
    case TS_STATE_WAIT:
    if(tsPosExp-tsPosTolerance>tsPosCur){
      tsTCnt=48;
      tsPosTolerance=128;
      PORTD &= ~(1<<MOTOR_MINUS);
      PORTD|=1<<MOTOR_PLUS;
      tsState=TS_STATE_RUN_POS;
    } else if(tsPosExp+tsPosTolerance<tsPosCur){
      tsTCnt=48;
      tsPosTolerance=128;
      PORTD &= ~(1<<MOTOR_PLUS);
      PORTD|=1<<MOTOR_MINUS;
      if(tsPosExp<(tsRefPath>>2)){
        tsRefPath=0;
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
    tsRefPath++;
    if(tsTCnt>0){
      tsTCnt--;
    } else if((PIND & (1<<MOTOR_OVR))!=0){
      PORTD &= ~((1<<MOTOR_MINUS) | (1<<MOTOR_PLUS));
      tsPosCur=0;
      tsRefPath=0;
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
  int16_t tmp=((int16_t)pBuf[1]<<8) | pBuf[0];
  if(tmp>9532){  // t>16 && t<30
    tsTust=tmp<17873?tmp:17873;
  }
  return MQTTSN_RET_ACCEPTED;
}
static uint8_t tsPoolAct(subidx_t * pSubidx, uint8_t sleep){
  return ((uint16_t)tsPosExp!=tsPosExpS)?1:0;
}
static uint8_t tsReadAct(subidx_t * pSubidx, uint8_t *pLen, uint8_t *pBuf){
  tsPosExpS=(uint16_t)tsPosExp;
  *pLen = 2;
  *(uint16_t *)pBuf = tsPosExpS;
  return MQTTSN_RET_ACCEPTED;
}
