#include "ojw_dmt.h"


#include <Servo.h>

Servo motor[4];  // create servo object to control a servo



typedef struct {
  bool     bEn;
  int16_t  pnMot[16];
  uint8_t  pnLed[16];
  bool     pbEn[16];
  bool     pbType[16];
  int16_t  nTime;
  int16_t  nDelay;
  uint8_t  nGroup;
  uint8_t  nCmd;
  int16_t  nData0;
  int16_t  nData1;
  int16_t  nData2;
  int16_t  nData3;
  int16_t  nData4;
  int16_t  nData5;
} SMotionTableDB_t;

typedef struct {
  char    strVersion[6];
  char    strTableName[21];

  uint8_t  nStartPosition;
  uint16_t nFrameSize;
  uint16_t nCommentSize;
  uint16_t nCnt_LineComment;
  uint32_t nPlayTime;
  uint16_t nRobotModelNum;
  uint8_t  nMotorCnt;
} SMotionHeader_t;


typedef struct {
  uint8_t         *pData;
  SMotionHeader_t  SHeader;
  SMotionTableDB_t STable;
} SMotionDB_t;

bool MotionLoadHeader(SMotionDB_t *pMotion, uint8_t *pData);
bool MotionGetTable(SMotionDB_t *pMotion, uint16_t tableIndex);


SMotionDB_t SMotion;




void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  motor[0].attach(9);
}

void loop() {
 

  
  // put your main code here, to run repeatedly:

  if (Serial.available() > 0)
  {
    Serial.read();
    
    Serial.println("size : " + String(sizeof(SMotion)));
    
    if (MotionLoadHeader(&SMotion, motion) == true)
    {
      Serial.println("nFrameSize      \t " + String(SMotion.SHeader.nFrameSize));
      Serial.println("nCommentSize    \t " + String(SMotion.SHeader.nCommentSize));
      Serial.println("nCnt_LineComment\t " + String(SMotion.SHeader.nCnt_LineComment));
      Serial.println("nPlayTime       \t " + String(SMotion.SHeader.nPlayTime));
      Serial.println("nRobotModelNum  \t " + String(SMotion.SHeader.nRobotModelNum));
      Serial.println("nMotorCnt       \t " + String(SMotion.SHeader.nMotorCnt));
          
      for (int i=0; i<SMotion.SHeader.nFrameSize; i++)
      {
        MotionGetTable(&SMotion, i);

        Serial.println("nTime  : " + String(SMotion.STable.nTime));
        Serial.println("nDelay : " + String(SMotion.STable.nDelay));
        
        for (int nAxis = 0; nAxis < SMotion.SHeader.nMotorCnt; nAxis++)
        {
          Serial.println("Mot[" + String(nAxis) + "] = " + String(SMotion.STable.pnMot[nAxis]));
        }
        Serial.println("---\n");

        int val;
        val = map(SMotion.STable.pnMot[1], 0, 1023, 0, 180);
        motor[0].write(val);
        delay(SMotion.STable.nTime);
      }
    }  
  }
}






#define _STR_EXT        "dmt"
#define _STR_EXT_UP     "DMT"
#define _STR_VER_V_12   "1.2"
#define _STR_VER_V_11   "1.1"
#define _STR_VER_V_10   "1.0"
#define _SIZE_FILE_NAME 21


bool MotionLoadHeader(SMotionDB_t *pMotion, uint8_t *pData)
{

  pMotion->pData = pData;


  memcpy(pMotion->SHeader.strVersion, &pData[0], sizeof(char) * 6);

  if (
    ( strncmp(_STR_EXT, pMotion->SHeader.strVersion, strlen(_STR_EXT)) != 0 ) &&
    ( strncmp(_STR_EXT_UP, pMotion->SHeader.strVersion, strlen(_STR_EXT_UP)) != 0 )
    )
  {
    return false;
  }

  // version check
  int nVersion = 10;
  if ( strncmp(_STR_VER_V_10, &pMotion->SHeader.strVersion[3], strlen(_STR_VER_V_10)) != 0 )
  {
    if ( strncmp(_STR_VER_V_11, &pMotion->SHeader.strVersion[3], strlen(_STR_VER_V_11)) == 0 )
    {
      nVersion = 11;
    }
    else
    {
      if ( strncmp(_STR_VER_V_12, &pMotion->SHeader.strVersion[3], strlen(_STR_VER_V_12)) == 0 )
      {
        nVersion = 12;
      }
      else
      {
        return false;
      }
    }
  }


  if (nVersion == 10)
  {
    memcpy(pMotion->SHeader.strTableName, &pData[6], sizeof(char) * 21);


    // Start Position(1)
    int nPos = 27;

    pMotion->SHeader.nStartPosition = (int)((pData[nPos] >= 0) ? pData[nPos] : 0);
    nPos++;
    // MotionFrame(2), Comment(2), Caption(2), PlayTime(4), RobotModelNumber(2), MotorCnt(1)
    // Size
    pMotion->SHeader.nFrameSize       = (int)(pData[nPos] + pData[nPos + 1] * 256); nPos += 2;
    pMotion->SHeader.nCommentSize     = (int)(pData[nPos] + pData[nPos + 1] * 256); nPos += 2;
    pMotion->SHeader.nCnt_LineComment = (int)(pData[nPos] + pData[nPos + 1] * 256); nPos += 2;
    pMotion->SHeader.nPlayTime        = (int)(pData[nPos] + pData[nPos + 1] * 256 + pData[nPos + 2] * 256 * 256 + pData[nPos + 3] * 256 * 256 * 256); nPos += 4;
    pMotion->SHeader.nRobotModelNum   = (int)(pData[nPos] + pData[nPos + 1] * 256); nPos += 2;
    pMotion->SHeader.nMotorCnt        = (int)(pData[nPos++]);
    // Size - MotionFrame, Comment, Caption, PlayTime
  }
  else
  {
    return false;
  }

  return true;
}

bool MotionGetTable(SMotionDB_t *pMotion, uint16_t tableIndex)
{
  uint32_t nPos;
  uint8_t *pData;
  uint16_t nMemorySize;
  int32_t  nData;
  int16_t  sData;

  pData = pMotion->pData;

  nMemorySize = 35 + pMotion->SHeader.nMotorCnt * 2;
  nPos = 41 + nMemorySize * tableIndex;

  int nEn = pData[nPos++];
  pMotion->STable.bEn = ((nEn & 0x01) != 0) ? true : false;

  int nMotorCntMax = pMotion->SHeader.nMotorCnt;
  // 0-Index, 1-En, 2 ~ 24, 25 - speed, 26 - delay, 27,28,29,30 - Data0-3, 31 - time, 32 - caption
  for (int nAxis = 0; nAxis < nMotorCntMax; nAxis++)
  {
    if (nAxis >= pMotion->SHeader.nMotorCnt) nPos += 2;
    else if (nAxis >= pMotion->SHeader.nMotorCnt) pMotion->STable.pnMot[nAxis] = 0;
    else
    {
      memcpy(&nData, &pData[nPos], sizeof(byte) * 2); nPos += 2;

      sData = (short)(nData & 0x0fff);
      if ((sData & 0x800) != 0) sData -= 0x1000;

      pMotion->STable.pnLed[nAxis]  = (uint8_t)((nData >> 12) & 0x07);
      pMotion->STable.pbType[nAxis] = (bool)(((nData & 0x8000) != 0) ? true : false);
      pMotion->STable.pbEn[nAxis]   = (bool)((sData == 0x7ff) ? false : true);

      if (sData == 0x7ff)
        pMotion->STable.pnMot[nAxis] = 0;
      else
        pMotion->STable.pnMot[nAxis] = sData;
    }
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // #region Speed(2), Delay(2), Group(1), Command(1), Data0(2), Data1(2)
  // Speed
  pMotion->STable.nTime = (int16_t)(pData[nPos] + pData[nPos + 1] * 256); nPos += 2;

  // Delay
  pMotion->STable.nDelay = (int16_t)(pData[nPos] + pData[nPos + 1] * 256); nPos += 2;

  // Group
  pMotion->STable.nGroup = (uint8_t)(pData[nPos++]);

  // Command
  pMotion->STable.nCmd = (uint8_t)(pData[nPos++]);

  // Data0
  pMotion->STable.nData0 = (int16_t)(pData[nPos] + pData[nPos + 1] * 256); nPos += 2;

  // Data1
  pMotion->STable.nData1 = (int16_t)(pData[nPos] + pData[nPos + 1] * 256); nPos += 2;
  //
  pMotion->STable.nData2 = 0; //nPos++;
  pMotion->STable.nData3 = 0; //nPos++;
  pMotion->STable.nData4 = 0; //nPos++;
  pMotion->STable.nData5 = 0; //nPos++;


  return true;
}

