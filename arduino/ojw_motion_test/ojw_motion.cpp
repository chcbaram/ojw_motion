#include <Arduino.h> 
#include "ojw_motion.h"



#define _STR_EXT        "dmt"
#define _STR_EXT_UP     "DMT"
#define _STR_VER_V_12   "1.2"
#define _STR_VER_V_11   "1.1"
#define _STR_VER_V_10   "1.0"
#define _SIZE_FILE_NAME 21




COjwMotion::COjwMotion()
{
  file_list.cnt = 0;
  file_list.index = 0;
  file_list.cur_play = -1;

  motion_start_func = NULL;
  motion_motor_func = NULL;  
}

COjwMotion::~COjwMotion()
{
}

void COjwMotion::setMotionStartCallback(void (*start_func)(SMotionHeader_t *p_header))
{
  motion_start_func = start_func;
}

void COjwMotion::setMotionMotorCallback(bool (*motor_func)(SMotionHeader_t *p_header, SMotionTableDB_t *p_table))
{
  motion_motor_func = motor_func;
}

bool COjwMotion::addMotionFile(uint16_t number, char *name,  uint8_t *motion_ptr)
{
  bool ret = false;
  SMotionFileListNode_t *p_node;


  if (file_list.index < OJW_MOTION_LIST_COUNT)
  {
    file_list.p_node[file_list.index] = (SMotionFileListNode_t *)malloc(sizeof(SMotionFileListNode_t));

    if (file_list.p_node[file_list.index] != NULL)
    {
      p_node = file_list.p_node[file_list.index];

      p_node->number = number;
      p_node->p_motion_buf = motion_ptr;
      strcpy(p_node->motion_name, name);

      file_list.cnt++;

      ret = true;
    }
  }

  return ret;
}

bool COjwMotion::playMotion(uint16_t number)
{
  bool ret = true;
  int16_t index = -1;
  uint16_t i;


  for (i=0; i<file_list.cnt; i++)
  {
    if (file_list.p_node[i]->number == number)
    {
      index = i;
      break;
    }
  }

  Serial.println(index);

  if (index >= 0)
  {
    if (MotionLoadHeader(&m_SMotion, file_list.p_node[index]->p_motion_buf) == true)
    {
      if (motion_start_func != NULL)
      {
        (*motion_start_func)(&m_SMotion.SHeader);
      }
          
      for (i=0; i<m_SMotion.SHeader.nFrameSize; i++)
      {
        MotionGetTable(&m_SMotion, i);
    
        if (motion_motor_func != NULL)
        {
          if ((*motion_motor_func)(&m_SMotion.SHeader, &m_SMotion.STable) == true)
          {
            break;
          }
        }
        delay(m_SMotion.STable.nTime);  
      }
    }      
  }

  return ret;
}




bool COjwMotion::MotionLoadHeader(SMotionDB_t *pMotion, uint8_t *pData)
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

bool COjwMotion::MotionGetTable(SMotionDB_t *pMotion, uint16_t tableIndex)
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

