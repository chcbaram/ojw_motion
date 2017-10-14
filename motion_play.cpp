//#include "ojw_motion.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>



typedef uint8_t byte;




typedef struct {
  bool     bEn;
  int32_t *pnMot;
  float   *pfXyz; // reserve
  int32_t *pnLed;
  bool    *pbEn;
  bool    *pbType;
  int32_t  nTime;
  int32_t  nDelay;
  int32_t  nGroup;
  int32_t  nCmd;
  int32_t  nData0;
  int32_t  nData1;
  int32_t  nData2;
  int32_t  nData3;
  int32_t  nData4;
  int32_t  nData5;
} SMotionTable_t;

typedef struct {
  char    strVersion[6];
  int32_t nFrameSize;
  int32_t nCnt_LineComment;
  int32_t nPlayTime;
  int32_t nCommentSize;
  int32_t nRobotModelNum;
  int32_t nMotorCnt;
  int32_t nStartPosition;
  char  strTableName[21];
  SMotionTable_t *pSTable;
} SMotion_t;




SMotion_t m_SMotion;








bool BinaryFileOpen(const char *strFileName, SMotion_t * pSMotion);
uint8_t *BinaryFileLoad(const char *strFileName);
void MainMotionInclude(const char *strFileName);



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


//char *file_name = "test5dof.dmt";
char *file_name = "test.dmt";

int main(int argc, char* argv[]) {
  uint8_t *pMotionBuffer;

  BinaryFileOpen(file_name, &m_SMotion);

  MainMotionInclude(file_name);

  pMotionBuffer = BinaryFileLoad(file_name);

  if (pMotionBuffer != NULL)
  {
    printf("motionSize : %d\n", (int)sizeof(SMotion));

    if (MotionLoadHeader(&SMotion, pMotionBuffer) == true)
    {
      printf("nFrameSize      \t %d\n", SMotion.SHeader.nFrameSize);
      printf("nCommentSize    \t %d\n", SMotion.SHeader.nCommentSize);
      printf("nCnt_LineComment\t %d\n", SMotion.SHeader.nCnt_LineComment);
      printf("nPlayTime       \t %d\n", SMotion.SHeader.nPlayTime);
      printf("nRobotModelNum  \t %d\n", SMotion.SHeader.nRobotModelNum);
      printf("nMotorCnt       \t %d\n", SMotion.SHeader.nMotorCnt);

      for (int i=0; i<SMotion.SHeader.nFrameSize; i++)
      {
        MotionGetTable(&SMotion, i);

        for (int nAxis = 0; nAxis < SMotion.SHeader.nMotorCnt; nAxis++)
        {
          printf("Mot[%d] = %d\n", nAxis, SMotion.STable.pnMot[nAxis]);
        }
        printf("---\n");
      }
    }

    free(pMotionBuffer);
  }

	return 0;
}


void MainMotionInclude(const char *strFileName)
{
  FILE *pfileAction;
  char szFilename[256];
  int file_size;

  sprintf(szFilename, "%s", strFileName);
  if ( ( pfileAction = fopen(szFilename, "rb") ) == NULL )
  {
    printf("[BinaryFileOpen] File[%s](Binary) open error\n", szFilename);
    return;
  }

  fseek( pfileAction, 0, SEEK_END );
  file_size = ftell( pfileAction );
  fseek( pfileAction, 0, SEEK_SET );

  uint8_t *pbyTmp = (byte *)malloc(sizeof(byte) * file_size);

  fread(pbyTmp, sizeof(char), file_size, pfileAction);
  fclose(pfileAction);



  if ( ( pfileAction = fopen("ojw_dmt.h", "w") ) == NULL )
  {
    printf("[BinaryFileOpen] File[%s](Binary) open error\n", szFilename);
    return;
  }

  fprintf(pfileAction, "const uint8_t motion[%d] PROGMEM = \n", file_size);
  fprintf(pfileAction, "{ \n");

  for (int i=0; i<file_size; i++)
  {
    if (i < file_size-1)
    {
      fprintf(pfileAction, "0x%02X, ", pbyTmp[i]);
    }
    else
    {
      fprintf(pfileAction, "0x%02X ", pbyTmp[i]);
    }

    if (i%10 == 9)
    {
      fprintf(pfileAction, "\n");
    }
  }

  fprintf(pfileAction, "\n}; \n");

  free(pbyTmp);
  fclose(pfileAction);
  return;
}



#define _STR_EXT        "dmt"
#define _STR_EXT_UP     "DMT"
#define _STR_VER_V_12   "1.2"
#define _STR_VER_V_11   "1.1"
#define _STR_VER_V_10   "1.0"
#define _SIZE_FILE_NAME 21


uint8_t *BinaryFileLoad(const char *strFileName)
{
  FILE *pfileAction;
  char szFilename[256];
  int file_size;

  sprintf(szFilename, "%s", strFileName);
  if ( ( pfileAction = fopen(szFilename, "rb") ) == NULL )
  {
    printf("[BinaryFileOpen] File[%s](Binary) open error\n", szFilename);
    return NULL;
  }

  fseek( pfileAction, 0, SEEK_END );
  file_size = ftell( pfileAction );
  fseek( pfileAction, 0, SEEK_SET );

  uint8_t *pbyTmp = (byte *)malloc(sizeof(byte) * file_size);

  fread(pbyTmp, sizeof(char), file_size, pfileAction);

  return pbyTmp;
}

bool BinaryFileOpen(const char *strFileName, SMotion_t * pSMotion)
{
  bool bFileOpened = false;
  FILE *pfileAction;
  char szFilename[256];

  sprintf(szFilename, "%s", strFileName);
  if ( ( pfileAction = fopen(szFilename, "rb") ) == NULL )
  {
    printf("[BinaryFileOpen] File[%s](Binary) open error\n", szFilename);
    return false;
  }
  bFileOpened = true;

  // header mark & version load
  memset(pSMotion->strVersion, 0, sizeof(char) * 6);
  fread(pSMotion->strVersion, sizeof(char), 6, pfileAction);

  if (
    ( strncmp(_STR_EXT, pSMotion->strVersion, strlen(_STR_EXT)) != 0 ) &&
    ( strncmp(_STR_EXT_UP, pSMotion->strVersion, strlen(_STR_EXT_UP)) != 0 )
    )
  {
    printf("[BinaryFileOpen]header mark not founded \n");
    fclose(pfileAction);
    return false;
  }

  // version check
  int nVersion = 10;
  if ( strncmp(_STR_VER_V_10, &pSMotion->strVersion[3], strlen(_STR_VER_V_10)) != 0 )
  {
    if ( strncmp(_STR_VER_V_11, &pSMotion->strVersion[3], strlen(_STR_VER_V_11)) == 0 )
    {
      nVersion = 11;
    }
    else
    {
      if ( strncmp(_STR_VER_V_12, &pSMotion->strVersion[3], strlen(_STR_VER_V_12)) == 0 )
      {
        nVersion = 12;
      }
      else
      {
        printf("[Action]version mismatch\n");
        fclose(pfileAction);
        return false;
      }
    }
  }


  if (nVersion == 10)
  {
    memset(pSMotion->strTableName, 0, sizeof(char) * 21);
    fread(pSMotion->strTableName, sizeof(char), 21, pfileAction);
    printf("Table Name = %s\r\n", pSMotion->strTableName);


    // Start Position(1)
    int nMemorySize = 14;
    byte *pbyTmp = (byte *)malloc(sizeof(byte) * nMemorySize);
    if (pbyTmp == NULL)
    {
      printf("[BinaryFileOpen]memory alloc error(Header)\n");
      fclose(pfileAction);
      return false;
    }


    fread(pbyTmp, sizeof(byte), nMemorySize, pfileAction);
    int nPos = 0;
    pSMotion->nStartPosition = (int)((pbyTmp[nPos] >= 0) ? pbyTmp[nPos] : 0);
    nPos++;
    // MotionFrame(2), Comment(2), Caption(2), PlayTime(4), RobotModelNumber(2), MotorCnt(1)
    // Size
    pSMotion->nFrameSize   = (int)(pbyTmp[nPos] + pbyTmp[nPos + 1] * 256); nPos += 2;
    pSMotion->nCommentSize = (int)(pbyTmp[nPos] + pbyTmp[nPos + 1] * 256); nPos += 2;
    pSMotion->nCnt_LineComment = (int)(pbyTmp[nPos] + pbyTmp[nPos + 1] * 256); nPos += 2;
    pSMotion->nPlayTime = (int)(pbyTmp[nPos] + pbyTmp[nPos + 1] * 256 + pbyTmp[nPos + 2] * 256 * 256 + pbyTmp[nPos + 3] * 256 * 256 * 256); nPos += 4;
    pSMotion->nRobotModelNum = (int)(pbyTmp[nPos] + pbyTmp[nPos + 1] * 256); nPos += 2;
    pSMotion->nMotorCnt = (int)(pbyTmp[nPos++]);
    // Size - MotionFrame, Comment, Caption, PlayTime

    printf("nFrameSize      \t %d\n", pSMotion->nFrameSize);
    printf("nCommentSize    \t %d\n", pSMotion->nCommentSize);
    printf("nCnt_LineComment\t %d\n", pSMotion->nCnt_LineComment);
    printf("nPlayTime       \t %d\n", pSMotion->nPlayTime);
    printf("nRobotModelNum  \t %d\n", pSMotion->nRobotModelNum);
    printf("nMotorCnt       \t %d\n", pSMotion->nMotorCnt);



    printf("==Done(Frame Size = %d)==\r\n", pSMotion->nFrameSize);


    pSMotion->pSTable = (SMotionTable_t *)malloc(sizeof(SMotionTable_t) * pSMotion->nFrameSize);//new SMotionTable_t[pSMotion->nFrameSize];
    for(int j = 0; j < pSMotion->nFrameSize; j++)
    {
        pSMotion->pSTable[j].pnMot  = (int *)malloc(sizeof(int)   * pSMotion->nMotorCnt);
        pSMotion->pSTable[j].pnLed  = (int *)malloc(sizeof(int)   * pSMotion->nMotorCnt);//new int[pSMotion->nMotorCnt];
        pSMotion->pSTable[j].pbEn   = (bool *)malloc(sizeof(bool) * pSMotion->nMotorCnt);//new bool[pSMotion->nMotorCnt];
        pSMotion->pSTable[j].pbType = (bool *)malloc(sizeof(bool) * pSMotion->nMotorCnt);//new bool[pSMotion->nMotorCnt];
    }

    int nH = pSMotion->nFrameSize;
    int nData;
    short sData;


    nMemorySize = 35 + pSMotion->nMotorCnt * 2;//15 + 24;
    byte *pbyteData = (byte *)malloc(sizeof(byte) * nMemorySize);
    if (pbyTmp == NULL)
    {
      printf("[BinaryFileOpen]memory alloc error(Header)\n");
      fclose(pfileAction);
      return false;
    }



    for (int j = 0; j < nH; j++)
    {
        nPos = 0;
        fread(pbyteData, sizeof(byte), nMemorySize, pfileAction);

        int nEn = pbyteData[nPos++];
        pSMotion->pSTable[j].bEn = ((nEn & 0x01) != 0) ? true : false;
        // #endregion Enable
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // #region Motor
        int nMotorCntMax = pSMotion->nMotorCnt;//(int)Math.Max(pSMotion->nMotorCnt, m_CHeader.nMotorCnt);
        // 0-Index, 1-En, 2 ~ 24, 25 - speed, 26 - delay, 27,28,29,30 - Data0-3, 31 - time, 32 - caption
        for (int nAxis = 0; nAxis < nMotorCntMax; nAxis++)
        {
       if (nAxis >= m_SMotion.nMotorCnt) nPos += 2;
            else if (nAxis >= pSMotion->nMotorCnt) pSMotion->pSTable[j].pnMot[nAxis] = 0;//0.0f;// �� ���ͼ����� ���� �ʴٸ� �� �κ��� 0 ���� ä�� ��
            else
            {
              memcpy(&nData, &pbyteData[nPos], sizeof(byte) * 2); nPos += 2;
                //nData = (int)(pbyteData[nPos] + pbyteData[nPos + 1] * 256); nPos += 2;
                sData = (short)(nData & 0x0fff);
                if ((sData & 0x800) != 0) sData -= 0x1000;

                pSMotion->pSTable[j].pnLed[nAxis] = (int)((nData >> 12) & 0x07);
                pSMotion->pSTable[j].pbType[nAxis] = (bool)(((nData & 0x8000) != 0) ? true : false);
                pSMotion->pSTable[j].pbEn[nAxis] = (bool)((sData == 0x7ff) ? false : true);

                if (sData == 0x7ff)
                    pSMotion->pSTable[j].pnMot[nAxis] = 0;//0.0f;
                else
                    pSMotion->pSTable[j].pnMot[nAxis] = sData;//(int)CalcEvd2Angle(nAxis, (int)sData);

            printf("Mot[%d] = %d\n", nAxis, sData);//pSMotion->pSTable[j].pnMot[nAxis] );
            }
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        // #region Speed(2), Delay(2), Group(1), Command(1), Data0(2), Data1(2)
        // Speed
        pSMotion->pSTable[j].nTime = (int)(pbyteData[nPos] + pbyteData[nPos + 1] * 256); nPos += 2;

        // Delay
        pSMotion->pSTable[j].nDelay = (int)(pbyteData[nPos] + pbyteData[nPos + 1] * 256); nPos += 2;

        // Group
        pSMotion->pSTable[j].nGroup = (int)(pbyteData[nPos++]);

        // Command
        pSMotion->pSTable[j].nCmd = (int)(pbyteData[nPos++]);

        // Data0
        pSMotion->pSTable[j].nData0 = (int)(pbyteData[nPos] + pbyteData[nPos + 1] * 256); nPos += 2;
        // Data1
        pSMotion->pSTable[j].nData1 = (int)(pbyteData[nPos] + pbyteData[nPos + 1] * 256); nPos += 2;
        //
        pSMotion->pSTable[j].nData2 = 0; //nPos++;
        pSMotion->pSTable[j].nData3 = 0; //nPos++;
        pSMotion->pSTable[j].nData4 = 0; //nPos++;
        pSMotion->pSTable[j].nData5 = 0; //nPos++;
        // #endregion Speed(2), Delay(2), Group(1), Command(1), Data0(2), Data1(2)
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        // #region �߰��� Frame ��ġ �� �ڼ�
        nPos += 24;
        //nPos += 4;
        //nPos += 4;
        //nPos += 4;

        //nPos += 4;
        //nPos += 4;
        //nPos += 4;
        // #endregion �߰��� Frame ��ġ �� �ڼ�

      printf("%d\r\n", nPos);
    }

    // �� ���� comment & caption �ʿ� ����.
    free(pbyteData);
    free(pbyTmp);
    pbyteData = NULL;
    pbyTmp = NULL;
  }
  fclose(pfileAction);
  return true;
}

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
