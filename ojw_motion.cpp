//#define _TEST_RECEIVE
/**
 * HerkuleX2
 * A library for DST HerkuleX Servo
 *
 * Copyright 2016 DST Robot
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; 
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General
 * Public License along with this library; if not, write to the
 * Free Software Foundation, Inc..
 * 
 * @author      Jinwook On ojw5014@hanmail.net
 * @modified    2016.06.23
 * @version     01.00.00 Released
 */
 
#include <iostream>
#include <cstring>

#include <termios.h>
#include <fcntl.h>
#include <stdio.h>

#include <unistd.h>
#include <sys/signal.h>
#include <sys/types.h>

#include "ojw_motion.h"

// Jinwook, On
#include "stdlib.h"
#include <sys/timeb.h>
#include "math.h"
#include <stdarg.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string.h>

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
// add : 20160621 Jinwook On in DST Robot
#define Round(dValue)     (((double)dValue > 0)?floor((double)dValue + 0.5):ceil((double)dValue - 0.5))
#define Roundf(fValue)    (((float)fValue > 0)?floor((float)fValue + 0.5f):ceil((float)fValue - 0.5f))

#define _TIME_DELAY	        0//10
#define _TIME_DELAY_THREAD  0

#define _CNT_RETRIEVE	5
int m_nRetrieve = 0;

bool m_bProgEnd;
char m_acRam[_SIZE_MOTOR_MAX][_SIZE_MEMORY];
char m_acRom[_SIZE_MOTOR_MAX][_SIZE_MEMORY];

char m_anPos[_SIZE_MOTOR_MAX];
char m_acStatus1[_SIZE_MOTOR_MAX];
char m_acStatus2[_SIZE_MOTOR_MAX];
int m_anAxis_By_ID[_SIZE_MOTOR_MAX];

bool m_bMultiTurn = false;

int m_nSeq_Receive = 0;	
int m_nSeq_Receive_Back = 0;

int GetAxis_By_ID(int nID) { return (nID == 0xfe) ? 0xfe : m_anAxis_By_ID[nID]; }
int m_nTty;	

#define _SIZE_BUFFER 1024
int m_nClientMotionFd;




bool m_bShowMessage;
void CMotor::Sync_Seq() { m_nSeq_Receive_Back= m_nSeq_Receive; }
void CMotor::Init()
{
	m_bShowMessage = false;

	m_nTimeout = 10;	
	m_ulDelay = 0;

	
	m_bIgnoredLimit = false;
	m_nTty = 0;				//file description
	m_nModel = 0;

	m_nClientMotionFd = -1;
		
	m_nSeq_Receive = 0;

	//m_bOpen = false;
	m_bStop = false;
	m_bEms = false;

	m_nMotorCnt_Back = m_nMotorCnt = 0;
	memset(m_acRam, 0, sizeof(char) * _SIZE_MEMORY);
	memset(m_acRom, 0, sizeof(char) * _SIZE_MEMORY);
	memset(m_aSParam_Axis, 0, sizeof(SParam_Axis_t) * _SIZE_MOTOR_MAX);
	memset(m_aSMot, 0, sizeof(SMot_t) * _SIZE_MOTOR_MAX);
	memset(m_aSMot_Prev, 0, sizeof(SMot_t) * _SIZE_MOTOR_MAX);

	memset(m_acStatus1, 0, _SIZE_MOTOR_MAX);
	memset(m_acStatus2, 0, _SIZE_MOTOR_MAX);
	memset(m_anAxis_By_ID, 0, _SIZE_MOTOR_MAX);
	
	memset(m_acEn, 0, _SIZE_MOTOR_MAX);
	memset(m_aSRead, 0, sizeof(SRead_t) * _SIZE_MOTOR_MAX);
	m_nReadCnt = 0;

	//m_nSeq_Receive = 0;
	//m_nSeq_Receive_Back = 0;
	Sync_Seq();

	memset(m_abReceivedPos, 0, sizeof(bool) * _SIZE_MOTOR_MAX);
	

	for (int i = 0; i < 256; i++) SetParam(i, _MODEL_DRS_0101);
	m_bProgEnd = false;	
}
CMotor::CMotor()									// Initialize
{
	Init();
}
CMotor::~CMotor()		 							// Destroy
{

}


	
void CMotor::Open(const char  *pcDevice, int nBaudrate)
{
	if (IsOpen() == false)
	{
	}
	if (IsOpen() == false)
	{
	}
}
void CMotor::Close()
{
	if (IsOpen() == true) 
	{
	}
}

//void CMotor::SetParam(int nId)
//{
//}
void CMotor::SetParam(int nAxis, int nRealID, int nDir, float fLimitUp, float fLimitDn, float fCenterPos, float fOffsetAngle_Display, float fMechMove, float fDegree)
{
    //if ((nAxis >= _CNT_MAX_MOTOR) || (nID >= _MOTOR_MAX)) return false;

    m_aSParam_Axis[nAxis].nID = m_aSMot[nAxis].nID = nRealID;
    m_aSParam_Axis[nAxis].nDir = m_aSMot[nAxis].nDir = nDir;
    m_aSParam_Axis[nAxis].fLimitUp = m_aSMot[nAxis].fLimitUp = fLimitUp;
    m_aSParam_Axis[nAxis].fLimitDn = m_aSMot[nAxis].fLimitDn = fLimitDn;
    m_aSParam_Axis[nAxis].fCenterPos = m_aSMot[nAxis].fCenterPos= fCenterPos;
    m_aSParam_Axis[nAxis].fOffsetAngle_Display = fOffsetAngle_Display;
    m_aSParam_Axis[nAxis].fMechMove = m_aSMot[nAxis].fMechMove = fMechMove;
    m_aSParam_Axis[nAxis].fDegree = m_aSMot[nAxis].fDegree = fDegree;
}
void CMotor::SetParam(int nAxis, int nModel)
{
	//printf("nAxis = %d, nModel = %d\r\n", nAxis, nModel);
	if (nModel == _MODEL_DRS_0603) m_bMultiTurn = true;
	else if (m_bMultiTurn == true) m_bMultiTurn = false;
	switch(nModel)
	{
		case _MODEL_DRS_0101: 
			SetParam_RealID(nAxis, nAxis);
			SetParam_Dir(nAxis, 0);
			SetParam_LimitUp(nAxis, 0);
			SetParam_LimitDown(nAxis, 0);
			SetParam_CenterEvdValue(nAxis, 512);
			SetParam_Display(nAxis, 0);
			SetParam_MechMove(nAxis, 1024);
			SetParam_Degree(nAxis, 333.3);
			break;
		case _MODEL_DRS_0102: 
			SetParam_RealID(nAxis, nAxis);
			SetParam_Dir(nAxis, 0);
			SetParam_LimitUp(nAxis, 0);
			SetParam_LimitDown(nAxis, 0);
			SetParam_CenterEvdValue(nAxis, 3196);
			SetParam_Display(nAxis, 0);
			SetParam_MechMove(nAxis, 6392);
			SetParam_Degree(nAxis, 360);
			break;
		case _MODEL_DRS_0201: 
			SetParam_RealID(nAxis, nAxis);
			SetParam_Dir(nAxis, 0);
			SetParam_LimitUp(nAxis, 0);
			SetParam_LimitDown(nAxis, 0);
			SetParam_CenterEvdValue(nAxis, 512);
			SetParam_Display(nAxis, 0);
			SetParam_MechMove(nAxis, 1024);
			SetParam_Degree(nAxis, 333.3);
			break;
		case _MODEL_DRS_0202: 
			SetParam_RealID(nAxis, nAxis);
			SetParam_Dir(nAxis, 0);
			SetParam_LimitUp(nAxis, 0);
			SetParam_LimitDown(nAxis, 0);
			SetParam_CenterEvdValue(nAxis, 3196);
			SetParam_Display(nAxis, 0);
			SetParam_MechMove(nAxis, 6392);
			SetParam_Degree(nAxis, 360);
			break;
		case _MODEL_DRS_0401: 
			SetParam_RealID(nAxis, nAxis);
			SetParam_Dir(nAxis, 0);
			SetParam_LimitUp(nAxis, 0);
			SetParam_LimitDown(nAxis, 0);
			SetParam_CenterEvdValue(nAxis, 1024);
			SetParam_Display(nAxis, 0);
			SetParam_MechMove(nAxis, 2048);
			SetParam_Degree(nAxis, 333.3);
			break;
		case _MODEL_DRS_0402: 
			SetParam_RealID(nAxis, nAxis);
			SetParam_Dir(nAxis, 0);
			SetParam_LimitUp(nAxis, 0);
			SetParam_LimitDown(nAxis, 0);
			SetParam_CenterEvdValue(nAxis, 16384);
			SetParam_Display(nAxis, 0);
			SetParam_MechMove(nAxis, 12962.099);
			SetParam_Degree(nAxis, 360);
			break;
		case _MODEL_DRS_0601: 
			SetParam_RealID(nAxis, nAxis);
			SetParam_Dir(nAxis, 0);
			SetParam_LimitUp(nAxis, 0);
			SetParam_LimitDown(nAxis, 0);
			SetParam_CenterEvdValue(nAxis, 1024);
			SetParam_Display(nAxis, 0);
			SetParam_MechMove(nAxis, 2048);
			SetParam_Degree(nAxis, 333.3);
			break;
		case _MODEL_DRS_0602: 
			SetParam_RealID(nAxis, nAxis);
			SetParam_Dir(nAxis, 0);
			SetParam_LimitUp(nAxis, 0);
			SetParam_LimitDown(nAxis, 0);
			SetParam_CenterEvdValue(nAxis, 16384);
			SetParam_Display(nAxis, 0);
			SetParam_MechMove(nAxis, 12962.099);
			SetParam_Degree(nAxis, 360);
		case _MODEL_DRS_0603: 
			SetParam_RealID(nAxis, nAxis);
			SetParam_Dir(nAxis, 0);
			SetParam_LimitUp(nAxis, 0);
			SetParam_LimitDown(nAxis, 0);
			SetParam_CenterEvdValue(nAxis, 0);
			SetParam_Display(nAxis, 0);
			SetParam_MechMove(nAxis, 12962.099);
			SetParam_Degree(nAxis, 360);
			break;
	}
}

void CMotor::SetParam_RealID(int nAxis, int nRealID) { m_aSParam_Axis[nAxis].nID = m_aSMot[nAxis].nID = nRealID; m_anAxis_By_ID[nRealID] = nAxis; }
void CMotor::SetParam_Dir(int nAxis, int nDir) { m_aSParam_Axis[nAxis].nDir = m_aSMot[nAxis].nDir = nDir; }
void CMotor::SetParam_LimitUp(int nAxis, float fLimitUp) { m_aSParam_Axis[nAxis].fLimitUp = m_aSMot[nAxis].fLimitUp = fLimitUp; }
void CMotor::SetParam_LimitDown(int nAxis, float fLimitDn) { m_aSParam_Axis[nAxis].fLimitDn = m_aSMot[nAxis].fLimitDn = fLimitDn; }
void CMotor::SetParam_CenterEvdValue(int nAxis, float fCenterPos) { m_aSParam_Axis[nAxis].fCenterPos = m_aSMot[nAxis].fCenterPos = fCenterPos; }
void CMotor::SetParam_Display(int nAxis, float fOffsetAngle_Display) { m_aSParam_Axis[nAxis].fOffsetAngle_Display = fOffsetAngle_Display; }
void CMotor::SetParam_MechMove(int nAxis, float fMechMove) { m_aSParam_Axis[nAxis].fMechMove = m_aSMot[nAxis].fMechMove = fMechMove; }
void CMotor::SetParam_Degree(int nAxis, float fDegree) { m_aSParam_Axis[nAxis].fDegree = m_aSMot[nAxis].fDegree = fDegree; }

void CMotor::Stop(int nAxis) // no stop flag setting
{
//	if (Get_Flag_Mode(nAxis) != 0)   // �ӵ�����
	Set_Turn(nAxis, 0);
//	Set_Flag_Stop(nAxis, true);
	Send_Motor(1000);
}
void CMotor::Stop()
{
//    for (int i = 0; i < _SIZE_MOTOR_MAX; i++) 
//    {
//	 if (Get_Flag_Mode(i) != 0)   // �ӵ�����
//        	Set(i, 0);
//	Set_Turn(nAxis, 0);
 //       Set_Flag_Stop(i, true);
//    }
    Set_Turn(254, 0);
    Send_Motor(100);
    m_bStop = true;
}	
void CMotor::Ems()
{
    Stop();
    SetTorque(false, false);
    m_bEms = true;
}

bool CMotor::GetErrorCode(int nAxis) { return m_acStatus1[nAxis]; }
bool CMotor::IsError(int nAxis)
{
	if (m_acStatus1[nAxis] != 0)
	{
		return true;		
	}
	return false;
/*
	0x01 : Exceed Input Voltage Limit
	0x02 : Exceed allowed POT limit
	0x04 : Exceed Temperature limit
	0x08 : Invalid Packet
	0x10 : Overload detected
	0x20 : Driver fault detected
	0x40 : EEP REG distorted
	0x80 : reserved
*/
}

bool CMotor::IsWarning(int nAxis)
{	
	// Status 2
	if ((m_acStatus2[nAxis] & 0x43) != 0)
	{
		return true;
	}
	return false;
	
/*
	0x01 : Moving flag
	0x02 : Inposition flag
	0x04 : Checksum Error
	0x08 : Unknown Command
	0x10 : Exceed REG range
	0x20 : Garbage detected
	0x40 : MOTOR_ON flag
	0x80 : reserved
	*/
}
//////////////////////////////////////////////////////////
// Reboot 
void CMotor::Reboot() { Reboot(_ID_BROADCASTING); }
void CMotor::Reboot(int nAxis)
{
	if (nAxis < 0xfe) Clear_Flag(nAxis);
       else
	{
	    for (int i = 0; i < _SIZE_MOTOR_MAX; i++) Clear_Flag(i);
	}

	int nID = m_aSMot[nAxis].nID;
	int nDefaultSize = _CHECKSUM2 + 1;
	byte pbyteBuffer[nDefaultSize];
	//(char *)pbyteBuffer = (char
	// Header
	pbyteBuffer[_HEADER1] = 0xff;
	pbyteBuffer[_HEADER2] = 0xff;
	// ID = 0xFE : ��ü���, 0xFD - �������Ͻ� ���� ���̵�
	pbyteBuffer[_ID] = (byte)(nID & 0xff);
	// Cmd
	pbyteBuffer[_CMD] = 0x09; // Reset

	//Packet Size
	pbyteBuffer[_SIZE] = (byte)((nDefaultSize) & 0xff);

	MakeCheckSum(nDefaultSize, pbyteBuffer);//, out pbyteBuffer[_CHECKSUM1], out pbyteBuffer[_CHECKSUM2]);

	SendPacket(pbyteBuffer, nDefaultSize);

	Clear_Flag();


// Initialize variable
	m_bStop = false;
	m_bEms = false;
	m_nMotorCnt_Back = m_nMotorCnt = 0;

}

///////////////////////////////////
// Motor Control - Reset
void CMotor::Reset() { Reset(_ID_BROADCASTING); }
void CMotor::Reset(int nAxis) 
{

}

int CMotor::Clip(int nLimitValue_Up, int nLimitValue_Dn, int nData)
{
    if (GetLimitEn() == false) return nData;

    int nRet = ((nData > nLimitValue_Up) ? nLimitValue_Up : nData);
    return ((nRet < nLimitValue_Dn) ? nLimitValue_Dn : nRet);
}
float CMotor::Clip(float fLimitValue_Up, float fLimitValue_Dn, float fData)
{
    if (GetLimitEn() == false) return fData;
    float fRet = ((fData > fLimitValue_Up) ? fLimitValue_Up : fData);
    return ((fRet < fLimitValue_Dn) ? fLimitValue_Dn : fRet);
}

int CMotor::CalcLimit_Evd(int nAxis, int nValue)
{

    if ((Get_Flag_Mode(nAxis) == 0) || (Get_Flag_Mode(nAxis) == 2))
    {
		//if ((m_aSMot[nAxis].fLimit_Down != 0) && (m_aSMot[nMot].fLimit_Down >= fValue)) fValue = m_aSMot[nMot].fLimit_Down;
		//if ((m_aSMot[nAxis].fLimit_Up != 0) && (m_aSMot[nMot].fLimit_Up <= fValue)) fValue = m_aSMot[nMot].fLimit_Up;
		//return fValue;













        int nPulse = nValue;
	 if (m_bMultiTurn == false)
	 {
	 	nValue &= 0x4000;
        	nValue &= 0x3fff;
	 }
 	 //int nLimit = 0x100000;
        int nUp = 0x100000;
        int nDn = -nUp;
        if (m_aSMot[nAxis].fLimitUp != 0) nUp = CalcAngle2Evd(nAxis, m_aSMot[nAxis].fLimitUp);
        if (m_aSMot[nAxis].fLimitDn != 0) nDn = CalcAngle2Evd(nAxis, m_aSMot[nAxis].fLimitDn);
        if (nUp < nDn) { int nTmp = nUp; nUp = nDn; nDn = nTmp; }
        return (Clip(nUp, nDn, nValue) | nPulse);
    }

//	int nDn = CalcAngle2Evd(nAxis, m_aSMot[nAxis].fLimitDn);
//	int nUp = CalcAngle2Evd(nAxis, m_aSMot[nAxis].fLimitUp);
//	if ((nDn != 0) && (nDn >= nValue)) nValue = nDn;
//	if ((nUp != 0) && (nUp <= nValue)) nValue = nUp;    
    return nValue;
}
float CMotor::CalcLimit_Angle(int nAxis, float fValue)
{
    if ((Get_Flag_Mode(nAxis) == 0) || (Get_Flag_Mode(nAxis) == 2))
    {
		if ((m_aSMot[nAxis].fLimitDn != 0) && (m_aSMot[nAxis].fLimitDn >= fValue)) fValue = m_aSMot[nAxis].fLimitDn;
		if ((m_aSMot[nAxis].fLimitUp != 0) && (m_aSMot[nAxis].fLimitUp <= fValue)) fValue = m_aSMot[nAxis].fLimitUp;
    }
    return fValue;
}

int CMotor::CalcTime_ms(int nTime)
{
    // 1 Tick �� 11.2 ms => 1:11.2=x:nTime => x = nTime / 11.2
    return ((nTime <= 0) ? 1 : (int)Roundf((float)nTime / 11.2f));
}
int CMotor::CalcAngle2Evd(int nAxis, float fValue)
{
	fValue *= ((m_aSMot[nAxis].nDir == 0) ? 1.0f : -1.0f);
	int nData = 0;
	if (Get_Flag_Mode(nAxis) != 0)   // �ӵ�����
	{
	    nData = (int)Roundf(fValue);
		//printf("Speed Turn");
	}
	else
	{
	    // ��ġ����
	    nData = (int)Roundf((m_aSMot[nAxis].fMechMove * fValue) / m_aSMot[nAxis].fDegree);
	    nData = nData + (int)Roundf(m_aSMot[nAxis].fCenterPos);
		
	    //printf("[%d]Angle(%.2f), Mech(%.2f), Degree(%.2f), Center(%.2f), nData(%d)\r\n", nAxis, fValue, m_aSMot[nAxis].fMechMove, m_aSMot[nAxis].fDegree, m_aSMot[nAxis].fCenterPos, nData);
	}

	return nData;
}
float CMotor::CalcEvd2Angle(int nAxis, int nValue)
{
	float fValue = ((m_aSMot[nAxis].nDir == 0) ? 1.0f : -1.0f);
	float fValue2 = 0.0f;
	if (Get_Flag_Mode(nAxis) != 0)   // �ӵ�����
	    fValue2 = (float)nValue * fValue;
	else                                // ��ġ����
	    fValue2 = (float)(((m_aSMot[nAxis].fDegree * ((float)(nValue - (int)Roundf(m_aSMot[nAxis].fCenterPos)))) / m_aSMot[nAxis].fMechMove) * fValue);
	return fValue2;
}


void CMotor::SetBaudrate(int nAxis, int nBaud) 	//torque on / Off
{
	int nID = m_aSMot[nAxis].nID;
	int i = 0;
	byte byBaud = 0x10;
	byBaud = ((nBaud == 1000000) ? 0x01 : 
		((nBaud == 666666) ? 0x02 : 
			((nBaud == 500000) ? 0x03 : 
				((nBaud == 400000) ? 0x04 : 
					((nBaud == 250000) ? 0x07 : 
						// baudrate : 0x10(16 - 115200) , 0x22(34 - 57600)
						((nBaud == 200000) ? 0x09 : ((nBaud == 57600) ? 0x22 : 0x10)) // 57600(0x22) : 115200(0x10)
					)
				)
			)
		)
	);
	byte pbyteBuffer[50];
	// Data
	pbyteBuffer[i++] = 4; // 4 �� �������� ���
	////////
	pbyteBuffer[i++] = 0x01;// ������ �������� ������
	pbyteBuffer[i++] = byBaud;
	Make_And_Send_Packet(nID, 0x01, i, pbyteBuffer);
	//pbyteBuffer = null;
}

void CMotor::SetMotorID(int nAxis, int nNewID) 	//torque on / Off
{
	int nID = m_aSMot[nAxis].nID;
	int i = 0;
	
	byte pbyteBuffer[50];
	// Data
	pbyteBuffer[i++] = 6; // 6 �� �������� ���
	////////
	pbyteBuffer[i++] = 0x01;// ������ �������� ������
	pbyteBuffer[i++] = (nNewID & 0xff);
	Make_And_Send_Packet(nID, 0x01, i, pbyteBuffer);
	//pbyteBuffer = null;
}
////////////////////////////////////
// Motor Control - Torq On / Off
void CMotor::SetTorque(int nAxis, bool bDrvOn, bool bSrvOn) 	//torque on / Off
{
}
void CMotor::SetTorque(bool bDrvOn, bool bSrvOn) { SetTorque((int)_ID_BROADCASTING, bDrvOn, bSrvOn); }

/////////////////////////////////////
// Data Command(No motion) - just setting datas   		=> use with Send_Motor
// ---- Position Control ----
void CMotor::Set(int nAxis, int nEvd)
{
	if ((m_bStop == true) || (m_bEms == true) || (m_bProgEnd == true)) return;
	Push_Id(nAxis);
	Read_Motor_Push(nAxis);
	m_aSMot[nAxis].bEn = true; 
	Set_Flag_Mode(nAxis, false);
	m_aSMot[nAxis].nPos = CalcLimit_Evd(nAxis, nEvd); 
	Set_Flag_NoAction(nAxis, false);
	//Push_Id(nAxis);	
}
int CMotor::Get(int nAxis) { return m_aSMot[nAxis].nPos; }
            
void CMotor::Set_Angle(int nAxis, float fAngle)
{
	if ((m_bStop == true) || (m_bEms == true) || (m_bProgEnd == true)) return;
	Push_Id(nAxis);
	Read_Motor_Push(nAxis);

	m_aSMot_Prev[nAxis].nFlag = m_aSMot[nAxis].nFlag;
	m_aSMot_Prev[nAxis].bEn = m_aSMot[nAxis].bEn;
	m_aSMot_Prev[nAxis].nPos = m_aSMot[nAxis].nPos;
	//m_aSMot_Prev[nAxis].fMechMove = m_aSMot[nAxis].fMechMove;
	//m_aSMot_Prev[nAxis].fDegree = m_aSMot[nAxis].fDegree;

	m_aSMot[nAxis].bEn = true; 
	Set_Flag_Mode(nAxis, false);
	m_aSMot[nAxis].nPos = CalcLimit_Evd(nAxis, CalcAngle2Evd(nAxis, fAngle)); 
	Set_Flag_NoAction(nAxis, false);
	//Push_Id(nAxis);	
}
float CMotor::Get_Angle(int nAxis) { return CalcEvd2Angle(nAxis, m_aSMot[nAxis].nPos); }



void CMotor::Send_Motor(int nMillisecond)
{
	if ((m_bStop == true) || (m_bEms == true)) return;
	
	m_nMotorCnt_Back = m_nMotorCnt; // ���߿� waitaction ���� ���

	//Sync_Seq();

	int nID;
	int i = 0;
	////////////////////////////////////////////////

	byte pbyteBuffer[256];//[1 + 4 * m_nMotorCnt];
	int nPos;
	int nFlag;
		
	// region S-Jog Time
	int nCalcTime = CalcTime_ms(nMillisecond);
	pbyteBuffer[i++] = (byte)(nCalcTime & 0xff);
	if (m_bMultiTurn == true) pbyteBuffer[i++] = (byte)((nCalcTime >> 8) & 0xff);
	int nCnt = m_nMotorCnt;//_SIZE_MOTOR_MAX;//m_nMotorCnt;
	for (int nAxis2 = 0; nAxis2 < nCnt; nAxis2++)
	{
		int nAxis = Pop_Id();//nAxis2;// i;//Pop_Id();
		//printf("ID=%d\r\n", nAxis);
		if (m_aSMot[nAxis].bEn == true)
		{
			//printf("ID=%d\r\n", nAxis);
			//nPos |= _JOG_MODE_SPEED << 10;  // �ӵ����� 
			// Position
			nPos = Get(nAxis);

			if (nPos < 0)
			{
			  nPos *= -1;
			  nPos |= 0x4000;
			}



			// ���ʹ� ���̵�(�ĸ鿡 �ٴ´�)
			nID = GetID_By_Axis(nAxis);
			pbyteBuffer[i++] = (byte)(nID & 0xff);

			m_aSMot[nAxis].bEn = false;
		}
		
	}

	Clear_Flag();
	m_ulDelay = nMillisecond;
	Wait_Ready();
}

void CMotor::Wait_Ready()
{
	m_CTmr.Set();
}





void CMotor::Motion_Play(const char *strFileName)
{
	if (BinaryFileOpen(strFileName, &m_SMotion) == true)
	{
		if (m_SMotion.nFrameSize > 0)
		{			
			m_bStart = true;

			WaitAction_SetTimer();

			printf("Frame=%d\r\n", m_SMotion.nFrameSize);
			for (int i = 0; i < m_SMotion.nFrameSize; i++)
			{
				printf("bEn=%d\r\n", m_SMotion.pSTable[i].bEn);
				if (m_SMotion.pSTable[i].bEn == true)
				{
					PlayFrame(m_SMotion.pSTable[i]);

					int nDelay = m_SMotion.pSTable[i].nTime + m_SMotion.pSTable[i].nDelay;

					if (nDelay > 0) WaitAction_ByTimer(nDelay);
					printf("nDelay=%d\r\n", nDelay);
				}
			}

			m_bStart = false;
			//m_bMotionEnd = false;
		}
	}
	

	// #region ���� ����� �޸� ����
	for(int j = 0; j < m_SMotion.nFrameSize; j++)
	{
		// test
//		printf("Value=[0]%d\n", m_SMotion.pSTable[0].bEn);
		
		free(m_SMotion.pSTable[j].pnMot);
		free(m_SMotion.pSTable[j].pnLed);
		free(m_SMotion.pSTable[j].pbEn);
		free(m_SMotion.pSTable[j].pbType);
	}
	free(m_SMotion.pSTable);
}

void CMotor::PlayFrame(SMotionTable_t STable)
{
	if ((m_bStop == false) && (m_bEms == false))// && (m_bMotionEnd == false))
	{
		//m_CMotor.ResetStop();
		SetTorque(true, true);
		for (int nAxis = 0; nAxis < m_SMotion.nMotorCnt; nAxis++)//.nMotorCnt; nAxis++)
		{
			if (
			    //(m_CHeader.pSMotorInfo[nAxis]. == EType_t._0102) || // ���ڴ��̰ų�
			    (m_aSMotorInfo[nAxis].nMotorControlType != 0) // ��ġ��� �ƴ϶�� //// Motor Control type => 0: Position, 1: Speed type
			    //(m_abEnc[nAxis] == true) || // ���ڴ��̰ų�
			    //(Grid_GetFlag_Type(m_nCurrntCell, nAxis) == true) // ��ġ��� �ƴ϶��
			    )
			{
			    // ��忡 ���� ������ Ʋ�����⿡ ��� ���ú��� ���� �Ѵ�.
			    Set_Flag_Mode(nAxis, m_aSMotorInfo[nAxis].nMotorControlType);
			    SetParam_Dir(nAxis, m_aSMotorInfo[nAxis].nMotorDir);

			    //float fTmpVal = (float)Math.Round(Convert.ToSingle(OjwGrid.GetData(nLine, nAxis)));
			    int nVal = STable.pnMot[nAxis];//CalcAngle2Evd(nAxis, fTmpVal);
			    if (nVal < 0)
			    {
			        nVal *= -1;
			        nVal |= 0x4000;
			    }
			    Set_Turn(nAxis, nVal);

			    Set_Flag_Led(nAxis, 
			        Get_Flag_Led_Green(STable.pnLed[nAxis]),
			        Get_Flag_Led_Blue(STable.pnLed[nAxis]),
			        Get_Flag_Led_Red(STable.pnLed[nAxis])
			        );
			}
			else
			{
			    // ��忡 ���� ������ Ʋ�����⿡ ��� ���ú��� ���� �Ѵ�.
			    Set_Flag_Mode(nAxis, m_aSMotorInfo[nAxis].nMotorControlType);
			    SetParam_Dir(nAxis, m_aSMotorInfo[nAxis].nMotorDir);

//printf("[PlayFrame][%d]:%d\r\n", nAxis, STable.pnMot[nAxis]);
//SetParam_Dir(nAxis, 0);
			    Set(nAxis, STable.pnMot[nAxis]);
			    printf("[%d]%d, %d, %.2f, %.2f\r\n", nAxis, STable.pnMot[nAxis], Get(nAxis), CalcEvd2Angle(nAxis, STable.pnMot[nAxis]), Get_Angle(nAxis));//Get_Angle(nAxis));
			    Set_Flag_Led(nAxis,
			        Get_Flag_Led_Green(STable.pnLed[nAxis]),
			        Get_Flag_Led_Blue(STable.pnLed[nAxis]),
			        Get_Flag_Led_Red(STable.pnLed[nAxis])
			        );
			}
		}
		Send_Motor(STable.nTime);
		// Sound & Buzz
		//m_CMotor.Mpsu_Play_HeadLed_Buzz(STable.nData4, STable.nData3);
	}
}


#define _STR_EXT        "dmt"
#define _STR_EXT_UP     "DMT"
#define _STR_VER_V_12   "1.2"
#define _STR_VER_V_11   "1.1"
#define _STR_VER_V_10   "1.0"
#define _SIZE_FILE_NAME 21

bool CMotor::BinaryFileOpen(const char *strFileName, SMotion_t * pSMotion)
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
			int nVersion = 11;
		}
		else
		{			
			if ( strncmp(_STR_VER_V_12, &pSMotion->strVersion[3], strlen(_STR_VER_V_12)) == 0 )
			{
				int nVersion = 12;
			}
			else
			{			
				printf("[Action]version mismatch\n");
				fclose(pfileAction);
				return false;		
			}	
		}
	}

	// title
	char strTitle[21];

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
                  
		printf("==Done(Frame Size = %d)==\r\n", pSMotion->nFrameSize);


		// #region ���� ���
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

			//En
		    // #region Enable
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

			      printf("Mot[%d] = %d\r\n", nAxis, sData);//pSMotion->pSTable[j].pnMot[nAxis] );
		        }
		    }
		    // #endregion Motor
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

