/**
 * ojw_motion
 * A library for Dongbu HerkuleX Servo
 *
 * Copyright 2014 Dongbu Robot
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

example )   sudo g++ -o testRun Test.cpp HerkuleX2.cpp HerkuleX2.h -lpthread -lwiringPi

 */
#ifndef __OJW_MOTION
#define __OJW_MOTION


// Jinwook On
//#include <time.h>	
#include <stdio.h>
#include <string.h>

#include <sys/time.h>
#include <unistd.h>
#include  <pthread.h>
typedef unsigned char byte;

extern int m_nTty;				//file description


///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
////////////////////
// add : Jinwook On 20160621
// get : Control Algorithm & Architecture from Open Jig Ware 

// Address
#define _ADDRESS_TORQUE_CONTROL         			52
#define _ADDRESS_LED_CONTROL  				53
#define _ADDRESS_VOLTAGE  					54
#define _ADDRESS_TEMPERATURE  				55
#define _ADDRESS_PRESENT_CONTROL_MODE  	56
#define _ADDRESS_TICK  						57
#define _ADDRESS_CALIBRATED_POSITION  	58

#define _SIZE_STRING						50

// Model
#define _MODEL_DRS_0101 1
#define _MODEL_DRS_0102 2
#define _MODEL_DRS_0201 3
#define _MODEL_DRS_0202 4
#define _MODEL_DRS_0401 5
#define _MODEL_DRS_0402 6
#define _MODEL_DRS_0601 7
#define _MODEL_DRS_0602 8
#define _MODEL_DRS_0603 9

#define _SIZE_MEMORY      256
#define _SIZE_MOTOR_MAX   16

#define _ID_BROADCASTING  254




#define _HEADER1 				0
#define _HEADER2 				1
#define _SIZE 					2
#define _ID 						3
#define _CMD 					4
#define _CHECKSUM1 				5
#define _CHECKSUM2 				6
#define _SIZE_PACKET_HEADER 	7

#define _FLAG_STOP 				0x01
#define _FLAG_MODE_SPEED  		0x02
#define _FLAG_LED_GREEN  		0x04
#define _FLAG_LED_BLUE  		0x08
#define _FLAG_LED_RED  			0x10
#define _FLAG_NO_ACTION 		0x20

typedef struct 
{
	bool bEnable;
	int nID;
	int nAddress_First;
	int nAddress_Length;
}SRead_t;
typedef struct 
{
    int nID;

    int nDir;

    float fLimitUp;    // limit Max value - 0: No use
    float fLimitDn;    // limit Min value - 0: No use
    // Center position(Evd : Engineering value of degree)
    float fCenterPos;

    float fOffsetAngle_Display; // �������� ȭ����� ���� Offset

    // gear ratio
    float fMechMove;
    float fDegree;
}SParam_Axis_t;

typedef struct 
{
    bool bEn;

    int nDir;
    //Center
    float fCenterPos;

    float fMechMove;
    float fDegree;

    float fLimitUp;    // Limit - 0: Ignore
    float fLimitDn;    // Limit - 0: Ignore

    int nID;

    int nPos;
    float fTime;
    float fSpeed;

    int nFlag; // 76[543210] NoAction(5), Red(4), Blue(3), Green(2), Mode(    
}SMot_t;

typedef struct {
	bool 	bEn;
	int 		*pnMot;
	float 	*pfXyz; // reserve
	int 		*pnLed;
	bool 	*pbEn;
	bool 	*pbType;
	int 		nTime;
	int 		nDelay;
	int 		nGroup;
	int 		nCmd;
	int 		nData0;
	int 		nData1;
	int 		nData2;
	int 		nData3;
	int 		nData4;
	int 		nData5;

	//char		strCaption[256];
} SMotionTable_t;

typedef struct {
	char 	strVersion[6];
	int 		nFrameSize;
	int 		nCnt_LineComment;
	int 		nPlayTime;
	int 		nCommentSize;
	int 		nRobotModelNum;
	int 		nMotorCnt;
	int 		nStartPosition;
	char 	strFileName[21];
	char 	strTableName[21];
	char 	strComment[256];
	SMotionTable_t *pSTable;
} SMotion_t;


typedef struct  			// Motor information
{
	int nMotorID;                    		// Motor ID
	int nMotorDir;                   		// Direction of Axis (0 - forward, 1 - inverse)
	float fLimit_Up;                 		// Max Angle(+)
	float fLimit_Down;               	// Min Angle(-)
	int nCenter_Evd;                 	// Pulse(Engineering value for 0 degree(Center Position))

	int nMechMove;                   	// Maximum Position ( Maximum Pulse value(Evd) )
	float fMechAngle;                	// It is a Degree when it moves in Maximum Position

	float fInitAngle;                		// Init position which you want it
	float fInitAngle2;               		// Init position which you want it(2'st)

	// Interference Axis(No use)
	int nInterference_Axis;          	// reserve(No use) - �̰� (-)���̸� ���� �� ����.
	float fW;                        		// reserve(No use) - Side �� ���� ���� ũ��(����)
	float fInterference_W;           	// reserve(No use) - �������� �յڷ� �پ��ٰ� �����ϰ� �ش� �������� ũ��(����)

	float fPos_Right;                		// reserve(No use) - ���� ������ ��ġ
	float fPos_Left;                 		// reserve(No use) - ���� ���� ��ġ

	float fInterference_Pos_Front;// reserve(No use) - �ش� �������� ���� ��ġ
	float fInterference_Pos_Rear;	// reserve(No use) - �ش� �������� ���� ��ġ

	// NickName
	char strNickName[_SIZE_STRING];              	// Nickname(32char)

	int nGroupNumber;                	// Group Number

	int nAxis_Mirror;                	// 0 ~ 253 : Motor ID of Mirroring one
	                                    		// -1      : there is no mirror motor.
	                                    		// -2 : there is no mirror motor(but it can has flip-direction(for using center), flip it from '0')

	int nMotorControlType;           	// Motor Control type => 0: Position, 1: Speed type
}SMotorInfo_t;



class CMotor 
{
public:
	CMotor();									// Initialize
	~CMotor();		 							// Destroy

	static void Open_Socket();//(const char *pcIp);//, int nPort);
	static void Close_Socket();
	void Open(const char *pcDevice, int nBaudrate);//, int nModel);		//serial port open
	void Close();								//serial port close

	void SetParam(int nAxis, int nRealID, int nDir, float fLimitUp, float fLimitDn, float fCenterPos, float fOffsetAngle_Display, float fMechMove, float fDegree);
	void SetParam(int nAxis, int nModel);
	void SetParam_RealID(int nAxis, int nRealID);		
	void SetParam_Dir(int nAxis, int nDir);		
	void SetParam_LimitUp(int nAxis, float fLimitUp);		
	void SetParam_LimitDown(int nAxis, float fLimitDn);		
	void SetParam_CenterEvdValue(int nAxis, float fCenterPos);		
	void SetParam_Display(int nAxis, float fOffsetAngle_Display);		
	void SetParam_MechMove(int nAxis, float fMechMove);		
	void SetParam_Degree(int nAxis, float fDegree);	

	bool SetParam_with_File(const char *strHeaderFile);

	static bool IsOpen_Socket();
	static bool IsOpen() { return ((m_nTty != 0) ? true : false); }
	bool IsStop() { return m_bStop; }
	bool IsEms() { return m_bEms; }

	void Stop(int nAxis);
	void Stop();
	void Ems();
	
	bool GetErrorCode(int nAxis); // Status 1
	bool IsError(int nAxis);
	bool IsWarning(int nAxis);
	////////////////////////////////////
	// Motor Control - Reset
	void Reboot();
	void Reboot(int nAxis);
	void Reset();
	void Reset(int nAxis);
	//void Reset_ErrorFlag();
	//void Reset_ErrorFlag(int nAxis);

	bool m_bIgnoredLimit;
	void SetLimitEn(bool bOn) { m_bIgnoredLimit = !bOn; }
	bool GetLimitEn() { return !m_bIgnoredLimit; }
	int Clip(int nLimitValue_Up, int nLimitValue_Dn, int nData);
	float Clip(float fLimitValue_Up, float fLimitValue_Dn, float fData);

	int CalcLimit_Evd(int nAxis, int nValue);	
	float CalcLimit_Angle(int nAxis, float fValue);
	
	int CalcTime_ms(int nTime);
	int CalcAngle2Evd(int nAxis, float fValue);
	float CalcEvd2Angle(int nAxis, int nValue);

	////////////////////////////////////
	// Motor Control - Torq On / Off
	void SetTorque(int nAxis, bool bDrvOn, bool bSrvOn); 	//torque on / Off
	void SetTorque(bool bDrvOn, bool bSrvOn);		//torque on / Off

	// baudrate
	void SetBaudrate(int nAxis, int nBaud); 	
	void SetMotorID(int nAxis, int nNewID);
	
	/////////////////////////////////////
	// Data Command(No motion) - just setting datas   		=> use with Send_Motor
	// ---- Position Control ----
	void Set(int nAxis, int nEvd);
	int 	Get(int nAxis);

	void Set_Angle(int nAxis, float fAngle);
	float Get_Angle(int nAxis);

	// ---- Speed Control ----
	void Set_Turn(int nAxis, int nEvd);
	int 	Get_Turn(int nAxis);

	int Get_Pos_Evd(int nAxis);
	float Get_Pos_Angle(int nAxis);
	/////////////////////////////////////
	// Led Control   									=> use with Send_Motor
	//void Set_Flag(int nAxis, int nFlag);
	void Clear_Flag();
	void Clear_Flag(int nAxis);
	void Set_Flag(int nAxis, bool bStop, bool bMode_Speed, bool bLed_Green, bool bLed_Blue, bool bLed_Red, bool bNoAction);
	void Set_Flag_Stop(int nAxis, bool bStop);
	void Set_Flag_Mode(int nAxis, bool bMode_Speed);
	void Set_Flag_Led(int nAxis, bool bGreen, bool bBlue, bool bRed);
	void Set_Flag_Led_Green(int nAxis, bool bGreen);
	void Set_Flag_Led_Blue(int nAxis, bool bBlue);
	void Set_Flag_Led_Red(int nAxis, bool bRed);
	void Set_Flag_NoAction(int nAxis, bool bNoAction);

	// 1111 1101
	int 	Get_Flag(int nAxis) { return m_aSMot[nAxis].nFlag; }
	int	Get_Flag_Mode(int nAxis);

	bool Get_Flag_Led_Green(int nAxis);
	bool Get_Flag_Led_Blue(int nAxis);
	bool Get_Flag_Led_Red(int nAxis);

	void TimerSet();			// Timer ����
	void TimerDestroy();		// Timer Destroy ( ������ ������ �ݵ�� Destroy�� �ϵ��� �Ѵ�. )
	unsigned long Timer();				// Timer ���� �� ��������� �ð� ���� return

	#define _TIME_MUL	1000
	class CTimer
	{
		public:
			CTimer() { m_bTimer = false; m_ulTimer; }
			~CTimer() {}
			void Set() { m_bTimer = true; gettimeofday(&m_tvTemp, NULL ); m_ulTimer = (m_tvTemp.tv_sec*_TIME_MUL) + (m_tvTemp.tv_usec/_TIME_MUL); }
			void Destroy() { m_bTimer = false; m_ulTimer; }
			unsigned long Get() { if (m_bTimer == true) { gettimeofday(&m_tvTemp, NULL );	 return (m_tvTemp.tv_sec*_TIME_MUL) + (m_tvTemp.tv_usec/_TIME_MUL) - m_ulTimer; } else return 0; }
		private:
			bool m_bTimer;
			unsigned long m_ulTimer;
			struct timeval m_tvTemp;
	};
	//////////////////////////////////////
	// Motor Control - Move Motor(Action)
	void Send_Led();
	void Send_Motor(int nMillisecond);

	//////////////////////////////////////
	// Wait
	void Wait_Ready();
	bool Wait_Motor(int nMilliseconds);
	bool Wait_Motor();
	bool Wait_Position(int nAxis, float fAngle, int nErrorTime);
	bool Wait_Delay(int nMilliseconds);


	CTimer m_CTmr_Motion;
	long	m_lWaitActionTimer;	// ���ؽð� Ÿ�̸�
	char WaitAction_SetTimer();	// �� ������ �߽����� Ÿ�̸Ӹ� �ʱ�ȭ
	char WaitAction_ByTimer(long t);	// WaitAction_SetTimer �� �ð����� �����ð��� �Ѿ������� üũ. �ѱ� ���� Wait

//#define _ADDRESS_TORQUE_CONTROL 			52
//#define _ADDRESS_LED_CONTROL 				53
//#define _ADDRESS_VOLTAGE 					54
//#define _ADDRESS_TEMPERATURE 				55
//#define _ADDRESS_PRESENT_CONTROL_MODE 	56
//#define _ADDRESS_TICK 						57
//#define _ADDRESS_CALIBRATED_POSITION		58

	void Read_Ram(int nAxis, int nAddress, int nLength);
	//void Read_Rom();
	void Read_Motor(int nAxis);
	void Read_Motor();
//	void Read_Motor(int nAxis);

	// Push Motor ID for checking(if you set a Motor ID with this function, you can get a feedback data with delay function)
	void Read_Motor_Push(int nAxis);
	// You can check your Motor ID for feedback, which you set or not.
	int Read_Motor_Index(int nAxis);
	bool Read_Motor_IsReceived();
	void Sync_Seq();

	// use this when you don't want to get some motor datas.
	void Read_Motor_Clear();

	// detail option
	void Read_Motor_Change_Address(int nAxis, int nAddress, int nLength);
	
	void Read_Motor_ShowMessage(bool bTrue);// { m_bShowMessage = bTrue; }
	
	//////////////////////////////////////
	// Setting
	//void Set_Ram(int nId, 
	//void Set_Rom(int nId, 
	
	//bool m_bProgEnd;	

	//void FileOpen(const char *strFileName, COjwDesignerHeader *pCHeader);
	void Motion_Play(const char *strFileName);
	void PlayFrame(SMotionTable_t STable);
	static void Write(byte *buffer, int nLength) {  }
private:
	
	SMotion_t m_SMotion;
	SMotorInfo_t m_aSMotorInfo[256];
	//unsigned char getChksum1(class DataPacket * buf);	//Check sum1 설정 
	//unsigned char getChksum2(unsigned char chksum1);	//Check sum2 설정	

	//void sendPacket(class DataPacket * buf);			//Packet 보내기
	//int receivePacket();
	void Init();
	//static bool m_bShowMessage;
	//int m_nSeq_Motor;
	//int m_nSeq_Motor_Back;
	unsigned long m_ulDelay;
	
	CTimer m_CTmr;


	CTimer m_CTmr_Timeout;



	int m_nModel;

	int m_nTimeout;
	
	SRead_t m_aSRead[_SIZE_MOTOR_MAX];
	int m_nReadCnt;
	int m_nMotorCnt;
	int m_nMotorCnt_Back;
	SParam_Axis_t m_aSParam_Axis[_SIZE_MOTOR_MAX];
	SMot_t m_aSMot[_SIZE_MOTOR_MAX];
	SMot_t m_aSMot_Prev[_SIZE_MOTOR_MAX];
	
	char m_acEn[_SIZE_MOTOR_MAX];

	bool m_bStop;
	bool m_bEms;
	bool m_bStart;
	
	void Push_Id(int nAxis);
	int Pop_Id();
	bool IsCmd(int nAxis);
	int GetID_By_Axis(int nAxis) { return (nAxis == 0xfe) ? 0xfe : m_aSMot[nAxis].nID; }

	bool BinaryFileOpen(const char *strFileName, SMotion_t *pSMotion);
};

#endif
