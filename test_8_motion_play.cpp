// compile : 
//     sudo g++ -o test8 test_8_motion_play.cpp HerkuleX2.cpp HerkuleX2.h -lpthread
// Run :
//     sudo ./test8

// 주의사항 : 모터의 각 방향의 셋팅이 DR-Sim 의 3D 모델링의 방향과 동일해야 합니다.
// motor.SetParam_Dir 함수 example 을 참조 바랍니다.

#include "HerkuleX2.h"

int main(int argc, char* argv[]) {

////// Start //////
	CMotor motor;

	// Comport Open
	motor.Open("/dev/ttyUSB0", 115200);

////// Init //////
	// Error Clear
	motor.Reset();
	// Torq On
	motor.SetTorque(true, true);

	// 첨부된 0~4 의 모터를 제어하는 모션이 저장된 모션 파일을 플레이 한다. (DR-Sim 에서 제작 가능)
	motor.Motion_Play("test5dof.dmt");


	printf("Done\n");
////// End //////
	// Port Close
	motor.Close();

	return 0;
}
