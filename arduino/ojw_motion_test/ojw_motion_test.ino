#include "ojw_dmt.h"
#include "ojw_motion.h"
#include <Servo.h>

Servo motor[4];  // create servo object to control a servo


COjwMotion ojw_motion;




void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  motor[0].attach(9);

  // ojw 모션 설정 
  //
  ojw_motion.setMotionMotorCallback(motionMotorFunc);
  ojw_motion.setMotionStartCallback(motionStartFunc);
  ojw_motion.addMotionFile(0, "go", motion);
}

void loop() { 
  // put your main code here, to run repeatedly:

  if (Serial.available() > 0)
  {
    Serial.read();
    ojw_motion.playMotion(0);
  }
}


void motionStartFunc(SMotionHeader_t *p_header)
{
  Serial.println("nFrameSize      \t " + String(p_header->nFrameSize));
  Serial.println("nCommentSize    \t " + String(p_header->nCommentSize));
  Serial.println("nCnt_LineComment\t " + String(p_header->nCnt_LineComment));
  Serial.println("nPlayTime       \t " + String(p_header->nPlayTime));
  Serial.println("nRobotModelNum  \t " + String(p_header->nRobotModelNum));
  Serial.println("nMotorCnt       \t " + String(p_header->nMotorCnt));
}

bool motionMotorFunc(SMotionHeader_t *p_header, SMotionTableDB_t *p_table)
{
  bool stop_motion = false;


  Serial.println("nTime  : " + String(p_table->nTime));
  Serial.println("nDelay : " + String(p_table->nDelay));
  
  for (int nAxis = 0; nAxis < p_header->nMotorCnt; nAxis++)
  {
    Serial.println("Mot[" + String(nAxis) + "] = " + String(p_table->pnMot[nAxis]));
  }
  Serial.println("---\n");

  int val;
  val = map(p_table->pnMot[1], 0, 1023, 0, 180);
  motor[0].write(val);

  return stop_motion;
}
