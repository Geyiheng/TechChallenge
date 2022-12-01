#include "OpenSpeed.h"
#include "CommandFactory.h"
#include "WorldModel/DribbleStatus.h"
#include "Vision/VisionModule.h"
#include <WorldModel/robot_power.h>
#include <cmath>
#include <fstream>

#include <iostream>

COpenSpeed::COpenSpeed() {

}

CPlayerCommand* COpenSpeed::execute(const CVisionModule* pVision) {
	int num = task().executor;
	double speedX = task().player.speed_x; // x����ƽ���ٶ�
	double speedY = task().player.speed_y; // y����ƽ���ٶ�
	double speedR = task().player.rotate_speed; // ת���ٶ�
	double dribblePower = DribbleStatus::Instance()->getDribbleCommand(num);
	return CmdFactory::Instance()->newCommand(CPlayerSpeedV2(num, speedX, speedY, speedR, dribblePower));
}