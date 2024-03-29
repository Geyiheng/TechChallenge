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
	double speedX = task().player.speed_x; // x方向平动速度
	double speedY = task().player.speed_y; // y方向平动速度
	double speedR = task().player.rotate_speed; // 转动速度
	double dribblePower = 0;// DribbleStatus::Instance()->getDribbleCommand(num);
	return CmdFactory::Instance()->newCommand(CPlayerSpeedV2(num, speedX, speedY, speedR, dribblePower));
}
