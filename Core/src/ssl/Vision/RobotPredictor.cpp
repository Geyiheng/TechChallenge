#include "RobotPredictor.h"
#include "GDebugEngine.h"
#include <utils.h>
#include <string.h>
#include <cstring>

CRobotPredictor::CRobotPredictor() : _isHasRotation(true) {
}

CRobotPredictor::CRobotPredictor(bool isHasRotation) : _isHasRotation(isHasRotation) {

}
void CRobotPredictor::updateVision(int cycle, const VehicleInfoT& player, const BallVisionT& ball, bool invert) {
	// !!!!�������������:
	// ����Ƕ��ֳ�Ҳ��ʶ����,�ͻ����updateOurVision, ����Ϊ�ڸú�����ȡ����cmd(ֻ���ҷ�������cmd������), ����Ͳ���.
	// ������Ҫһ������ҷ�����Է����ķ�ʽ!!
	RobotVisionData& predictedVision = _visionLogger.getVision(cycle);
	const double x = invert ? -player.pos.x : player.pos.x;
	const double y = invert ? -player.pos.y : player.pos.y;
	const double vel_x = invert ? -player.vel.x() : player.vel.x();
	const double vel_y = invert ? -player.vel.y() : player.vel.y();
	const double acc_x = invert ? -player.acc.x() : player.acc.x();
	const double acc_y = invert ? -player.acc.y() : player.acc.y();
	//std::cout << "Id:" << realNum << "\tPos: " << x << "," << y << "\tVel:" << player.vel
	//         << "\tdir:" << player.dir << std::endl;
	double dir, dirVel;

	dir = invert ? Utils::Normalize(player.dir + Param::Math::PI) : Utils::Normalize(player.dir);

	if (fabs(player.dirvel) > 6) {
		dirVel = 0;
	}
	else {
		dirVel = invert ? Utils::Normalize(-player.dirvel) : Utils::Normalize(player.dirvel);
	}

	predictedVision.cycle = cycle;
	predictedVision.SetPos(x, y);
	predictedVision.SetDir(dir);
	predictedVision.SetValid(player.pos.valid);
	predictedVision.SetAcc(CVector(acc_x, acc_y));
	predictedVision.SetVel(CVector(vel_x, vel_y));
	predictedVision.SetRotVel(dirVel);
	//if (player.isYellow && player.ID == 5)
	//	std::cout << "realVel: " << CVector(vel_x, vel_y) << std::endl;
	
	if (_isHasRotation && !Param::Vehicle::V2::WHEEL_SPEED_CALLBACK) {//our robot
		if (_commandLogger.commandValid(cycle - 1)) {
			cmd = _commandLogger.getCommand(cycle - 1);
		}
		else {
			cmd.rot_speed = 0;
			cmd.vel = CVector(0, 0);
		}
		// ��ǰһ֡���������ǰ���ٶ�(����ͼ��ʵ�ʲ�õ��ٶ�!!! ʵ��������: ����ͼ���ٶȺ������������,Ч������)
		CVector updateVel = cmd.vel.rotate(_visionLogger.getVision(cycle - 1).Dir()) * 1;
		predictedVision.SetVel(updateVel);
		double timeDelay = 0.00; // s
		double fixedX = invert ? x - updateVel.x() * timeDelay : x + updateVel.x() * timeDelay;
		double fixedY = invert ? y - updateVel.y() * timeDelay : y + updateVel.y() * timeDelay;
		predictedVision.SetPos(fixedX, fixedY);
		GDebugEngine::Instance()->gui_debug_x(CGeoPoint(x, y), COLOR_WHITE);
		GDebugEngine::Instance()->gui_debug_x(CGeoPoint(fixedX, fixedY), COLOR_PURPLE);
		//CString cmd_show;
		//cmd_show.Format("cmd: %f %f", cmd.vel.x(), cmd.vel.y()); //updateVel.x(), updateVel.y());
		//GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(fixedX, fixedY), cmd_show);
		// modified by zhyaic 2014.3.26, �˴��������·�����ת�ٶ���Ϊ��ʱ�����˵���ת�ٶȣ������ǵ���������y�����нϴ����ʱ
		// ����ط�Ӧ����ע��!!!!!!
		predictedVision.SetRotVel(cmd.rot_speed);
	}
	
}

bool CRobotPredictor::checkValid(int cycle, const CGeoPoint & pos) {
	// ͨ������һ֡��λ�����Ƚ�,��ȷ����ǰ��Ϊ��С��λ��pos�Ƿ���Ч
	const double PLAYER_OUT_BUFFER = Param::Rule::Version == 2003 ? -20 : -50;
	if (!Utils::IsInField(pos, PLAYER_OUT_BUFFER)) {
		return false; // ��Ա�ڱ߽�����ܶ�
	}
	if (_visionLogger.visionValid(cycle - 1)) {
		const RobotVisionData& lastRobot = _visionLogger.getVision(cycle - 1);
		if (lastRobot.Valid()) {
			const double MAX_PLAYER_MOVE_DIST_PER_CYCLE = 10;
			const double MAX_PLAYER_ROTATE_ANGLE_PER_CYCLE = Param::Math::PI / 2;
			const CVector playerMoved = pos - lastRobot.Pos();
			if (playerMoved.mod() > MAX_PLAYER_MOVE_DIST_PER_CYCLE) {
				return false; // �ߵ�̫�࣬�����ܣ�һ���ǿ�����
			}
		}
	}
	return true;
}

void CRobotPredictor::updateCommand(int cycle, const CPlayerCommand * cmd) {
	_commandLogger.setCommand(cycle, cmd->getAffectedVel(), cmd->getAffectedRotateSpeed());
}
