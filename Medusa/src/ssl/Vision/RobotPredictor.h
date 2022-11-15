/**
* @file RobotPredictor.h
* ���ļ�Ϊ������λ����ϢԤ����Ķ���.
* @date $Date: 2004/06/18 01:50:22 $
* @version $Revision: 1.12 $
* @author peter@mail.ustc.edu.cn
*/

#ifndef _ROBOT_PREDICTOR_H_
#define _ROBOT_PREDICTOR_H_

#include "WorldModel/WorldDefine.h"
#include <WorldModel/server.h>
#include <PlayerCommand.h>

namespace {
	const int MAX_INFORMATIONS = 16;
}

// ����
struct RobotCommandEffect {
	RobotCommandEffect() :cycle(0), rot_speed(0) {}
	int cycle; // ���ڵ����ڣ�������֤�Ƿ���Ч
	CVector vel; // ƽ���ٶ�
	double rot_speed; // ��ת�ٶ�
};

class CRobotCommandLogger {
public:
	bool commandValid(int cycle) const { return _command[(cycle + MAX_INFORMATIONS) % MAX_INFORMATIONS].cycle == cycle; }
	const RobotCommandEffect& getCommand(int cycle) const { return _command[(cycle + MAX_INFORMATIONS) % MAX_INFORMATIONS]; }
	void setCommand(int cycle, const CVector& vel, double rot_speed)
	{
		_command[(cycle + MAX_INFORMATIONS) % MAX_INFORMATIONS].cycle = cycle;

		if (std::isfinite(vel.mod()) && std::isfinite(rot_speed)) {
			_command[(cycle + MAX_INFORMATIONS) % MAX_INFORMATIONS].vel = vel;
			_command[(cycle + MAX_INFORMATIONS) % MAX_INFORMATIONS].rot_speed = rot_speed;
		}
		else {
			printf("%.2f , %.2f, %.2f command error!\n", vel.x(), vel.y(), rot_speed);
			_command[(cycle + MAX_INFORMATIONS) % MAX_INFORMATIONS].vel = CVector();
			_command[(cycle + MAX_INFORMATIONS) % MAX_INFORMATIONS].rot_speed = 0.0;
		}
	}
	void print(int cycle) {
		std::cout << " cycle: " << cycle << std::endl;
		for (int i = 0; i < MAX_INFORMATIONS; i++) {
			std::cout << "i:" << i << " RobotComandLogger: cycle = " << _command[(cycle - i + MAX_INFORMATIONS) % MAX_INFORMATIONS].cycle << " velx: " << _command[i].vel.x()
				<< " vely: " << _command[i].vel.y() << " rotvel: " << _command[i].rot_speed << std::endl;
		}
	}
private:
	RobotCommandEffect _command[MAX_INFORMATIONS];
};

struct RobotVisionData : public PlayerVisionT {
	int cycle;
	int realNum;
};

class CRobotVisionLogger {
public:
	CRobotVisionLogger() {}
	bool visionValid(int cycle) const { return _vision[(cycle + MAX_INFORMATIONS) % MAX_INFORMATIONS].cycle == cycle; }
	const RobotVisionData& getVision(int cycle) const { return _vision[(cycle + MAX_INFORMATIONS) % MAX_INFORMATIONS]; }
	RobotVisionData& getVision(int cycle) { return _vision[(cycle + MAX_INFORMATIONS) % MAX_INFORMATIONS]; }
	void setVision(int cycle, const RobotVisionData& vision, int realNum)
	{
		_vision[(cycle + MAX_INFORMATIONS) % MAX_INFORMATIONS] = vision;
		_vision[(cycle + MAX_INFORMATIONS) % MAX_INFORMATIONS].cycle = cycle;
	}
private:
	RobotVisionData _vision[MAX_INFORMATIONS];
};

/**
* CRobotPredictor.
* ͨ�������˵��������Ӿ�ȷ�������˵�λ����Ϣ
*/
class CRobotPredictor {
public:
	CRobotPredictor();
	CRobotPredictor(bool isHasRotaion);
	void updateCommand(int cycle, const CPlayerCommand* cmd); // ��������
	void updateVision(int cycle, const VehicleInfoT& player, const BallVisionT& ball, bool invert); // �����Ӿ���Ϣ

	CRobotCommandLogger& getCommandLogger() {
		return _commandLogger;
	}
	const RobotVisionData& getResult(int cycle) const {
		return _visionLogger.getVision(cycle);
	}
	void setIsHasRotation(bool isHasRot) {
		_isHasRotation = isHasRot;
	}
protected:
	bool checkValid(int cycle, const CGeoPoint& pos);
private:
	bool _isHasRotation; // С���Ƿ���нǶ�(����)��Ϣ
	CRobotCommandLogger _commandLogger; // �����¼
	CRobotVisionLogger _visionLogger; // Ԥ�������Ϣ��¼
	RobotCommandEffect cmd;
};
#endif // _ROBOT_PREDICTOR_H_
