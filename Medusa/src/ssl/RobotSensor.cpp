#include "RobotSensor.h"

#include "BestPlayer.h"
#include <WorldModel/WorldModel.h>
#include <fstream>
#include "Global.h"
#include "src_cmd.pb.h"
#include "SSLStrategy.h"

using namespace std;

extern bool IS_SIMULATION;

namespace {	
	//ofstream stopmsgfile("D:\\stopmsg.txt");
	bool debug = true;
}

CRobotSensor::CRobotSensor()
{
	// ���ݽ��г�ʼ��
	memset(_isValid, false, sizeof(_isValid));
	memset(_lastKickingChecked, false, sizeof(_lastKickingChecked));
	memset(_lastCheckedKickingCycle, 0, sizeof(_lastCheckedKickingCycle));	

	for(int i = 0; i < Param::Field::MAX_PLAYER - 10; i ++) {
		robotInfoBuffer[i].bInfraredInfo = false;
		robotInfoBuffer[i].nKickInfo = 0;

		_lastInfraredInfo[i] = false;
		_last_change_num[i] = -1;
		_last_real_num_index[i] = -1;
	}
}

void CRobotSensor::Update(int cycle)
{
	// ���治����ֱ�����Է���
	if (IS_SIMULATION) {
		return;
	}

	rbk::protocol::Robots_Status robots_status;
	GRBKHandle::Instance()->getSubscriberData(robots_status);
	if (robots_status.robots_status_size() > 0) {
		//std::cout<<robots_status.DebugString()<<std::endl;
	}
	//std::cout << "SIZE: "<<robots_status.robots_status_size() << std::endl;
	//TODO: make num compatible!!!!!!!!2020-11-05
	for (int i = 0; i < Param::Field::MAX_PLAYER - 10; i ++) {
		int realNum = i;
		if (robots_status.robots_status_size() > 0) {
			rbk::protocol::Robot_Status rs = robots_status.robots_status(realNum);
			if (rs.robot() != realNum) {
				LogError("Do not match real number: "<< rs.robot()<<" "<<realNum);
			} 
			else{
				rawDataBuffer.bInfraredInfo = rs.infrared();
				rawDataBuffer.nRobotNum = rs.robot();
				rawDataBuffer.nKickInfo = rs.chip_kick() || rs.flat_kick();
				//std::cout << "rawDataBuffer.nKickInfo: " << rawDataBuffer.nKickInfo << std::endl;
			}

			// ǿ�Ʊ���һ��ʱ�����߳�
			if (_lastKickingChecked[i] || robotInfoBuffer[i].nKickInfo > 0) {
				if (cycle - _lastCheckedKickingCycle[i] > 5) {
					_lastKickingChecked[i] = false;
					robotInfoBuffer[i].nKickInfo = 0;
				}
			}

			if (rs.change_num() != _last_change_num[i] || rs.robot() != _last_real_num_index[i]) {
				_last_change_num[i] = rs.change_num();
				_last_real_num_index[i] = rs.robot();
				//std::cout <<"This is num changed: "<< rs.infrared() << " " << rs.flat_kick() << " " << rs.chip_kick() << std::endl;
				_isValid[i] = true;

				UpdateBallDetected(i);

				// ����������İ�
				if (rawDataBuffer.nKickInfo) {
					robotInfoBuffer[i] = rawDataBuffer;
					_lastKickingChecked[i] = true;
					_lastCheckedKickingCycle[i] = cycle;
				}
			} 
			else {
				_lastKickingChecked[i] = false;
			}
		}
	}
}

bool CRobotSensor::IsInfraredOn(int num)				
{ 
	if(IS_SIMULATION){
		return BestPlayer::Instance()->isOurPlayerStrictControlBall(num);
	} 
	else{
		return robotInfoBuffer[num].bInfraredInfo; 
	}
}

void CRobotSensor::UpdateBallDetected(int num)
{
	// ���������Լ����������
	int realnum = num;
	if (realnum != rawDataBuffer.nRobotNum) {
		return ;
	}

	// ���ں����ź�ʱ�ı���ϴ� [8/7/2011 cliffyin]
	// ����
	bool currentInfraredInfo = rawDataBuffer.bInfraredInfo;
	// std::cout << realnum << ": This is infrared: " << currentInfraredInfo << ". \n";
	// std::cout << "last: "<< _lastInfraredInfo[num] << ", curr: " << currentInfraredInfo << "\n";
	if (_lastInfraredInfo[num] != currentInfraredInfo) {
		_lastInfraredInfo[num] = currentInfraredInfo;
		robotInfoBuffer[num].bInfraredInfo = currentInfraredInfo;
	}

	return ;
}