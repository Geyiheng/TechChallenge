#ifndef _ROBOT_FILTERED_INFO_H_
#define _ROBOT_FILTERED_INFO_H_

#include <singleton.h>
#include <param.h>
#include "WorldModel/WorldDefine.h"
/*
* С�������֪��Ϣ���մ��� created by qxz
*/

/// Ԥ�����һЩ������
#define MAX_STORE_SIZE 5  // �������5֡����Ϣ
#define NO_KICK 0
#define IS_FLAT_KICK_ON 1
#define IS_CHIP_KICK_ON 2
#define LOWEST_ENERGY 0
#define LOW_ENERGY 1
#define NORMAL_ENERGY 2
#define HIGH_ENERGY 3
#define HIGHEST_ENERGY 4

/// �����˱����֪��Ϣ
class CRobotSensor{
public:
	// ���캯��
	CRobotSensor();
	// ��������
    ~CRobotSensor();

	// ȫ�ָ��½��
	void Update(int cycle);

	// С��˫��ͨѶ�Ƿ��Ѿ������������Ƿ������µ�
	bool IsInfoValid(int num)				{ return _isValid[num]; }

	// С���Ƿ���������źţ�һ���ʾ������ǰ��
	bool IsInfraredOn(int num);
	// ����С���ĺ����ź�
    void ResetInraredOn(int num)			{ robotInfoBuffer[num].bInfraredInfo = false; }

	// С���з�����ƽ�������Ļ���
	int IsKickerOn(int num)					{ return robotInfoBuffer[num].nKickInfo; }

private:
    void receiveRobotStatus();
	// ���µ��ڲ��ӿ�
	void UpdateBallDetected(int num);

	ROBOTINFO robotInfoBuffer[Param::Field::MAX_PLAYER];		// ��Ź��˺�Ľ��
    ROBOTINFO rawDataBuffer[Param::Field::MAX_PLAYER];

	bool _isValid[Param::Field::MAX_PLAYER];					// ��ǰ�����Ƿ������µ�

};

typedef Falcon::NormalSingleton< CRobotSensor > RobotSensor;			// ȫ�ַ��ʽӿ�
#endif
