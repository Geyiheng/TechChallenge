/**
* @file param.h
* ���ļ�����������õ��ĸ��ֲ���.
* @date 10/1/2002
* @version 1.0
* @author peter@mail.ustc.edu.cn
*/
#ifndef _PARAM_H_
#define _PARAM_H_
#include <os_param.h> // ��ϵͳ�йصĲ���
//#include "ParamManagerNew.h"
/**
* �������ֿռ�Param.
* ������ֿռ䶨���˳������õ������еĲ���
*/
namespace Param{
	const int CAMERA = 8;
	const int BALLNUM = 20;
	const int BLUE = 0;
	const int YELLOW = 1;
	const int BALLMERGEDISTANCE = 0;
	const int ROBOTMERGEDOSTANCE = 100;
	const int TEAMS = 2;
	namespace Field{
		const int POS_SIDE_LEFT = 1;
		const int POS_SIDE_RIGHT = -1;
		const int MAX_PLAYER = 16;

		/* ���ȵ�λ��Ϊ����,ʱ�䵥λ��Ϊ��,������λΪ��,�Ƕȵ�λΪ���� */
		/* Ball */
		const double BALL_SIZE	= 5; // �뾶
		const double BALL_DECAY = -0.8; // ��������ļ��ٶȺ��ٶȳ�����,��λΪ /s
		/* Player */
		const double MAX_PLAYER_SIZE = 18;
		/* Field */
		const double GOAL_POST_AVOID_LENGTH = 2;         //��������������ı��ϳ���
		const double GOAL_POST_THICKNESS = 2;           //�������
		const int MAX_BALL_SPEED = 630;
		const bool   IF_USE_ELLIPSE = false;             // whether use ellipse penalty

		const double PITCH_LENGTH = 1200; // ���س�
		const double PITCH_WIDTH = 900; // ���ؿ�
		const double PITCH_MARGIN = 1; // ���صı߽���
		const double CENTER_CIRCLE_R = 100; // ��Ȧ�뾶
		const double PENALTY_AREA_WIDTH = 240; // �������
		const double PENALTY_AREA_DEPTH = 120; // �������
		const double PENALTY_AREA_R = 80; // ����Բ��
		const double PENALTY_AREA_L = 35; // ��������Բ�����߶�
		const double PENALTY_L = 50;
		const double PENALTY_MARK_X = 480; // ������X����
		const double FIELD_WALL_DIST = 20; // ���ػ������߽�ľ���
		const double GOAL_WIDTH = 120; // ���ſ��
		const double GOAL_DEPTH = 20; // �������
		const double FREE_KICK_AVOID_BALL_DIST = 50; // ���������ʱ��,�Է�����������ôԶ
		const double RATIO = 1.5;
	}
	namespace Math{
		const double PI = 3.14159265358979323846;
		//const double test = SingleParamManager::Instance()->test_double;
	}
	namespace Vehicle{
		namespace V2{
			const double PLAYER_SIZE = 9;
			const double PLAYER_FRONT_TO_CENTER = 7.6;
			const double KICK_ANGLE = ::Param::Math::PI*17/180; // ���Ի��������������Ƕ�
			const double DRIBBLE_SIZE = PLAYER_FRONT_TO_CENTER + ::Param::Field::BALL_SIZE; // ����ʱ����ľ���
			const double DRIBBLE_ANGLE = ::Param::Math::PI * 17 / 180; // ���Դ��������������Ƕ�
			const double HEAD_ANGLE = 57*Param::Math::PI / 180; // ǰ��Ŀ��ڽǶ�
			const double TOUCH_SHIFT_DIST = 9.96; //��Touchʱ���˵ľ���
		}
	}
	namespace AvoidDist{
		const double TEAMMATE_AVOID_DIST = Param::Vehicle::V2::PLAYER_SIZE * 3;;
		const double OPP_AVOID_DIST = Param::Field::MAX_PLAYER_SIZE;
		const double BALL_AVOID_DIST = Param::Field::BALL_SIZE / 2 + 2.0f;
	}
	namespace Vision{
		const double FRAME_RATE = 75; // ÿ������
	}
	namespace Latency{ 
        const float TOTAL_LATED_FRAME = 4.7f; // �ӳٵ�������,��������
	}
	namespace Rule{
		const int Version = 2019; // ����İ汾
		const double MaxDribbleDist = 50; // ���������
	}
}
#endif
