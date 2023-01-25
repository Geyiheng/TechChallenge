/**
* @file param.h
* ���ļ�����������õ��ĸ��ֲ���.
* @date 10/1/2002
* @version 1.0
* @author peter@mail.ustc.edu.cn
*/
#ifndef _PARAM_H_
#define _PARAM_H_
#include "os_param.h" // ��ϵͳ�йصĲ���
#include "singleton.h"
#include "ParamManagerNew.h"
/**
* �������ֿռ�Param.
* ������ֿռ䶨���˳������õ������еĲ���
*/		

namespace Param{
	const auto a = OParamManager::Instance()->setFileName("./");
	const auto b = VParamManager::Instance()->setFileName("./");
	//const int CAMERA = 8;
	//const int BALLNUM = 20;
	//const int BLUE = 0;
	//const int YELLOW = 1;
	//const int BALLMERGEDISTANCE = 0;
	//const int ROBOTMERGEDOSTANCE = 100;
	//const int TEAMS = 2;
	namespace Field{
		const int POS_SIDE_LEFT = 1;
		const int POS_SIDE_RIGHT = -1;
		const int MAX_PLAYER = 16;

		/* ���ȵ�λ��Ϊ����,ʱ�䵥λ��Ϊ��,������λΪ��,�Ƕȵ�λΪ���� */
		/* Ball */
		const double BALL_SIZE = VParamManager::Instance()->value("Physics/ballradius", 21.5).toDouble()*0.1*2; //5; // �뾶?ֱ��
		//const double BALL_DECAY = -0.8; // ��������ļ��ٶȺ��ٶȳ�����,��λΪ /s
		/* Player */
		const double MAX_PLAYER_SIZE = OParamManager::Instance()->value("Size/carDiameter", 180).toDouble() * 0.1; //18;
		/* Field */
		const QString field = OParamManager::Instance()->value("Alert", "field", "Division_A").toString();
		//const double GOAL_POST_AVOID_LENGTH = 2;         //��������������ı��ϳ���
		const double GOAL_POST_THICKNESS = OParamManager::Instance()->value(field + "/goal_thickness", 20).toDouble() * 0.1; //2;           //�������
		//const int MAX_BALL_SPEED = 630;
		const bool   IF_USE_ELLIPSE = OParamManager::Instance()->value("Division_B", "if_ellipse_penalty", false).toBool(); //false;             // whether use ellipse penalty

		const double PITCH_LENGTH = OParamManager::Instance()->value(field + "/field_length", 12000).toDouble() * 0.1; //1200; // ���س�
		const double PITCH_WIDTH = OParamManager::Instance()->value(field + "/field_width", 9000).toDouble() * 0.1; //900; // ���ؿ�
		const double PITCH_MARGIN = OParamManager::Instance()->value(field + "/field_line_width", 10).toDouble() * 0.1; //1; // ���صı߽���
		const double CENTER_CIRCLE_R = OParamManager::Instance()->value(field + "/center_radius", 500).toDouble() * 0.1 * 2; //100; // ��Ȧ�뾶?ֱ��
		const double PENALTY_AREA_WIDTH = OParamManager::Instance()->value(field + "/penalty_width", 2400).toDouble() * 0.1; //240; // �������
		const double PENALTY_AREA_DEPTH = OParamManager::Instance()->value(field + "/penalty_depth", 1200).toDouble() * 0.1; //120; // �������
		const double PENALTY_AREA_R = OParamManager::Instance()->value("Division_B/penalty_radius", 800).toDouble() * 0.1; //80; // ����Բ��
		const double PENALTY_AREA_L = OParamManager::Instance()->value("Division_B/penalty_area_l", 350).toDouble() * 0.1; //35; // ��������Բ�����߶�
		const double PENALTY_L = 50;
		const double PENALTY_MARK_X = OParamManager::Instance()->value(field + "/penalty_point", 1200).toDouble() * 0.1; //480; // ������X����
		const double FIELD_WALL_DIST = OParamManager::Instance()->value(field + "/field_margin", 300).toDouble() * 0.1;  //20; // ���ػ������߽�ľ���
		const double GOAL_WIDTH = OParamManager::Instance()->value(field + "/goal_width", 1200).toDouble() * 0.1; //120; // ���ſ��
		const double GOAL_DEPTH = OParamManager::Instance()->value(field + "/goal_depth", 200).toDouble() * 0.1; //20; // �������
		const double FREE_KICK_AVOID_BALL_DIST = OParamManager::Instance()->value(field + "/field_free_kick", 700).toDouble() * 0.1; //50; // ���������ʱ��,�Է�����������ôԶ
		const double RATIO = 1.5;
	}
	namespace Math{
		const double PI = VParamManager::Instance()->value("Physics/PI", 3.14159265358979323846).toDouble(); //3.14159265358979323846;
		//const double test = SingleParamManager::Instance()->test_double;
	}
	namespace Vehicle{
		namespace V2{
			const double PLAYER_SIZE = OParamManager::Instance()->value("Size/carDiameter", 180).toDouble() * 0.05; //9;
			const double PLAYER_FRONT_TO_CENTER = VParamManager::Instance()->value("Physics/botCenterToMouth", 76).toDouble() * 0.1; //7.6;
			const double KICK_ANGLE = ::Param::Math::PI*17/180; // ���Ի��������������Ƕ�
			//const double DRIBBLE_SIZE = PLAYER_FRONT_TO_CENTER + ::Param::Field::BALL_SIZE; // ����ʱ����ľ���
			const double DRIBBLE_ANGLE = ::Param::Math::PI * 17 / 180; // ���Դ��������������Ƕ�
			//const double HEAD_ANGLE = 57*Param::Math::PI / 180; // ǰ��Ŀ��ڽǶ�
			const double TOUCH_SHIFT_DIST = 9.96; //��Touchʱ���˵ľ���
		}
	}
	namespace AvoidDist{
		const double TEAMMATE_AVOID_DIST = Param::Vehicle::V2::PLAYER_SIZE * 3;;
		//const double OPP_AVOID_DIST = Param::Field::MAX_PLAYER_SIZE;
		//const double BALL_AVOID_DIST = Param::Field::BALL_SIZE / 2 + 2.0f;
	}
	namespace Vision{
		const double FRAME_RATE = OParamManager::Instance()->value("Alert/frameRate", 75).toDouble(); //75; // ÿ������
	}
	namespace Latency{ 
		const float TOTAL_LATED_FRAME = VParamManager::Instance()->value("Physics/total_lated_frame", 4.7).toFloat(); //4.7f; // �ӳٵ�������,��������
	}
	namespace Rule{
		const int Version = 2019; // ����İ汾
		const double MaxDribbleDist = 50; // ���������
	}
}
#endif
