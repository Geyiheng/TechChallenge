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
#include "ParamManagerNew.h"
/**
* �������ֿռ�Param.
* ������ֿռ䶨���˳������õ������еĲ���
*/		

namespace Param{
	//const int CAMERA = 8;
	//const int BALLNUM = 20;
	//const int BLUE = 0;
	//const int YELLOW = 1;
	//const int BALLMERGEDISTANCE = 0;
	//const int ROBOTMERGEDOSTANCE = 100;
	//const int TEAMS = 2;
	const QString team = SParamManager::Instance()->value("Team", "blueTeam", "SRC").toString();
	namespace Field{
		const int POS_SIDE_LEFT = 1;
		const int POS_SIDE_RIGHT = -1;
		const int MAX_PLAYER = 16;

		/* ���ȵ�λ��Ϊ����,ʱ�䵥λ��Ϊ��,������λΪ��,�Ƕȵ�λΪ���� */
		/* Ball */
		const double BALL_SIZE = SParamManager::Instance()->value("Ball/BallRadius", 0.0215).toDouble()*100*2; //5; // �뾶?ֱ��
		//const double BALL_DECAY = -0.8; // ��������ļ��ٶȺ��ٶȳ�����,��λΪ /s
		/* Player */
		const double MAX_PLAYER_SIZE = QSettings(QString("../data/config/") + QString("%1.ini").arg(team), QSettings::IniFormat).value("Geometery/Radius", 0.09).toDouble() * 200; //18;
		/* Field */
		const QString field = OParamManager::Instance()->value("Alert", "field", "Division_A").toString();
		//const double GOAL_POST_AVOID_LENGTH = 2;         //��������������ı��ϳ���
		const double GOAL_POST_THICKNESS = OParamManager::Instance()->value(field + "/goal_thickness", 20).toDouble() * 0.1; //2;           //�������
		const bool   IF_USE_ELLIPSE = OParamManager::Instance()->value("Division_B", "if_ellipse_penalty", false).toBool(); //false;             // whether use ellipse penalty

		const double PITCH_LENGTH = OParamManager::Instance()->value(field + "/field_length", 12000).toDouble() * 0.1; //1200; // ���س�
		const double PITCH_WIDTH = OParamManager::Instance()->value(field + "/field_width", 9000).toDouble() * 0.1; //900; // ���ؿ�
		const double PITCH_MARGIN = OParamManager::Instance()->value(field + "/field_line_width", 10).toDouble() * 0.1; //1; // ���صı߽���
		const double CENTER_CIRCLE_R = OParamManager::Instance()->value(field + "/center_radius", 500).toDouble() * 0.1 * 2; //100; // ��Ȧ�뾶?ֱ��
		const double PENALTY_AREA_WIDTH = OParamManager::Instance()->value(field + "/penalty_width", 2400).toDouble() * 0.1; //240; // �������
		const double PENALTY_AREA_DEPTH = OParamManager::Instance()->value(field + "/penalty_depth", 1200).toDouble() * 0.1; //120; // �������
		const double PENALTY_AREA_R = OParamManager::Instance()->value("Division_B/penalty_radius", 800).toDouble() * 0.1; //80; // ����Բ��
		const double PENALTY_AREA_L = OParamManager::Instance()->value("Division_B/penalty_area_l", 350).toDouble() * 0.1; //35; // ��������Բ�����߶�
		const double PENALTY_L = ParamManager::Instance()->value("Defence/penalty_l", 50).toDouble(); //50;
		const double PENALTY_MARK_X = OParamManager::Instance()->value(field + "/penalty_point", 1200).toDouble() * 0.1; //480; // ������X����
		const double FIELD_WALL_DIST = OParamManager::Instance()->value(field + "/field_margin", 300).toDouble() * 0.1;  //20; // ���ػ������߽�ľ���
		const double GOAL_WIDTH = OParamManager::Instance()->value(field + "/goal_width", 1200).toDouble() * 0.1; //120; // ���ſ��
		const double GOAL_DEPTH = OParamManager::Instance()->value(field + "/goal_depth", 200).toDouble() * 0.1; //20; // �������
		const double FREE_KICK_AVOID_BALL_DIST = ParamManager::Instance()->value("Rule/free_kick_avoid_ball_dist", 50).toDouble(); //50; // ���������ʱ��,�Է�����������ôԶ
		const double RATIO = 1.5;
	}
	namespace Math{
		const double PI = CParamManager::Instance()->value("Math/PI", 3.14159265358979323846).toDouble(); //3.14159265358979323846;
		//const double test = SingleParamManager::Instance()->test_double;
	}
	namespace Vehicle{
		namespace V2{
			const double PLAYER_SIZE = QSettings(QString("../data/config/") + QString("%1.ini").arg(team), QSettings::IniFormat).value("Geometery/Radius", 0.09).toDouble() * 100; //9;
			const double PLAYER_FRONT_TO_CENTER = QSettings(QString("../data/config/") + QString("%1.ini").arg(team), QSettings::IniFormat).value("Geometery/CenterFromKicker", 0.073).toDouble() * 100; //7.6;
			const double KICK_ANGLE = ::Param::Math::PI*17/180; // ���Ի��������������Ƕ�
			//const double DRIBBLE_SIZE = PLAYER_FRONT_TO_CENTER + ::Param::Field::BALL_SIZE; // ����ʱ����ľ���
			const double DRIBBLE_ANGLE = ::Param::Math::PI * 17 / 180; // ���Դ��������������Ƕ�
			//const double HEAD_ANGLE = 57*Param::Math::PI / 180; // ǰ��Ŀ��ڽǶ�
			const double TOUCH_SHIFT_DIST = ParamManager::Instance()->value("TOUCHKICK/TOUCH_SHIFT_DIST", 9.96).toDouble(); //9.96; //��Touchʱ���˵ľ���
			const bool WHEEL_SPEED_CALLBACK = SParamManager::Instance()->value("Communication", "wheelSpeedCallBack", false).toBool(); // �������Ƿ�ʵʱ��������
		}
	}
	namespace AvoidDist{
		const double TEAMMATE_AVOID_DIST = Param::Vehicle::V2::PLAYER_SIZE * 3;;
		//const double OPP_AVOID_DIST = Param::Field::MAX_PLAYER_SIZE;
		//const double BALL_AVOID_DIST = Param::Field::BALL_SIZE / 2 + 2.0f;
	}
	namespace Vision{
		const double FRAME_RATE = SParamManager::Instance()->value("World/DesiredFPS", 75).toDouble(); //75; // ÿ������
	}
	namespace Latency{ 
		const float TOTAL_LATED_FRAME = VParamManager::Instance()->value("Physics/total_lated_frame", 4.7).toFloat(); //4.7f; // �ӳٵ�������,��������
	}
	namespace Rule{
		const int Version = ParamManager::Instance()->value("Rule/Version", 2019).toInt(); //2019; // ����İ汾
		const double MaxDribbleDist = ParamManager::Instance()->value("Rule/MaxDribbleDist", 50).toDouble(); //50; // ���������
		//const int MAX_BALL_SPEED = ParamManager::Instance()->value("Rule/MAX_BALL_SPEED", 630).toInt(); //630;
	}
}
#endif
