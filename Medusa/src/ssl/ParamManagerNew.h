#include <tinyxml2/tinyxml2.h>
#include <param.h>
#include <tinyxml/ParamReader.h>
#include <inifile/inifile2.h>

#ifndef PARAMMANAGERNEW_H
#define PARAMMANAGERNEW_H

double queryParamByName(char* xmlPath, std::string paramName);

struct GetBallParam {
    double back_bias;
};

class CParamManagerNew {
public:
    CParamManagerNew();
    ~CParamManagerNew();

    /************************************************************************/
    /*                           ���Կ���                                    */
    /************************************************************************/
    // GotoPosition �еĵ��Կ���
    int DRAW_TARGET;
    int RECORD_NUM;
    int RECORD_COMMAND;
    int NOT_MOVE;

    
    // SmartGotoPosition �еĵ��Կ���
    int DRAW_RRT;
    int DRAW_TRAJ;
    int DRAW_OBS;
    int DRAW_BALLPLACE_AREA;
    int DRAW_PENALTY_DEBUG_MSG;

	// TouchKick �еĵ��Կ���
	int TOUCH_SHIFT_DIST;
	int TOUCH_Debug;
    
    // Defence �еĵ��Կ���
    bool DEFENCE_DEBUG_MODE;
    bool ATTACK_DEF_MODE;
    bool GOALIE_EVALUATE;
    int MARKING_MODE;

    // �������
    double PERIOD_MOVE_X;
    double PERIOD_MOVE_Y;
    double PERIOD_MOVE_ROT;
    double A_MAX_1;
    double V_LIMIT_1;
    double PERIOD_V_LIMIT_1;
    double V_LIMIT_2;
    double PERIOD_V_LIMIT_2;
    double A_MAX_2;
    double V_LIMIT_3;
    double PERIOD_V_LIMIT_3;
    double V_LIMIT_4;
    double PERIOD_V_LIMIT_4;
    double PlACEBALL_SPEED;
    double PlACEBALL_ROT_SPEED;
    double PlACEBALL_ACCELERATION;
    double PlACEBALL_DECELERATION;
    double PlACEBALL_ROT_ACCELERATION;
    double PlACEBALL_CLOSE_DISTANCE;
    int PlACEBALL_PLAYER_NUM;
    double D_MAX_FACTOR;
    double MAX_WHEEL_SPEED;

    // ���ز���
    double FIELD_WALL_DIST;
    double PENALTY_AREA_DEPTH;

    double SUPPORT_DIST;
    GetBallParam GET_BALL_PARAM;

	// Goalie2022���� by SYLG
	double SLOW_BALL_SPD;
	int KICKPOWER_GOALIE;
	double HAVE_BALL_DIST;
	double CLOSE_DIST;
	double CHALLENGE_BALL_DIST;
	double BLOCK_DIST;

    //break ����
    double BREAK_BACK_DRIBBLE_SPEED;
    double BREAK_SHOOT_ACCURACY;
    bool BREAK_DEBUG;
    double COEF_NEARSCORE;
    double COEF_BLOCKSCORE;
    double COEF_DISTSCORE;
    
    //advance���� byTYH  2022.10
    double KICK_DIST;  /*��������Χ Խ��Խ��������*/
    int WantToLessShoot ; /*��������Խ��Խ�������� ���Ϊ0 ���Ϊ5*/
    double RELIEF_DIST ;  /*GET�н���״���µ�RELIEF�жϾ���*/
    double OPP_HAS_BALL_DIST; /*�жϵз��Ƿ�����ľ��� ��Ҫ����*/
    double CanPassToWingDist; /*Advance�ܹ������߷���ٽ����*/
    double CanWingShootDist; /*�߷��ܹ����ŵ��ٽ����*/
    double SHOOT_PRECISION;	/*����������С���ȽǷ�ĸ��Խ��Խ��Խ��ȷ ���Ϊ7���17*/
    double AdGetBallBias;  /*Getball��ƫ����*/
    double BalltoMeVelTime;/*Advance�����������ȥ�ӵ��ٽ�ʱ��*/
    /*�������Ȳ���*/
    int KICKPOWER;
    int CHIPPOWER;
    int ADV_FPASSPOWER;
    int ADV_CPASSPOWER;
    int RELIEF_POWER;
    int  BACK_POWER;
    bool Advance_DEBUG_ENGINE ;

    /*ReceivePass add by tyh*/
    double PassBalltoMeVel;

private:
};

typedef NormalSingleton< CParamManagerNew > ParamManager;

#endif // PARAMMANAGERNEW_H
