#ifndef  _ADVANCE_
#define _ADVANCE_

#include <skill/PlayerTask.h>
#include "DefendUtils.h"
/**********************************************************
* High Level Skill: Advance Ball Up Field / Down Field
* Description: ͨ��������-->����-->���ţ����н������߽�������
*			   �շ�����
* Author: ̷���
* Created Date: 2022/10/10
***********************************************************/

struct PassDirOrPos {
	double dir;
	CGeoPoint pos;
};

class CAdvance : public  CStatedTask{

public:

	CAdvance();
	~CAdvance();
	CAdvance(const CGeoPoint& ballTarget);
	virtual void plan(const CVisionModule* pVision);
	virtual CPlayerCommand* execute(const CVisionModule * pVision);
	virtual bool isEmpty()const{return false;}

private:
	enum {
		BEGIN = 0,
		GET,
		KICK,
		PASS,
		JUSTCHIPPASS,
		BREAKSHOOT,
		BREAKPASS
	};
	int _lastCycle;
	int _state;
	int _cycle;
	int infraredOn;
	int meHasBall;
	int meLoseBall;

	int opponentID;
	int NumberOfSupport;/* Gpu���Ľ��������� */
	int NowIsShoot; 
	/**********************************************************
	* Description: �����б� �Ѿ�����ini�ľ����Բ�������
	*              �������Ե�����
	* Author: ̷���
	* Created Date: 2022/10/10
	***********************************************************/
	//advance���� byTYH  2022.10
	double KICK_DIST ;  /*��������Χ Խ��Խ��������*/
	int WantToLessShoot ; /*��������Խ��Խ�������� ���Ϊ0 ���Ϊ5*/
	double RELIEF_DIST ;  /*GET�н���״���µ�RELIEF�жϾ���*/
	double OPP_HAS_BALL_DIST ; /*�жϵз��Ƿ�����ľ��� ��Ҫ����*/
	double CanPassToWingDist ; /*Advance�ܹ������߷���ٽ����*/
	double CanWingShootDist ; /*�߷��ܹ����ŵ��ٽ����*/
	double SHOOT_PRECISION ; /*����������С���ȽǷ�ĸ��Խ��Խ��Խ��ȷ */
	double GetBallBias; /*Getballƫ���� ��Ҫ����ȥ�� */
	double BalltoMeVelTime;/*Advance�����������ȥ�ӵ��ٽ�Time*/
	/*�������Ȳ���*/
	int KICKPOWER ;
	int CHIPPOWER ;
	int ADV_FPASSPOWER ;
	int ADV_CPASSPOWER ;
	int RELIEF_POWER ;
	int  BACK_POWER ;
	int Advance_DEBUG_ENGINE;
	/**********************************************************
	* Description: ͨ�ò��� 
	* Author: ̷���
	* Created Date: 2022/10/10
	***********************************************************/
	CGeoPoint theirLeft = CGeoPoint(Param::Field::PITCH_LENGTH / 2, -Param::Field::GOAL_WIDTH / 2);
	CGeoPoint theirRight = CGeoPoint(Param::Field::PITCH_LENGTH / 2, Param::Field::GOAL_WIDTH / 2);
	CGeoPoint theirCenter = CGeoPoint(Param::Field::PITCH_LENGTH / 2, 0);
	CGeoPoint ourGoal = CGeoPoint(-Param::Field::PITCH_LENGTH / 2, 0);

	CGeoPoint SupportPoint[9];

	int LastPassPoint = 0; /*�־û�������*/
	double last_dir_deviation = 100;
	double last_target_dir = 0; /*isDirOk����Ƕ�*/
	double KickorPassDir = 0;/*���������ŵķ��� Ӧ����һ��������ʾ ���пɳ�����������*/
	bool IsMeSupport = 0;

	/**********************************************************
	* Description: ����ຯ���������Ӿ���λ���ж�
	* Author: ̷���
	* Created Date: 2022/10/10
	***********************************************************/
	bool isVisionHasBall(const CVisionModule* pVision, const int vecNumber);
	bool checkOppHasBall(const CVisionModule* pVision);
	int getTheirMostClosetoPosPlayerNum(const CVisionModule* pVision, CGeoPoint pos);
	bool checkBallFront(const CVisionModule* pVision, double angle);
	bool IsOurNearHere(const CVisionModule* pVision, CGeoPoint checkPoint, const int vecNumber);

	bool Me2OppTooclose(const CVisionModule* pVision, const int vecNumber);
	bool isPassBalltoMe(const CVisionModule* pVision, int vecNumber);
	bool isDirOK(const CVisionModule* pVision, int vecNumber, double targetDir, int ShootOrPass);
	bool isInBreakArea(const CVisionModule* pVision, int vecNumber);
	bool JudgeIsMeSupport(const CVisionModule* pVision, int vecNumber);
	/**********************************************************
	* Description: ״̬�л��ж��ຯ��������״̬ת��֮����ж�
	* Author: ̷���
	* Created Date: 2022/10/10
***********************************************************/
	int toChipOrToFlat(const CVisionModule* pVision, int vecNumber);
	bool tendToShoot(const CVisionModule* pVision, int vecNumber);
	int CanSupportKick(const CVisionModule* pVision, int vecNumber);
	bool isTheLineBlocked(const CVisionModule* pVision, CGeoPoint startPoint, CGeoPoint targetPoint);

	/**********************************************************
	* Description: �����ຯ����������GET��ʹ��
	* Author: ̷���
	* Created Date: 2022/10/10
***********************************************************/
	bool isOppFaceOurDoor(const CVisionModule* pVision, double angle);
	bool checkTheyCanShoot(const CVisionModule* pVision, int vecNumber);
/**********************************************************
	* Description: �����ຯ�������о���ʵ��
	* Author: ̷���
	* Created Date: 2022/10/10
***********************************************************/

	PassDirOrPos PassDirInside(const CVisionModule* pVision, int vecNumber);
	double PassDir(const CVisionModule* pVision, int vecNumber);
	CGeoPoint GenerateBreakShootPoint(const CVisionModule* pVision, int vecNumber);
	CGeoPoint GenerateBreakPassPoint(const CVisionModule* pVision, int vecNumber);
	double TheMinDistBetweenTheOppAndTheLine(const CVisionModule* pVision, CGeoPoint startPoint, CGeoPoint targetPoint);

protected:

	CPlayerCommand* _directCommand;
};
#endif