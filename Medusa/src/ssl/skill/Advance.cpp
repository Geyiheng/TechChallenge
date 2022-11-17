/*
	22.10.10  written by tyh
	ADVANCE NEW
*/
#include "Advance.h"
#include <Vision/VisionModule.h>
#include <WorldModel/KickStatus.h>
#include "RobotSensor.h"
#include "skill/Factory.h"
#include <WorldModel/WorldModel.h>
#include "WorldModel/DribbleStatus.h"
#include "PointCalculation/IndirectDefender.h"
#include <utils.h>
#include <BestPlayer.h>
#include "KickDirection.h"
#include <GDebugEngine.h>
#include <iostream>
#include <BestPlayer.h>
#include <TaskMediator.h>
#include "Global.h"
#include <cstring>

CAdvance::CAdvance()
{
	NowIsShoot = 0;
	KICK_DIST = paramManager->KICK_DIST;  /*��������Χ Խ��Խ��������*/
	WantToLessShoot = paramManager->WantToLessShoot; /*��������Խ��Խ�������� ���Ϊ0 ���Ϊ5*/
	RELIEF_DIST = paramManager->RELIEF_DIST;  /*GET�н���״���µ�RELIEF�жϾ���*/
	OPP_HAS_BALL_DIST = paramManager->OPP_HAS_BALL_DIST; /*�жϵз��Ƿ�����ľ��� ��Ҫ����*/
	CanPassToWingDist = paramManager->CanPassToWingDist; /*Advance�ܹ������߷���ٽ����*/
	CanWingShootDist = paramManager->CanWingShootDist; /*�߷��ܹ����ŵ��ٽ����*/
	SHOOT_PRECISION = paramManager->SHOOT_PRECISION;	/*����������С���ȽǷ�ĸ��Խ��Խ��Խ��ȷ ���Ϊ7���17*/
	GetBallBias = paramManager->AdGetBallBias;	/*AdvanceGetball��ƫ��*/
	BalltoMeVelTime = paramManager->BalltoMeVelTime; /*Advance�����������ȥ�ӵ��ٽ�ʱ��*/
	/*�������Ȳ���*/
	KICKPOWER = paramManager->KICKPOWER;
	CHIPPOWER = paramManager->CHIPPOWER; // ��ʱ������
    ADV_FPASSPOWER_Alpha = paramManager->ADV_FPASSPOWER;
    ADV_CPASSPOWER_Alpha = paramManager->ADV_CPASSPOWER;
    // max:600 350
	RELIEF_POWER = paramManager->RELIEF_POWER;
    BACK_POWER = paramManager->BACK_POWER;
	Advance_DEBUG_ENGINE = paramManager->Advance_DEBUG_ENGINE;
}


CAdvance::~CAdvance() {

}

void CAdvance::plan(const CVisionModule* pVision)
{
	if (pVision->Cycle() - _cycle > Param::Vision::FRAME_RATE * 0.1) {
		_state = BEGIN;
	}
	/**********************************************************
	* Description: ��ʼ����Ҫ���������б�
	* Author: ̷���
	* Created Date: 2022/10/10
	***********************************************************/
	const int maxFrared = 100 * 1.25;
	const int maxMeHasBall = int(50 * 1.25);
	int _executor = task().executor;
	int tandemNum = task().ball.receiver;
	int DoNotEnterDefenseBox = PlayerStatus::DODGE_OUR_DEFENSE_BOX;
	int AllowDribbleFlag = PlayerStatus::DRIBBLING;
	int ShootAllowDribble = DoNotEnterDefenseBox | AllowDribbleFlag;
	int ShootNotNeedDribble = DoNotEnterDefenseBox & (~AllowDribbleFlag);
	bool frared = RobotSensor::Instance()->IsInfraredOn(_executor);
	if (frared) { infraredOn = infraredOn >= maxFrared ? maxFrared : infraredOn + 1; }
	else { infraredOn = 0; }

	/**********************************************************
	* Description: ��ʼ����������б�
	* Author: ̷���
	* Created Date: 2022/10/10
	***********************************************************/
	const PlayerVisionT& me = pVision->OurPlayer(_executor);
	const BallVisionT& ball = pVision->Ball();
	int GoalieNumber = 0;
	int NumofPlayerInFrontfiled = 0;
	bool isMeHasBall = false;
    bool isMechHasBall = infraredOn >= 1;
	bool visionHasBall = isVisionHasBall(pVision, _executor);
    isMeHasBall = isMechHasBall; //visionHasBall; //isMechHasBall&
	/* �˴�����ֻ�����Ӿ��ж� �����ж���Ҫʵ�����е��� */

	if (isMeHasBall) {
		meHasBall = meHasBall >= maxMeHasBall ? maxMeHasBall : meHasBall + 1;
		meLoseBall = 0;
	}
	else {
		meHasBall = 0;
		meLoseBall = meLoseBall >= maxMeHasBall ? maxMeHasBall : meLoseBall + 1;
	}
	double BallToOurGoal = (ball.Pos() - ourGoal).mod();
	CVector me2goal = theirCenter - me.Pos();
	bool isOppHasBall = checkOppHasBall(pVision);
	const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
	const CVector opp2ball = ball.Pos() - opp.Pos();
	double ball2oppDist = opp2ball.mod();
	double ball2meDist = (ball.Pos() - me.Pos()).mod();
	double Me2Receiver = (me.Pos() - pVision->OurPlayer(tandemNum).Pos()).mod();
	double me2BestOppDist = CVector(pVision->TheirPlayer(opponentID).Pos() - me.Pos()).mod();
	
    if (KickorPassDir < 1e-3) {
            if (me2goal.mod() < KICK_DIST && (Me2OppTooclose(pVision, _executor) || isInBreakArea(pVision, _executor))) {
                KickorPassDir = KickDirection::Instance()->getPointShootDir(pVision, ball.Pos());
            }
            else KickorPassDir = PassDir(pVision, _executor);
        }



    PassDirOrPos TMP;
    CGeoPoint PassPos;
    /*??��??*/

	CGeoPoint ShootPoint, PassPoint;/*���������ŵķ��� Ӧ����һ��������ʾ ���пɳ�����������*/

	for(int i=0;i<9;++i)
		SupportPoint[i] = GPUBestAlgThread::Instance()->getBestPointFromArea(i);/* Gpu��� */
//	NormalPlayUtils::generatePassPoint(ball.Pos(), SupportPoint[0], SupportPoint[1], SupportPoint[2], SupportPoint[3]);
	NumberOfSupport = 6;/*��ʱֻ���Ƕ���볡����*/
	IsMeSupport = JudgeIsMeSupport(pVision, _executor);/*�ж����ǲ���support ���ڴ���*/

	/**********************************************************
	* Description: ��Ļ�Ҳ� ����has��lose��ͼ��
	* Author: ̷���
	* Created Date: 2022/10/10
	***********************************************************/
	if (Advance_DEBUG_ENGINE) {
		GDebugEngine::Instance()->gui_debug_x(CGeoPoint(-545, -int(300 * meHasBall / maxMeHasBall)), COLOR_YELLOW);
		GDebugEngine::Instance()->gui_debug_x(CGeoPoint(-575, -int(300 * meLoseBall / maxMeHasBall)), COLOR_ORANGE);
		GDebugEngine::Instance()->gui_debug_x(CGeoPoint(-545, -int(300 * meHasBall / maxMeHasBall)), COLOR_YELLOW);
		GDebugEngine::Instance()->gui_debug_x(CGeoPoint(-575, -int(300 * meLoseBall / maxMeHasBall)), COLOR_ORANGE);
		GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(-545, -300), "HAS", COLOR_BLACK);
		GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(-575, -300), "LOSE", COLOR_BLACK);
	}
	for (int i = 0; i < Param::Field::MAX_PLAYER; i++) {
		if (pVision->OurPlayer(i).Valid() && i != GoalieNumber)
			if (pVision->OurPlayer(i).Pos().x() > Param::Field::PITCH_LENGTH / 10)
				NumofPlayerInFrontfiled++;
	}

	/**********************************************************
	* Description: ״̬����
	* Author: ̷���
	* Created Date: 2022/10/10
	***********************************************************/
	switch (_state) {
	case BEGIN:
		_state = GET;
		break;
	case GET:
		if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -400), "Push GET", COLOR_YELLOW);
		if (meHasBall>3) {
			KickStatus::Instance()->resetAdvancerPassTo();
			/*����Һ�����֮��ľ���С��KICK_DIST������˳��Ϊ shoot->break->pass */
			if (NowIsShoot == 1) { _state = KICK; break; }
			if (NowIsShoot == 2) { _state = BREAKSHOOT; break; }
			if (me2goal.mod() < KICK_DIST) {
				if (tendToShoot(pVision, _executor)) {
					NowIsShoot = 1;
					_state = KICK; break;
				}
				else if(Me2OppTooclose(pVision, _executor) || isInBreakArea(pVision, _executor)) {
					NowIsShoot = 2;
					_state = BREAKSHOOT; break;
				}
				else {
					_state = PASS; break;
				}
			}
			else if (me.X() > 0) { 
				/*����ǰ��*/
				if (CanSupportKick(pVision, _executor)){
					_state = PASS; break;

				}
				else if(Me2OppTooclose(pVision, _executor)) {
					_state = BREAKPASS; break;
				}
				else { _state = JUSTCHIPPASS; break; }
			}
			else {
				/*���ں�  �˴���Ϊһ����ܵ�TODO*/
				if (CanSupportKick(pVision, _executor)) {
					_state = PASS; break;

				}
				else if (Me2OppTooclose(pVision, _executor)) {
					_state = BREAKPASS; break;
				}
				else { _state = JUSTCHIPPASS; break; }
			}
		}
		else { _state = GET; break; }
	case KICK:
		if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -400), "Push KICK", COLOR_YELLOW);
		if (meLoseBall > 10 && ball2meDist > 10) _state = GET;
		break;
	case PASS:
		if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -400), "Push PASS", COLOR_YELLOW);
		if (meLoseBall > 10 && ball2meDist > 10) _state = GET;
		break;
	case JUSTCHIPPASS:
		if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -400), "Push CHIP", COLOR_YELLOW);
		if (meLoseBall > 10 && ball2meDist > 10) _state = GET;
		break;
	case BREAKSHOOT:
		if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -400), "Push CHIP", COLOR_YELLOW);
		if (meLoseBall > 10 && ball2meDist > 10) _state = GET;
		break;
	case BREAKPASS:
		if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -400), "Push CHIP", COLOR_YELLOW);
		if (meLoseBall > 10 && ball2meDist > 10) _state = GET;
		break;
	}
	
	/**********************************************************
	* Description: ״ִ̬��
	* Author: ̷���
	* Created Date: 2022/10/10
	***********************************************************/
	switch (_state) {
	case GET:
		if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(200, -400), "let GET", COLOR_YELLOW);
		if (ball2meDist > 50 || (isPassBalltoMe(pVision, _executor)))NowIsShoot = 0;
		/*���shoot���*/

		if (BallToOurGoal < RELIEF_DIST && ball2oppDist < 30) {
			/*��ҪRELIEF�Ľ������*/
			if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -400), "RELIEF", COLOR_ORANGE);
			KickorPassDir = KickDirection::Instance()->getPointShootDir(pVision, pVision->OurPlayer(_executor).Pos());
			KickStatus::Instance()->setChipKick(_executor, RELIEF_POWER);
			setSubTask(PlayerRole::makeItChaseKickV2(_executor, KickorPassDir, ShootNotNeedDribble));
		}
		else if (!ball.Valid()) {
			/*�򲻺Ϸ������*/
			if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -400), "Ball invalid", COLOR_ORANGE);
			double faceDir = opp.Dir() + Param::Math::PI;
			setSubTask(PlayerRole::makeItChaseKickV2(_executor, faceDir, ShootNotNeedDribble));
		}
		else if (checkTheyCanShoot(pVision, _executor)) {
			/*���������Ż��� ���з��*/
			if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -400), "they can shoot", COLOR_ORANGE);
			double faceDir = opp.Dir() + Param::Math::PI;
			setSubTask(PlayerRole::makeItChaseKickV2(_executor, faceDir, ShootNotNeedDribble));
		}
		else if (isPassBalltoMe(pVision, _executor)) {
			/*�ҷ����ҽ��д���*/
            if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -350), "pass ball to me", COLOR_ORANGE);

            if (me2goal.mod() < KICK_DIST && (Me2OppTooclose(pVision, _executor) || isInBreakArea(pVision, _executor))) {
                KickorPassDir = KickDirection::Instance()->getPointShootDir(pVision, pVision->OurPlayer(_executor).Pos());
            }
            else KickorPassDir = PassDir(pVision, _executor);
			setSubTask(PlayerRole::makeItReceivePass(_executor, KickorPassDir));
		}
		else {
			/*��û�еõ��� ��Ҫȥgetball*/
			if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -300), "LOSE and GETBALL", COLOR_ORANGE);
			//KickorPassDir = KickDirection::Instance()->getPointShootDir(pVision, pVision->OurPlayer(_executor).Pos());
			/*�˴�����ɳ־û����� ����Ҫ���иı�*/
			setSubTask(PlayerRole::makeItNoneTrajGetBall(_executor, KickorPassDir, CVector(0, 0), ShootNotNeedDribble, GetBallBias));
		}
		break;
	case KICK:   // ����
		if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(200, -400), "KICK", COLOR_YELLOW);
		KickorPassDir = KickDirection::Instance()->getPointShootDir(pVision, pVision->OurPlayer(_executor).Pos());
		KickStatus::Instance()->clearAll();
		if (Utils::InTheirPenaltyArea(ball.Pos(), 0)) {
			/*������ڶԷ�����*/
			if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -400), "ball in their PEN", COLOR_ORANGE);
			KickStatus::Instance()->setKick(_executor, KICKPOWER);
			setSubTask(PlayerRole::makeItShootBallV2(_executor, KickorPassDir, ShootNotNeedDribble));
		}
		else {
			/*����KICK�׶�  ��Ҫ�����Ƿ����Ѿ�ת��ɹ�  �˴���δ�걸���ܴ���BUG*/
			if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -400), "Let Kick", COLOR_ORANGE);
			if (isDirOK(pVision, _executor, KickorPassDir, 1)) {
				if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -350), "Kick isDirOK", COLOR_ORANGE);
				KickStatus::Instance()->setKick(_executor, KICKPOWER);
				setSubTask(PlayerRole::makeItShootBallV2(_executor, KickorPassDir));
			}
			else {
				//setSubTask(PlayerRole::makeItGoAndTurnKickV4(_executor, kickDir));
				setSubTask(PlayerRole::makeItNoneTrajGetBall(_executor, KickorPassDir, CVector(0, 0), ShootNotNeedDribble, GetBallBias));
				if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -350), "Kick is NOT DirOK ", COLOR_ORANGE);
			}
		}
		break;
	case PASS:
		if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(200, -400), "PASS", COLOR_YELLOW);
		KickStatus::Instance()->clearAll();
		if (CanSupportKick(pVision, _executor) == 1 || toChipOrToFlat(pVision, _executor) == 1) {
			/*����Ϊ����ƽ���� ��ƽ����*/
			if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -400), "FLATPASS", COLOR_ORANGE);
            TMP = PassDirInside(pVision, _executor);
            KickorPassDir = TMP.dir;
            PassPos = TMP.pos;
			if (isDirOK(pVision, _executor, KickorPassDir, 0)) {
                KickStatus::Instance()->setKick(_executor, GetFPassPower(me.Pos(), PassPos));
                KickStatus::Instance()->setAdvancerPassTo(PassPos);
				setSubTask(PlayerRole::makeItShootBallV2(_executor, KickorPassDir));
				if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -350), "FLAT isDirOK", COLOR_ORANGE);
			}
			else {
				//setSubTask(PlayerRole::makeItGoAndTurnKickV4(_executor, passDir));
				setSubTask(PlayerRole::makeItNoneTrajGetBall(_executor, KickorPassDir, CVector(0, 0), ShootNotNeedDribble, GetBallBias));
				if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -350), "FLAT is NOT DirOK", COLOR_ORANGE);
			}
		}
		else {
			/*����Ϊ����ƽ���� ��ѡ������ ���Ǻ���ͬ��ΪflatPassDir*/
			if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -400), "CHIPPASS", COLOR_CYAN);
            TMP = PassDirInside(pVision, _executor);
            KickorPassDir = TMP.dir;
            PassPos = TMP.pos;
			if (isDirOK(pVision, _executor, KickorPassDir, 0)) {
                KickStatus::Instance()->setChipKick(_executor, GetCPassPower(me.Pos(), PassPos));
                KickStatus::Instance()->setAdvancerPassTo(PassPos);
				setSubTask(PlayerRole::makeItShootBallV2(_executor, KickorPassDir));
				if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -400), "CHIP isDirOK", COLOR_ORANGE);
			}
			else {
				//setSubTask(PlayerRole::makeItGoAndTurnKickV4(_executor, passDir));
				setSubTask(PlayerRole::makeItNoneTrajGetBall(_executor, KickorPassDir, CVector(0, 0), ShootNotNeedDribble, GetBallBias));
				if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -400), "CHIP is NOT DirOK", COLOR_ORANGE);
			}
		}
		break;

	case JUSTCHIPPASS:
		if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(200, -400), "CHIP", COLOR_YELLOW);
		KickStatus::Instance()->clearAll();
        TMP = PassDirInside(pVision, _executor);
        KickorPassDir = TMP.dir;
        PassPos = TMP.pos;
		if (isDirOK(pVision, _executor, KickorPassDir, 0)) {
            KickStatus::Instance()->setChipKick(_executor, GetCPassPower(me.Pos(), PassPos));
            KickStatus::Instance()->setAdvancerPassTo(PassPos);
			setSubTask(PlayerRole::makeItShootBallV2(_executor, KickorPassDir));
			if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -400), "CHIP isDirOK", COLOR_ORANGE);
		}
		else {
			//setSubTask(PlayerRole::makeItGoAndTurnKickV4(_executor, passDir));
			setSubTask(PlayerRole::makeItNoneTrajGetBall(_executor, KickorPassDir, CVector(0, 0), ShootNotNeedDribble, GetBallBias));
			if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -400), "CHIP is NOT DirOK", COLOR_ORANGE);
		}
		break;

	case BREAKSHOOT:
		if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(200, -400), "BREAKSHOOT", COLOR_YELLOW);
		KickStatus::Instance()->clearAll();
		ShootPoint = GenerateBreakShootPoint(pVision, _executor);
		setSubTask(PlayerRole::makeItBreak(_executor, ShootPoint));
		break;

	case BREAKPASS:
		if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(200, -400), "BREAKPASS", COLOR_YELLOW);
		KickStatus::Instance()->clearAll();
		PassPoint = GenerateBreakPassPoint(pVision, _executor);
		setSubTask(PlayerRole::makeItBreak(_executor, PassPoint));
		break;
	}
	_cycle = pVision->Cycle();
	CStatedTask::plan(pVision);
}

/**********************************************************
	* Description: ����ຯ���������Ӿ���λ�õ��ж�
	* Author: ̷���
	* Created Date: 2022/10/10
***********************************************************/
bool CAdvance::isVisionHasBall(const CVisionModule* pVision, const int vecNumber) {
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	const BallVisionT& ball = pVision->Ball();
    double visionJudgDist = 11.3;
	bool distVisionHasBall = CVector(me.Pos() - ball.Pos()).mod() <= visionJudgDist;
	bool dirVisionHasBall;
	double meDir = me.Dir();
	double me2Ball = (ball.Pos() - me.Pos()).dir();
	double meDir_me2Ball_Diff = abs(Utils::Normalize((meDir - me2Ball)));
	if (meDir_me2Ball_Diff < Param::Math::PI / 6.0) 
		dirVisionHasBall = true;
	else 
		dirVisionHasBall = false;
	bool isVisionPossession = dirVisionHasBall && distVisionHasBall;
	return isVisionPossession;
}

bool CAdvance::checkOppHasBall(const CVisionModule* pVision) {
	int _executor = task().executor;
	const BallVisionT& ball = pVision->Ball();
	const PlayerVisionT& me = pVision->OurPlayer(_executor);
	const CVector self2ball = ball.Pos() - me.Pos();
	opponentID = 0;

	const CBestPlayer::PlayerList& oppList = BestPlayer::Instance()->theirFastestPlayerToBallList();
	if (oppList.size() < 1)return false;
	else opponentID = oppList[0].num;
	if (!pVision->TheirPlayer(opponentID).Valid()) {
		opponentID = getTheirMostClosetoPosPlayerNum(pVision, pVision->Ball().Pos());
	}
	const PlayerVisionT& opponent = pVision->TheirPlayer(opponentID);
	if (Advance_DEBUG_ENGINE)GDebugEngine::Instance()->gui_debug_msg(opponent.Pos(), "Best Opp!", COLOR_WHITE);
	CVector opponent2ball = ball.Pos() - opponent.Pos();
	CVector opp2ourGoal = ourGoal - opponent.Pos();
	double opponent2ball_diff = fabs(Utils::Normalize(opponent2ball.dir() - opponent.Dir()));
	double judgeDist = OPP_HAS_BALL_DIST;
	if (opponent2ball.mod() < judgeDist && opponent2ball_diff < Param::Math::PI * 70 / 180)
		return true; // take opponent's direction into consideration.If direction not towards the ball,ignore it
	else
		return false;
}
int CAdvance::getTheirMostClosetoPosPlayerNum(const CVisionModule* pVision, CGeoPoint pos) {
	double dist = 1000;
	int num = 0;
	for (int i = 0; i < Param::Field::MAX_PLAYER; i++) {
		if (pVision->TheirPlayer(i).Valid()) {
			if (pVision->TheirPlayer(i).Pos().dist(pos) < dist) {
				dist = pVision->TheirPlayer(i).Pos().dist(pos);
				num = i;
			}
		}
	}
	return num;
}
bool CAdvance::checkBallFront(const CVisionModule* pVision, double angle) { 
	/*�ж����Ƿ��ڵ���ǰ�� ���ڼн�Ҫ��*/
	const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
	const BallVisionT& ball = pVision->Ball();
	CVector opp2ball = ball.Pos() - opp.Pos();
	bool ballDirFrontOpp = abs(Utils::Normalize(opp.Dir() - opp2ball.dir())) < angle;
	bool ballDistFrontOpp = opp2ball.mod() < OPP_HAS_BALL_DIST + 10;
	//GDebugEngine::Instance()->gui_debug_line(opp.Pos(),opp.Pos() + Utils::Polar2Vector(200 , 0),COLOR_BLACK);
	bool isBallFrontOpp = ballDirFrontOpp && ballDistFrontOpp;
	return isBallFrontOpp;
}

bool CAdvance::isPassBalltoMe(const CVisionModule* pVision, int vecNumber) {
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	const BallVisionT& ball = pVision->Ball();
	CVector ball2me = me.Pos() - ball.Pos();
	double diff_ballMoving2Me = Utils::Normalize(ball2me.dir() - ball.Vel().dir());
	if (ball.Valid() && abs(diff_ballMoving2Me) < Param::Math::PI / 10 && (ball2me.mod() / ball.Vel().mod() < BalltoMeVelTime)) {// 
		return true;
	}
	return false;
}

bool CAdvance::isTheLineBlocked(const CVisionModule* pVision, CGeoPoint startPoint, CGeoPoint targetPoint) {
	/*����·�����Ƿ�ᱻ�����赲*/
	double k_m = WantToLessShoot;
	double opp2LineDist = TheMinDistBetweenTheOppAndTheLine(pVision, startPoint, targetPoint);
	if (opp2LineDist < k_m * Param::Vehicle::V2::PLAYER_SIZE)return true;
	return false;
}

bool CAdvance::IsOurNearHere(const CVisionModule* pVision, CGeoPoint checkPoint, const int vecNumber) {
	const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
	const BallVisionT& ball = pVision->Ball();

	for (int i = 0; i < 8; i++) {
		if (vecNumber == i)continue;
		if (pVision->OurPlayer(i).Valid()) {
			const PlayerVisionT& me = pVision->OurPlayer(i); 
			if ((me.Pos() - checkPoint).mod() < 100) {
				return true;
			}
		}
	}
	return false;
}
bool CAdvance::Me2OppTooclose(const CVisionModule* pVision, const int vecNumber) { //�Ƿ�̫����
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
	const BallVisionT& ball = pVision->Ball();
	CVector me2Ball = ball.Pos() - me.Pos();
	CVector me2Opp = opp.Pos() - me.Pos();
	if ((abs(me2Ball.mod()) < 80 && abs(me2Opp.mod()) < 80) && (me2Ball.dir() - me2Opp.dir() < Param::Math::PI /  3)) {
		if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(450, 450), "TOO CLOSE with ball", COLOR_ORANGE);
		return true;
	}
	return false;
}

bool CAdvance::isDirOK(const CVisionModule* pVision, int vecNumber, double targetDir, int ShootOrPass) {
	double ShootPrecision = SHOOT_PRECISION;
    //double offset = 0.05;

	const BallVisionT& ball = pVision->Ball();
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
	double myDir = me.Dir();
	CVector opp2ball = ball.Pos() - opp.Pos();
	CVector ball2goal = theirCenter - ball.Pos();
    if (!ShootOrPass) ShootPrecision = ShootPrecision * 0.8;
    /*if (myDir - targetDir > 0)targetDir -= offset;
    else targetDir += offset;
            ��������  ����������
    */
	last_target_dir = targetDir;

    if (ShootOrPass) {
        CGeoLine start2Target = CGeoLine(me.Pos(), myDir);
        CGeoLineLineIntersection Intersection = CGeoLineLineIntersection(start2Target, GOATLINE);
        if (abs(Intersection.IntersectPoint().y()) > 60) return false;
    }
    /*�������λ����������*/

    if (abs(targetDir - last_target_dir) > 0.3 * Param::Math::PI / SHOOT_PRECISION) {
		last_dir_deviation = 100;  //���ýǶȲ�
	}
	if (Me2OppTooclose(pVision, vecNumber)) {
		/*̫���� ����*/
        if (abs(myDir - targetDir) < 0.5 * Param::Math::PI / SHOOT_PRECISION) {
			last_dir_deviation = 100;
			return true;
		}
	}
	if ((ShootOrPass && ball2goal.mod() < 250) || (opp2ball.mod() < 200)) {
		/*��������������Ҿ��������㹻�� ����Ҫ����ĵ���
		  ����������ұȽϽ��� �ٵ������޷�������*/
		if (abs(myDir - targetDir) < 0.35 * Param::Math::PI / SHOOT_PRECISION) {
			last_dir_deviation = 100;
			return true;
		}
	}
	if (abs(myDir - targetDir) > 0.25 * Param::Math::PI / SHOOT_PRECISION) {
		/*����Ƕȹ��� Ӧ��Ϊfalse*/
		last_dir_deviation = myDir - targetDir;
		return false;
	}
	else if ((abs(myDir - targetDir) > abs(last_dir_deviation) || (myDir - targetDir) * last_dir_deviation <= 0)){
		/*�������ϴνǶȲ�û�еõ���Ч����*/
		if (abs(myDir - targetDir) < 0.2 * Param::Math::PI / SHOOT_PRECISION) {
			/*�����Զ��Կ��Խ�����*/
			last_dir_deviation = 100;
			return true;
		}
	}
	else if (abs(myDir - targetDir) < 0.15 * Param::Math::PI / SHOOT_PRECISION) {
		/*������Ȼ�ڵ��� ���ǿ������������ķ�Χ������*/
		last_dir_deviation = 100;
		return true;
	}
	last_dir_deviation = myDir - targetDir;
	return false;
}
bool CAdvance::isInBreakArea(const CVisionModule* pVision, int vecNumber) {
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	if (abs(me.Y()) < 180 && me.X() < 540 && me.X() > 315)return true;
	return false;
}
bool CAdvance::JudgeIsMeSupport(const CVisionModule* pVision, int vecNumber) {
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	return abs(me.Y()) < 200;
}
/**********************************************************
	* Description: ״̬�л��ж��ຯ��������״̬ת��֮����ж�
	* Author: ̷���
	* Created Date: 2022/10/10
***********************************************************/
bool CAdvance::tendToShoot(const CVisionModule* pVision, int vecNumber) {
	/*�ж������ܷ�����*/
	// NEEDMODIFY
	int n = 0;
	int best_n = 0;
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	const BallVisionT& ball = pVision->Ball();
	bool shootBlocked = false;
	double kickDir = KickDirection::Instance()->getPointShootDir(pVision, pVision->OurPlayer(vecNumber).Pos());
	if (fabs(kickDir - 1000.0) < 10) return false;
	/*�޸�tendToShoot��getPointShootDir�жϲ����ݵ�����*/
	CGeoLine ball2ourGoal = CGeoLine(me.Pos(), kickDir);
	CGeoPoint projectionPoint;
	double k_m = WantToLessShoot;
	double opp2LineDist = 1000;
	while (n <= Param::Field::MAX_PLAYER) {
		if (!pVision->TheirPlayer(n).Valid()) { n++; continue; }
		projectionPoint = ball2ourGoal.projection(pVision->TheirPlayer(n).Pos());
		if (opp2LineDist > (projectionPoint - pVision->TheirPlayer(n).Pos()).mod() && projectionPoint.x() >= me.X()) {
			opp2LineDist = (projectionPoint - pVision->TheirPlayer(n).Pos()).mod();
			best_n = n;
			if (opp2LineDist < k_m * Param::Vehicle::V2::PLAYER_SIZE) {
				shootBlocked = true;
				break;
			}
		}
		n++;
	}
	KickDirection::Instance()->GenerateShootDir(vecNumber, pVision->OurPlayer(vecNumber).Pos());
	bool kickValid = KickDirection::Instance()->getIsKickValid();
	const PlayerVisionT& opp = pVision->TheirPlayer(best_n);
	double me2theirbest = (me.Pos() - opp.Pos()).mod();
	double me2goal = (me.Pos() - theirCenter).mod();

	if (shootBlocked) return false;
	else return kickValid;
}

int CAdvance::CanSupportKick(const CVisionModule* pVision, int vecNumber) {
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	bool isBlock = 0 ;
	double SupportShootDir = 0, MeToSupportDist = 0 , SupportToGoal = 0 ;
	for (int i = 0; i < NumberOfSupport; ++i) {
		isBlock = isTheLineBlocked(pVision, me.Pos(), SupportPoint[i]);
		SupportShootDir = KickDirection::Instance()->getPointShootDir(pVision, SupportPoint[i]);
		MeToSupportDist = (me.Pos() - SupportPoint[i]).mod();
		SupportToGoal = (CGeoPoint(Param::Field::PITCH_LENGTH / 2.0, 0) - SupportPoint[i]).mod();
		if (!isBlock && SupportShootDir != 1000 && MeToSupportDist < CanPassToWingDist && SupportToGoal < CanWingShootDist)
			return 1;
	}
	return 0;
}
int CAdvance::toChipOrToFlat(const CVisionModule* pVision, int vecNumber) {
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	for (int i = 0; i < NumberOfSupport; ++i) 
		if (!isTheLineBlocked(pVision, me.Pos(), SupportPoint[i]))
			return 1;
	return 0;
}

/**********************************************************
	* Description: �����ຯ����������GET��ʹ��
	* Author: ̷���
	* Created Date: 2022/10/10
***********************************************************/
bool CAdvance::isOppFaceOurDoor(const CVisionModule* pVision, double angle) {
	//�ж��Ƿ������opp�ܽ���opp�����������źܽ�
	const BallVisionT& ball = pVision->Ball();
	const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
	double opp2BallDist = (opp.Pos() - ball.Pos()).mod();
	bool isBallNearOpp = opp2BallDist < OPP_HAS_BALL_DIST;
	double judgeAngle = abs(Utils::Normalize((opp.Dir() - CVector(CGeoPoint(-Param::Field::PITCH_LENGTH / 2.0, 0) - opp.Pos()).dir())));
	bool isFaceOurDoor = judgeAngle < angle || judgeAngle == angle;
	return isFaceOurDoor && isBallNearOpp;
}
bool CAdvance::checkTheyCanShoot(const CVisionModule* pVision, int vecNumber) {
	const BallVisionT& ball = pVision->Ball();
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
	CVector opp2ball = ball.Pos() - opp.Pos();
	CVector me2ball = ball.Pos() - me.Pos();
	double opp2ballDist = opp2ball.mod();
	double me2ballDist = me2ball.mod();
	if (isOppFaceOurDoor(pVision, Param::Math::PI / 6.0) && checkBallFront(pVision, Param::Math::PI / 6.0) && ((me2ballDist - opp2ballDist) > -5) && (opp.Pos() - ourGoal).mod() < Param::Field::GOAL_WIDTH / 2) {
		return true;
	}
	else { return false; }
}

/**********************************************************
	* Description: �����ຯ�������о���ʵ��
	* Author: ̷���
	* Created Date: 2022/10/10
***********************************************************/
PassDirOrPos CAdvance::PassDirInside(const CVisionModule* pVision, int vecNumber) {
	PassDirOrPos ReturnValue; 
	/*�ж�SupportPoint��λ�Ƿ����ʹ��*/
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	bool isOurNearPoint[9] = { 0 }, isBlockPoint[9] = { 0 }, isCanUse[9] = { 0 }, OneOfUsCanShoot = 0;
	int TheNumberOfCanShootPoint = 0, NowShootNumber = 0, TheidxOfCanShootPoint[9] = { 0 };
	double ShootDir[9] = { 0 }, ChangeDir[9] = { 0 }, DistToPoint[9] = { 0 }, DistOppToTheLine[9] = { 0 };
	double FinalDir = 0;
	/*����Ϊ�����Ƿ������ڴ���㸽���������Ƿ��赲�ô���·�����õ��ܷ�ʹ�ã�������û�е�����Ҫ���ܹ�shoot�ĵ��idx*/
	/*����Ϊ������Ҫ��ĵ������TheNumberOfCanShootPoint���־û����number��shootdir����Ҫת�䷽���Dir�����սǶ�Dir*/
	for (int i = 0; i < NumberOfSupport; ++i) {
		isOurNearPoint[i] = IsOurNearHere(pVision, SupportPoint[i], vecNumber);
		isBlockPoint[i] = isTheLineBlocked(pVision, me.Pos(), SupportPoint[i]);
		DistOppToTheLine[i] = TheMinDistBetweenTheOppAndTheLine(pVision, me.Pos(), SupportPoint[i]);
		ShootDir[i] = KickDirection::Instance()->getPointShootDir(pVision, SupportPoint[i]);
		DistToPoint[i] = (SupportPoint[i] - me.Pos()).mod();
		ChangeDir[i] = me.Dir() - (SupportPoint[i] - me.Pos()).dir();
		isCanUse[i] = isOurNearPoint[i] && (!isBlockPoint[i]) && (ShootDir[i] != 1000) && (SupportPoint[i].x() > me.X());
		/*��ǰ��������ŵ��������ҷ��������Աߣ�û���赲�����ſ���*/
		if (isCanUse[i])OneOfUsCanShoot = 1;
	}
    ReturnValue.dir = (SupportPoint[LastPassPoint] - me.Pos()).dir();
    ReturnValue.pos = SupportPoint[LastPassPoint];
    if (SupportPoint[LastPassPoint].x() > me.X() && isOurNearPoint[LastPassPoint] && (abs(me.Dir() - (SupportPoint[LastPassPoint] - me.Pos()).dir()) < 0.5 * Param::Math::PI / SHOOT_PRECISION)) {
        return ReturnValue;
	}
	/*����ϵͳ�ȶ��� */

	if (OneOfUsCanShoot) {
		/*���������һ�������ǰ�ᣬ��ֻ������Щ��������Ҫ��ĵ�*/
		IsMeSupport = JudgeIsMeSupport(pVision, vecNumber);
		if (IsMeSupport) {
			if (isCanUse[1]) {
				ReturnValue.dir = (SupportPoint[1] - me.Pos()).dir();
				ReturnValue.pos = SupportPoint[1];
				return ReturnValue;
			}
			if (isCanUse[4]) {
				ReturnValue.dir = (SupportPoint[4] - me.Pos()).dir();
				ReturnValue.pos = SupportPoint[4];
				return ReturnValue;
			}
		}
		for (int i = 0; i < NumberOfSupport; ++i)
            if (isCanUse[i] && SupportPoint[i].x() > -330)TheidxOfCanShootPoint[TheNumberOfCanShootPoint++] = i;
    }
    else {
		/*��������˵ĵط�ȫ�����Դ� ��Ϊ�����Ѿ��ھ�����������*/
		IsMeSupport = JudgeIsMeSupport(pVision, vecNumber);
		if (IsMeSupport) {
			if (isOurNearPoint[1]) {
				ReturnValue.dir = (SupportPoint[1] - me.Pos()).dir();
				ReturnValue.pos = SupportPoint[1];
				return ReturnValue;
			}
			if (isOurNearPoint[4]) {
				ReturnValue.dir = (SupportPoint[4] - me.Pos()).dir();
				ReturnValue.pos = SupportPoint[4];
				return ReturnValue;
			}
		}
		for (int i = 0; i < NumberOfSupport; ++i)
            if(isOurNearPoint[i] && SupportPoint[i].x() > -330)TheidxOfCanShootPoint[TheNumberOfCanShootPoint++] = i;
    }
	double NowValue = -1, MinValue = 1e9; int Maxidx = -1;
	for (int i = 0; i < TheNumberOfCanShootPoint; ++i) {
		int NowIdx = TheidxOfCanShootPoint[i];
		NowValue = ChangeDir[NowIdx];
		if (NowValue < MinValue)Maxidx = NowIdx, MinValue = NowValue;
	}
    if (Maxidx < 0)Maxidx = 1;
	NowShootNumber = Maxidx;
	ReturnValue.dir = (SupportPoint[NowShootNumber] - me.Pos()).dir();
	ReturnValue.pos = SupportPoint[NowShootNumber];
	LastPassPoint = NowShootNumber;
	return ReturnValue;
}
double CAdvance::PassDir(const CVisionModule* pVision, int vecNumber) {
	PassDirOrPos ReturnValue = PassDirInside(pVision, vecNumber);
	return ReturnValue.dir;
}
double CAdvance::TheMinDistBetweenTheOppAndTheLine(const CVisionModule* pVision, CGeoPoint startPoint, CGeoPoint targetPoint) {
	/*����·�����Ƿ�ᱻ�����赲*/
	int n = 0;
	const BallVisionT& ball = pVision->Ball();
	double passDir = (targetPoint - startPoint).dir();
	CGeoLine start2Target = CGeoLine(startPoint, passDir);
	CGeoPoint projectionPoint;
	double opp2LineDist = 1000;
	while (n <= Param::Field::MAX_PLAYER) {
		if (!pVision->TheirPlayer(n).Valid()) { n++; continue; }
		projectionPoint = start2Target.projection(pVision->TheirPlayer(n).Pos());
		if (opp2LineDist > (projectionPoint - pVision->TheirPlayer(n).Pos()).mod() && projectionPoint.x() < Param::Field::PITCH_LENGTH / 2.0 && projectionPoint.x() > startPoint.x()) {
			opp2LineDist = (projectionPoint - pVision->TheirPlayer(n).Pos()).mod();
		}
		n++;
	}
	return opp2LineDist;
}
CGeoPoint CAdvance::GenerateBreakShootPoint(const CVisionModule* pVision, int vecNumber) {
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
    CGeoPoint ShootPoint = KickDirection::Instance()->GetTheShootPoint(pVision, me.Pos());
    ShootPoint.setY(ShootPoint.y() - me.VelY()*3);
    //double MeVel = me.VelY()
    if(ShootPoint.y() > 50)return GOATPoint1;
    else if(ShootPoint.y() < -50)return GOATPoint2;
    return ShootPoint;
}

CGeoPoint CAdvance::GenerateBreakPassPoint(const CVisionModule* pVision, int vecNumber) {
	PassDirOrPos ReturnValue = PassDirInside(pVision, vecNumber);
	return ReturnValue.pos;
}

double CAdvance::GetFPassPower(CGeoPoint StartPoint, CGeoPoint targetPoint) {
    double dist = (StartPoint - targetPoint).mod();
    return min(600.0, ADV_FPASSPOWER_Alpha* dist);
}
double CAdvance::GetCPassPower(CGeoPoint StartPoint, CGeoPoint targetPoint) {
    double dist = (StartPoint - targetPoint).mod();
    return min(350.0, ADV_CPASSPOWER_Alpha * dist);
}

CPlayerCommand* CAdvance::execute(const CVisionModule* pVision)
{
	if( subTask() ){
		return subTask()->execute(pVision);
	}
	if( _directCommand ){
		return _directCommand;
	}
	return 0;
}

