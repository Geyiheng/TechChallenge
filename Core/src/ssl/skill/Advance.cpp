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
	KickorPassDir = 0;
	KICK_DIST = paramManager->KICK_DIST;  /*射门允许范围 越高越容易射门*/
	WantToLessShoot = paramManager->WantToLessShoot; /*射门倾向，越低越容易射门 最低为0 最高为5*/
	RELIEF_DIST = paramManager->RELIEF_DIST;  /*GET中紧急状况下的RELIEF判断距离*/
	OPP_HAS_BALL_DIST = paramManager->OPP_HAS_BALL_DIST; /*判断敌方是否有球的距离 需要调整*/
	CanPassToWingDist = paramManager->CanPassToWingDist; /*Advance能够传给边锋的临界距离*/
	CanWingShootDist = paramManager->CanWingShootDist; /*边锋能够射门的临界距离*/
	SHOOT_PRECISION = paramManager->SHOOT_PRECISION;	/*允许射门最小精度角分母，越大越慢越精确 最低为7最高17*/
	GetBallBias = paramManager->AdGetBallBias;	/*AdvanceGetball的偏差*/
	BalltoMeVelTime = paramManager->BalltoMeVelTime; /*Advance传球给我主动去接的临界时间*/
	OBSTACLE_RADIUS = paramManager->BREAK_OBSTACLE_RADIUS;
	/*射门力度参数*/
	KICKPOWER = paramManager->KICKPOWER;
	CHIPPOWER = paramManager->CHIPPOWER; // 暂时不用了
	ADV_FPASSPOWER_Alpha = paramManager->ADV_FPASSPOWER;
	ADV_CPASSPOWER_Alpha = paramManager->ADV_CPASSPOWER;
	PUSHPOWER = paramManager->ADV_PUSHPOWER;
	// max:600 350
	RELIEF_POWER = paramManager->RELIEF_POWER;
	BACK_POWER = paramManager->BACK_POWER;
	Advance_DEBUG_ENGINE = paramManager->Advance_DEBUG_ENGINE;
	// GetBallV4
	LARGE_ADJUST_ANGLE = paramManager->LARGE_ADJUST_ANGLE;


}


CAdvance::~CAdvance() {

}

void CAdvance::plan(const CVisionModule* pVision)
{
	if (pVision->Cycle() - _cycle > Param::Vision::FRAME_RATE * 0.1) {
		_state = BEGIN;
	}
	/**********************************************************
	* Description: 初始化必要基础参数列表
	* Author: 谭宇宏
	* Created Date: 2022/10/10
	***********************************************************/
	const int maxFrared = 100 * 1.25;
	const int maxMeHasBall = int(50 * 1.25);
	int _executor = task().executor;

	int DoNotEnterDefenseBox = PlayerStatus::DODGE_OUR_DEFENSE_BOX;
	int AllowDribbleFlag = PlayerStatus::DRIBBLING;
	int ShootAllowDribble = DoNotEnterDefenseBox | AllowDribbleFlag;
	int ShootNotNeedDribble = DoNotEnterDefenseBox & (~AllowDribbleFlag);
	bool frared = RobotSensor::Instance()->IsInfraredOn(_executor);
	if (frared) { infraredOn = infraredOn >= maxFrared ? maxFrared : infraredOn + 1; }
	else { infraredOn = 0; }

	/**********************************************************
	* Description: 初始化任务参数列表
	* Author: 谭宇宏
	* Created Date: 2022/10/10
	***********************************************************/
	const PlayerVisionT& me = pVision->OurPlayer(_executor);
	const BallVisionT& ball = pVision->Ball();
	int GoalieNumber = 0;
	int NumofPlayerInFrontfiled = 0;


	double BallToOurGoal = (ball.Pos() - ourGoal).mod();
	CVector me2goal = theirCenter - me.Pos();
	bool isOppHasBall = checkOppHasBall(pVision);
	const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
	const CVector opp2ball = ball.Pos() - opp.Pos();
	double ball2oppDist = opp2ball.mod();
	double ball2meDist = (ball.Pos() - me.Pos()).mod();

	double me2BestOppDist = CVector(pVision->TheirPlayer(opponentID).Pos() - me.Pos()).mod();

	if (fabs(KickorPassDir) < 1e-3) {
		KickorPassDir = KickDirection::Instance()->getPointShootDir(pVision, pVision->OurPlayer(_executor).Pos());
	}
	//LastPassDirToJudge = -999;
	//bool JudgePassMeIsBeBlocked(const CVisionModule *pVision, int vecNumber);


	PassDirOrPos TMP;
	//CGeoPoint PassPoint;
	/*??痦??*/

	CGeoPoint ShootPoint, PassPoint;/*传球与射门的方向 应该用一个变量表示 具有可持续化的作用*/
	ShootPoint = GenerateBreakShootPoint(pVision, _executor);
	PassPoint = GenerateBreakPassPoint(pVision, _executor);
	NumberOfSupport = min(6, AREANUM);/*暂时只考虑对面半场六个*/
	for (int i = 0; i < NumberOfSupport; ++i)
		SupportPoint[i] = GPUBestAlgThread::Instance()->getBestPointFromArea(i);/* Gpu算点 */
	// 可视化球的预测位置
	/*
	for (int i = 0; i < 6; i++) {
		CGeoPoint ball_predict_pos = GPUBestAlgThread::Instance()->getBallPosFromFrame(ball.Pos(), ball.Vel(), i * 8);
		GDebugEngine::Instance()->gui_debug_msg(ball_predict_pos, (to_string(i * 8)).c_str(), COLOR_YELLOW);
		GDebugEngine::Instance()->gui_debug_x(ball_predict_pos, COLOR_BLUE);
	}
	*/

	//	NormalPlayUtils::generatePassPoint(ball.Pos(), SupportPoifnt[0], SupportPoint[1], SupportPoint[2], SupportPoint[3]);

	IsMeSupport = JudgeIsMeSupport(pVision, _executor);/*判断我是不是support 用于传中*/
	IHaveSupport = CanSupportKick(pVision, _executor); // 是否存在支援车

	/**********************************************************
	* Description: 屏幕右侧 画出has与lose的图像
	* Author: 谭宇宏
	* Created Date: 2022/10/10
	***********************************************************/


	for (int i = 0; i < Param::Field::MAX_PLAYER; i++) {
		if (pVision->OurPlayer(i).Valid() && i != GoalieNumber) {
			NumOfOurPlayer++;
			if (pVision->OurPlayer(i).Pos().x() > Param::Field::PITCH_LENGTH / 10)
				NumofPlayerInFrontfiled++;
		}
	}
	/**********************************************************
	* Description: 状态分配
	* Author: 谭宇宏
	* Created Date: 2022/10/10
	***********************************************************/
	Advance_DEBUG_ENGINE = 1;

	bool IsSimulator = 1;
	MeIsInWhichArea = InWhichArea(pVision, _executor);

	switch (_state) {
	case BEGIN:
		_state = GET;
		break;
	case GET:
		if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -400), "Push GET", COLOR_YELLOW);

		if (checkOppHasBall(pVision)) {
			if (BallStatus::Instance()->getBallPossession(true, _executor) > 0.3) {
				_state = GET; break;
			} // 正在两方对吸
			else {
				if (WeNeedBlockTheBall(pVision, _executor)) {
					_state = BLOCK;
					break;
				}
				else { _state = GET; break; }
			}
		}	// 敌人持球，此时需要进行防守

		/*
		else if (NumOfOurPlayer <= 2) {
			if (BallStatus::Instance()->getBallPossession(true, _executor) > 0.3) {
				TaskMediator::Instance()->resetAdvancerPassTo();
				_state = GenerateStateOfFoulTrouble(pVision, _executor);
				break;
			}
			else {
				_state = GET;
				break;
			}
		}	// 我方陷入犯规麻烦
		*/
		if (BallStatus::Instance()->getBallPossession(true, _executor) > 0.3) {
			_state = GenerateNextState(pVision, _executor);
			break;
		}
		else {
			_state = GenerateNextState(pVision, _executor);
			if (CanWeUseChaseBecauseOfGetBallV3(pVision, _executor)) {
				if (_state == KICK) {
					_state = CHASEKICK;
					break;
				}
				else if (_state == PUSHOUT) {
					_state = CHASEPUSH;
					break;
				}
			}
			_state = GET;
			break;
		}
		break;

	case KICK:
		if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -400), "Push KICK", COLOR_YELLOW);
		// if (meLoseBall > 10 && ball2meDist > 10) _state = GET;
		if (BallStatus::Instance()->getBallPossession(true, _executor) == 0 && ball2meDist > 10) _state = GET;
		break;
	case PASS:
		if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -400), "Push PASS", COLOR_YELLOW);
		// if (meLoseBall > 10 && ball2meDist > 10) _state = GET;
		if (BallStatus::Instance()->getBallPossession(true, _executor) == 0 && ball2meDist > 10) _state = GET;
		break;
	case JUSTCHIPPASS:
		if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -400), "Push CHIP", COLOR_YELLOW);
		// if (meLoseBall > 10 && ball2meDist > 10) _state = GET;
		if (BallStatus::Instance()->getBallPossession(true, _executor) == 0 && ball2meDist > 10) _state = GET;
		break;
	case BREAKSHOOT:
		if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -400), "Push BREAK", COLOR_YELLOW);
		// if (meLoseBall > 10 && ball2meDist > 10) _state = GET;
		if (BallStatus::Instance()->getBallPossession(true, _executor) == 0 && ball2meDist > 10) _state = GET;
		break;
	case PUSHOUT:
		if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -400), "Push OUT", COLOR_YELLOW);
		// if (meLoseBall > 10 && ball2meDist > 10) _state = GET;
		if (BallStatus::Instance()->getBallPossession(true, _executor) == 0 && ball2meDist > 10) _state = GET;
		break;
	case BREAKPASS:
		if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -400), "Push BreakPass", COLOR_YELLOW);
		// if (meLoseBall > 10 && ball2meDist > 10) _state = GET;
		if (BallStatus::Instance()->getBallPossession(true, _executor) == 0 && ball2meDist > 10) _state = GET;
		break;
	case BLOCK:
		if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -400), "Push BreakPass", COLOR_YELLOW);
		if (!WeNeedBlockTheBall(pVision, _executor)) _state = GET;
		break;

	case CHASEKICK:
		if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -400), "Push CHASEKICK", COLOR_YELLOW);
		if (GenerateNextState(pVision, _executor) != KICK || !CanWeUseChaseBecauseOfGetBallV3(pVision, _executor)) _state = GET;
		if (BallStatus::Instance()->getBallPossession(true, _executor) > 0.3 ) _state = KICK;
		break;
	case CHASEPUSH:
		if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -400), "Push CHASEPUSH", COLOR_YELLOW);
		if (GenerateNextState(pVision, _executor) != PUSHOUT || !CanWeUseChaseBecauseOfGetBallV3(pVision, _executor)) _state = GET, cout << "why hereeeee??" << ' ' << GenerateNextState(pVision, _executor) << endl;

		if (BallStatus::Instance()->getBallPossession(true, _executor) > 0.3 ) _state = PUSHOUT;
		break;
	}
	/*debug here
	*/
	if (BallStatus::Instance()->getBallPossession(true, _executor) > 0.3) {
		_state = BREAKPASS;
		//_state = BREAKSHOOT;
		//_state = PUSHOUT;
	}
	else _state = GET;
	/**********************************************************
	* Description: 状态执行
	* Author: 谭宇宏
	* Created Date: 2022/10/10
	***********************************************************/
	bool isChipKick = 0;
	double kickPower = 0;
	if (_state != KICK && _state != BREAKSHOOT) {
		NowIsShoot = 0;
	}

	switch (_state) {
	case GET:
		if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(200, -400), "let GET", COLOR_YELLOW);
		if (ball2meDist > 50 || (isPassBalltoMe(pVision, _executor)))NowIsShoot = 0;
		/*清空shoot标记*/
		/*
		if (BallToOurGoal < RELIEF_DIST && ball2oppDist < 30) {
			/*需要RELIEF的紧急情况
			if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -400), "RELIEF", COLOR_ORANGE);
			KickorPassDir = KickDirection::Instance()->getPointShootDir(pVision, pVision->OurPlayer(_executor).Pos());
			KickStatus::Instance()->setChipKick(_executor, RELIEF_POWER);
			setSubTask(PlayerRole::makeItChaseKickV2(_executor, KickorPassDir, ShootNotNeedDribble));
		}
		else
		*/
		if (!ball.Valid()) {
			/*球不合法的情况*/
			if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -400), "Ball invalid", COLOR_ORANGE);
			double faceDir = opp.Dir() + Param::Math::PI;
			setSubTask(PlayerRole::makeItChaseKickV2(_executor, faceDir, ShootNotNeedDribble));
		}
		else if (checkTheyCanShoot(pVision, _executor)) {
			/*敌人有射门机会 进行封堵*/
			if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -400), "they can shoot", COLOR_ORANGE);
			double faceDir = opp.Dir() + Param::Math::PI;
			setSubTask(PlayerRole::makeItChaseKickV2(_executor, faceDir, ShootNotNeedDribble));
		}
		else if (isPassBalltoMe(pVision, _executor) /*&& !JudgePassMeIsBeBlocked(pVision, _executor)*/) {
			/*我方给我进行传球*/
			if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -350), "pass ball to me", COLOR_ORANGE);

			if (me2goal.mod() < KICK_DIST && (Me2OppTooclose(pVision, _executor) || isInBreakArea(pVision, _executor))) {
				KickorPassDir = KickDirection::Instance()->getPointShootDir(pVision, pVision->OurPlayer(_executor).Pos());
			}
			else KickorPassDir = PassDir(pVision, _executor);
			setSubTask(PlayerRole::makeItReceivePass(_executor, KickorPassDir));
		}
		else {
			/*并没有得到球 需要去getball*/
			if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -300), "LOSE and GETBALL", COLOR_ORANGE);
			//KickorPassDir = KickDirection::Instance()->getPointShootDir(pVision, pVision->OurPlayer(_executor).Pos());
			/*此处朝向可持久化即可 不需要进行改变*/
			LastPassDirToJudge = -999;
			//cout << OppIsNearThanMe(pVision, _executor) << ' ' << OppIsFarThanMe(pVision, _executor) << endl;
			//if (OppIsNearThanMe(pVision, _executor)) KickorPassDir = generateOppIsNearThanMeDir(pVision, _executor);
			//else if (OppIsFarThanMe(pVision, _executor)) KickorPassDir = generateOppIsFarThanMeDir(pVision, _executor);

			setSubTask(PlayerRole::makeItNoneTrajGetBall(_executor, KickorPassDir, CVector(0, 0), ShootNotNeedDribble, GetBallBias));
		}
		break;
	case KICK:   // 射门
		NowIsShoot = 1;
		if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(200, -400), "KICK", COLOR_YELLOW);
		KickorPassDir = KickDirection::Instance()->getPointShootDir(pVision, pVision->OurPlayer(_executor).Pos());
		//KickStatus::Instance()->setBothKick(_executor, 0, 0);
		if (Utils::InTheirPenaltyArea(ball.Pos(), 0)) {
			/*如果球在对方禁区*/
			if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -400), "ball in their PEN", COLOR_ORANGE);
			KickStatus::Instance()->setKick(_executor, KICKPOWER);
			//setSubTask(PlayerRole::makeItShootBallV2(_executor, KickorPassDir, ShootNotNeedDribble));
		}
		else {
			/*正常KICK阶段  需要区分是否方向已经转向成功  此处尚未完备可能存在BUG*/
			if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -400), "Let Kick", COLOR_ORANGE);

			//setSubTask(PlayerRole::makeItDribbleTurnKickV2(_executor, KickorPassDir, 0.2 * Param::Math::PI / SHOOT_PRECISION, 0, KICKPOWER, PassPoint));
			//if (canScore(pVision, _executor, OBSTACLE_RADIUS, me.Dir())) {
			if (isDirOK(pVision, _executor, KickorPassDir, 1)) {
				if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -350), "Kick isDirOK", COLOR_ORANGE);
				KickStatus::Instance()->setKick(_executor, KICKPOWER);
				setSubTask(PlayerRole::makeItSimpleGoto(_executor, ball.Pos(), (ball.Pos() - me.Pos()).dir(), task().player.flag));
			}
			else {
				if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -350), "Kick is NOT DirOK ", COLOR_ORANGE);

				CGeoPoint TargetUp = CGeoPoint(Param::Field::PITCH_LENGTH / 2, -Param::Field::GOAL_WIDTH / 2 + 5);
				CGeoPoint TargetDown = CGeoPoint(Param::Field::PITCH_LENGTH / 2, Param::Field::GOAL_WIDTH / 2 - 5);
				double thetaUp = (TargetUp - ball.Pos()).dir();
				double thetaDown = (TargetDown - ball.Pos()).dir();
				if (me.Dir() < thetaUp) KickorPassDir = thetaDown;
				else if (me.Dir() > thetaDown)KickorPassDir = thetaUp;

				setSubTask(PlayerRole::makeItNoneTrajGetBall(_executor, KickorPassDir, CVector(0, 0), ShootNotNeedDribble, GetBallBias));
			}
		}
		break;
	case PASS:
		if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(200, -400), "PASS", COLOR_YELLOW);
		//KickStatus::Instance()->setBothKick(_executor, 0, 0);
		TMP = PassDirInside(pVision, _executor);
		KickorPassDir = TMP.dir;
		PassPoint = TMP.pos;


		if (isDirOK(pVision, _executor, KickorPassDir, 0)) {
			double ThePower = 0;
			bool IsFlatKick = toChipOrToFlat(pVision, _executor, PassPoint);
			if (IsFlatKick) {
				ThePower = GetFPassPower(me.Pos(), PassPoint);
				KickStatus::Instance()->setKick(_executor, ThePower);
			}
			else {
				ThePower = GetCPassPower(me.Pos(), PassPoint);
				KickStatus::Instance()->setChipKick(_executor, ThePower);
			}
			if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -350), "PASS isDirOK", COLOR_ORANGE);
			setSubTask(PlayerRole::makeItSimpleGoto(_executor, ball.Pos(), (ball.Pos() - me.Pos()).dir(), task().player.flag));
			TaskMediator::Instance()->setAdvancerPassTo(PassPoint, NumberOfSupport);
		}
		else {
			if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -350), "PASS is NOT DirOK ", COLOR_ORANGE);
			setSubTask(PlayerRole::makeItNoneTrajGetBall(_executor, KickorPassDir, CVector(0, 0), ShootNotNeedDribble, GetBallBias));
		}
		/*
		if (toChipOrToFlat(pVision, _executor, PassPoint) == 1) {
			/*鉴定为可以平传球 则平传球
			if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -400), "FLATPASS", COLOR_ORANGE);
			setSubTask(PlayerRole::makeItDribbleTurnKickV2(_executor, KickorPassDir, 0.2 * Param::Math::PI / SHOOT_PRECISION, 0, GetFPassPower(me.Pos(), PassPoint), PassPoint));
		}
		else {
			/*鉴定为不能平传球 则选择挑传 考虑函数同样为flatPassDir
			if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -400), "CHIPPASS", COLOR_CYAN);
			setSubTask(PlayerRole::makeItDribbleTurnKickV2(_executor, KickorPassDir, 0.2 * Param::Math::PI / SHOOT_PRECISION, 1, GetCPassPower(me.Pos(), PassPoint), PassPoint));
		}
		*/
		break;

	case JUSTCHIPPASS:
		if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(200, -400), "JUSTCHIP", COLOR_YELLOW);
		//KickStatus::Instance()->setBothKick(_executor, 0, 0);
		TMP = PassDirInside(pVision, _executor);
		KickorPassDir = TMP.dir;
		PassPoint = TMP.pos;
		if (isDirOK(pVision, _executor, KickorPassDir, 1)) {
			double ThePower = GetCPassPower(me.Pos(), PassPoint);
			KickStatus::Instance()->setChipKick(_executor, ThePower);
			if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -350), "JUSTCHIP isDirOK", COLOR_ORANGE);
			setSubTask(PlayerRole::makeItSimpleGoto(_executor, ball.Pos(), (ball.Pos() - me.Pos()).dir(), task().player.flag));
			TaskMediator::Instance()->setAdvancerPassTo(PassPoint, NumberOfSupport);
		}
		else {
			if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -350), "JUSTCHIP is NOT DirOK ", COLOR_ORANGE);
			setSubTask(PlayerRole::makeItNoneTrajGetBall(_executor, KickorPassDir, CVector(0, 0), ShootNotNeedDribble, GetBallBias));
		}
		break;

	case BREAKSHOOT:
		NowIsShoot = 2;
		if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(200, -400), "BREAKSHOOT", COLOR_YELLOW);
		//KickStatus::Instance()->setBothKick(_executor, 0, 0);
		ShootPoint = (pVision->Cycle() % 60 == 0) ? GenerateBreakShootPoint(pVision, _executor) : ShootPoint;
		KickorPassDir = (ShootPoint - me.Pos()).dir();
		setSubTask(PlayerRole::makeItBreak(_executor, false, ShootPoint));
		/*
		if(AdJudgeBreakCanDo(pVision, _executor, ShootPoint)||true)setSubTask(PlayerRole::makeItBreak(_executor, false, ShootPoint));
		else setSubTask(PlayerRole::makeItNoneTrajGetBall(_executor, generateGetballDir(pVision, _executor), CVector(0, 0), ShootNotNeedDribble, GetBallBias));
		*/
		break;

	case BREAKPASS:
	{
		if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(200, -400), "BREAKPASS", COLOR_YELLOW);
		//KickStatus::Instance()->setBothKick(_executor, 0, 0);
		PassPoint = (pVision->Cycle() % 60 == 0) ? GenerateBreakPassPoint(pVision, _executor) : PassPoint;
		//PassPoint = GenerateBreakPassPoint(pVision, _executor);
		TaskMediator::Instance()->setAdvancerPassTo(PassPoint, NumberOfSupport);  //breakpass具有连续性 不适合采用setpass的技术
		bool isChipKick = toChipOrToFlat(pVision, _executor, PassPoint) == 0;
		double kickPower = 0;
		//cout << "if_chip:" << isChipKick << endl;
		KickorPassDir = (PassPoint - me.Pos()).dir();
		if (isChipKick)
			kickPower = GetCPassPower(me.Pos(), PassPoint);
		else
			kickPower = GetFPassPower(me.Pos(), PassPoint);
		//cout << "Pass Point: " << PassPoint.x() << "," << PassPoint.y() << endl;
		setSubTask(PlayerRole::makeItBreak(_executor, true, PassPoint, false, 5 * Param::Math::PI / 180, false, isChipKick, kickPower));
		break;
	}

	case PUSHOUT:
		if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(200, -400), "PUSHOUT", COLOR_YELLOW);

		//KickStatus::Instance()->setBothKick(_executor, 0, 0);

		PassPoint = generateNormalPushPoint(pVision, _executor);// : PassPoint;
		TaskMediator::Instance()->setAdvancerPassTo(PassPoint, NumberOfSupport);  //breakpass : 具有连续性 不适合采用setpass的技术
		//bool;
		isChipKick = toChipOrToFlat(pVision, _executor, PassPoint) == 0;
		//double;
		kickPower = 0;
		KickorPassDir = (PassPoint - me.Pos()).dir();
		if (isChipKick)
			kickPower = GetCPassPower(me.Pos(), PassPoint) * 0.8;
		else
			kickPower = GetFPassPower(me.Pos(), PassPoint) * 0.5;
		setSubTask(PlayerRole::makeItBreak(_executor, true, PassPoint, false, 5 * Param::Math::PI / 180, false, isChipKick, kickPower));
		/*
		if ((pVision->Cycle() % 6000 == 0)) {
			PassPoint = generateNormalPushPoint(pVision, _executor);
			KickorPassDir = (PassPoint - me.Pos()).dir();
		}
		if (!IsSimulator) {
			if (fabs((KickorPassDir - (ball.Pos() - me.Pos()).dir())) < Param::Math::PI * 20.0 / 180) {
				if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -350), "Crash The Ball", COLOR_ORANGE);
				setSubTask(PlayerRole::makeItSimpleGoto(_executor, ball.Pos() + Utils::Polar2Vector(100, (ball.Pos() - me.Pos()).dir()), (ball.Pos() - me.Pos()).dir(), task().player.flag));
			}
			else {
				if (isDirOK(pVision, _executor, KickorPassDir, 1)) {
					if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -350), "PUSHOUT isDirOK", COLOR_ORANGE);
					KickStatus::Instance()->setKick(_executor, 260);
					setSubTask(PlayerRole::makeItNoneTrajGetBall(_executor, KickorPassDir, CVector(0, 0), ShootNotNeedDribble, GetBallBias));
				}
				else {
					if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -350), "PUSHOUT isNotDirOK", COLOR_ORANGE);
					setSubTask(PlayerRole::makeItNoneTrajGetBall(_executor, KickorPassDir, CVector(0, 0), ShootNotNeedDribble, GetBallBias));
					KickStatus::Instance()->setBothKick(_executor, 0, 0);
				}
			}
		}
		else {
			if (isDirOK(pVision, _executor, KickorPassDir, 1)) {
				if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -350), "PUSHOUT isDirOK", COLOR_ORANGE);
				if(isTheLineBlocked(pVision, ball.Pos(), opp.Pos())) KickStatus::Instance()->setChipKick(_executor, 100);
				else KickStatus::Instance()->setKick(_executor, 260);
				setSubTask(PlayerRole::makeItSimpleGoto(_executor, ball.Pos(), task().player.flag));

			}
			else {
				if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -350), "PUSHOUT isNotDirOK", COLOR_ORANGE);
				setSubTask(PlayerRole::makeItNoneTrajGetBall(_executor, KickorPassDir, CVector(0, 0), ShootNotNeedDribble, GetBallBias));
				KickStatus::Instance()->setBothKick(_executor, 0, 0);
			}
		}
		*/
		//setSubTask(PlayerRole::makeItSimpleGoto(_executor, ball.Pos() + Utils::Polar2Vector(10, (ball.Pos() - me.Pos()).dir()), (ball.Pos() - me.Pos()).dir(), task().player.flag));
		/*
		if (isDirOK(pVision, _executor, KickorPassDir, 1)) {
			if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -350), "PUSHOUT isDirOK", COLOR_ORANGE);
			KickStatus::Instance()->setKick(_executor, 350);
			setSubTask(PlayerRole::makeItSimpleGoto(_executor, ball.Pos(), (ball.Pos() - me.Pos()).dir(), task().player.flag));
		}
		else {
			if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -350), "PUSHOUT is NOT DirOK ", COLOR_ORANGE);
			setSubTask(PlayerRole::makeItNoneTrajGetBall(_executor, KickorPassDir, CVector(0, 0), ShootNotNeedDribble, GetBallBias));
		}
		*/
		break;

	case CHASEKICK:
		if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(200, -400), "CHASEKICK", COLOR_YELLOW);
		KickorPassDir = KickDirection::Instance()->getPointShootDir(pVision, pVision->OurPlayer(_executor).Pos());
		if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -400), "CHASEKICK", COLOR_ORANGE);
		setSubTask(PlayerRole::makeItChaseKickV2(_executor, KickorPassDir, ShootNotNeedDribble));
		break;
	case CHASEPUSH:
		if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(200, -400), "CHASEPUSH", COLOR_YELLOW);
		if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -400), "CHASEPUSH", COLOR_ORANGE);
		PassPoint = generateNormalPushPoint(pVision, _executor);
		KickorPassDir = (PassPoint - me.Pos()).dir();
		setSubTask(PlayerRole::makeItChaseKickV2(_executor, KickorPassDir, ShootNotNeedDribble, GetFPassPower(ball.Pos(), PassPoint)));
		break;
	case BLOCK:
		if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(200, -400), "BLOCK", COLOR_YELLOW);
		KickorPassDir = (opp.Pos() - me.Pos()).dir();
		TaskT BlockTask(task());
		BlockTask.player.pos = opp.Pos() + Utils::Polar2Vector(Param::Vehicle::V2::PLAYER_FRONT_TO_CENTER, Utils::Normalize((me.Pos() - opp.Pos()).dir())); // 预测球的位置 + 5.85     这个长度越大离球越远
		BlockTask.player.angle = (opp.Pos() - me.Pos()).dir();
		BlockTask.player.needdribble = 0;
		setSubTask(TaskFactoryV2::Instance()->SmartGotoPosition(BlockTask));
		break;

	}
	//setSubTask(PlayerRole::makeItStop(_executor, 0));

	_cycle = pVision->Cycle();
	CStatedTask::plan(pVision);
}

/**********************************************************
	* Description: 检测类函数，用于视觉与位置等判断
	* Author: 谭宇宏
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
	opponentID = getTheirMostClosetoPosPlayerNum(pVision, pVision->Ball().Pos());
	const PlayerVisionT& opponent = pVision->TheirPlayer(opponentID);
	if (Advance_DEBUG_ENGINE)GDebugEngine::Instance()->gui_debug_msg(opponent.Pos(), "Best Opp!", COLOR_WHITE);
	CVector opponent2ball = ball.Pos() - opponent.Pos();
	double opponent2ball_diff = fabs(Utils::Normalize(opponent2ball.dir() - opponent.Dir()));
	double judgeDist = OPP_HAS_BALL_DIST;
	double Dirthreshold = 16.0;
	if (opponent2ball.mod() < judgeDist && opponent2ball_diff < Param::Math::PI * Dirthreshold / 180)
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
	/*判断球是否在敌人前面 存在夹角要求*/

	const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
	const BallVisionT& ball = pVision->Ball();
	CVector opp2ball = ball.Pos() - opp.Pos();
	bool ballDirFrontOpp = abs(Utils::Normalize(opp.Dir() - opp2ball.dir())) < angle;
	bool ballDistFrontOpp = opp2ball.mod() < OPP_HAS_BALL_DIST + 10;
	//GDebugEngine::Instance()->gui_debug_line(opp.Pos(),opp.Pos() + Utils::Polar2Vector(200 , 0),COLOR_BLACK);
	bool isBallFrontOpp = ballDirFrontOpp && ballDistFrontOpp;
	//printf("here %d\n",isBallFrontOpp);
	return isBallFrontOpp;
}

bool CAdvance::isPassBalltoMe(const CVisionModule* pVision, int vecNumber) {
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	const BallVisionT& ball = pVision->Ball();
	CVector ball2me = me.Pos() - ball.Pos();
	double diff_ballMoving2Me = Utils::Normalize(ball2me.dir() - ball.Vel().dir());
	const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
	CVector opp2me = me.Pos() - opp.Pos();
	//    printf("%.3f %.3f\n",(opp.Pos() - me.Pos()).mod(), Utils::Normalize(ball2me.dir() - opp2me.dir()));
	bool nearBoundary = fabs(me.Pos().x()) > Param::Field::PITCH_LENGTH / 2 - Param::Vehicle::V2::PLAYER_SIZE * 4 ||
		fabs(me.Pos().y()) > Param::Field::PITCH_WIDTH / 2 - Param::Vehicle::V2::PLAYER_SIZE * 4; // 靠近边界不能让开
	if ((opp.Pos() - me.Pos()).mod() < 60 && Utils::Normalize(ball2me.dir() - opp2me.dir()) < Param::Math::PI / 7 && !nearBoundary) return false;
	if (ball.Vel().mod() < 175.0 && !nearBoundary) return false;
	if (ball.Valid() && abs(diff_ballMoving2Me) < Param::Math::PI / 7.5 && (ball2me.mod() / ball.Vel().mod() < BalltoMeVelTime)) {//
		return true;
	}
	return false;
}

bool CAdvance::isTheLineBlocked(const CVisionModule* pVision, CGeoPoint startPoint, CGeoPoint targetPoint) {
	/*该条路径上是否会被敌人阻挡*/
	double k_m = WantToLessShoot;
	double opp2LineDist = TheMinDistBetweenTheOppAndTheLine(pVision, startPoint, targetPoint);
	if (opp2LineDist < k_m * Param::Vehicle::V2::PLAYER_SIZE)return true;
	return false;
}

bool CAdvance::IsOurNearHere(const CVisionModule* pVision, int supportIndex) {
	int supporter = TaskMediator::Instance()->supporter(supportIndex);
	if (supporter != 0 && pVision->OurPlayer(supporter).Pos().dist(SupportPoint[supportIndex]) < 20)
		return true;
	return false;
}
bool CAdvance::IsOurNearHere(const CVisionModule* pVision, CGeoPoint checkPoint, const int vecNumber) {
	const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
	const BallVisionT& ball = pVision->Ball();

	for (int i = 0; i < Param::Field::MAX_PLAYER; i++) {
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
int CAdvance::TheirRobotInBreakArea(const CVisionModule* pVision, const int vecNumber) {
	int cnt = 0, n = 0;
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	while (n <= Param::Field::MAX_PLAYER) {
		if (!pVision->TheirPlayer(n).Valid()) { n++; continue; }
		const PlayerVisionT& opp = pVision->TheirPlayer(n);
		double x = opp.X(), y = opp.Y();
		cnt += (abs(y) < 180 && (x > 250 || x > me.Pos().x()));
		n++;
	}
	return cnt;
}
bool CAdvance::Me2OppTooclose(const CVisionModule* pVision, const int vecNumber) { //是否太近了
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
	const BallVisionT& ball = pVision->Ball();
	CVector me2Ball = ball.Pos() - me.Pos();
	CVector me2Opp = opp.Pos() - me.Pos();

	/*
	char me2Ball1[100];
	char me2Opp1[100];
	char dir1[100];
	sprintf(me2Ball1, "%f", abs(me2Ball.mod()));
	sprintf(me2Opp1, "%f", abs(me2Opp.mod()));
	sprintf(dir1, "%f", (me2Ball.dir() - me2Opp.dir()) / Param::Math::PI * 180);
	GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(-320, -350), me2Ball1, COLOR_YELLOW);
	GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(-320, -300), me2Opp1, COLOR_YELLOW);
	GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(-320, -250), dir1, COLOR_YELLOW);
	*/

	//changed by lsf
	if (fabs(me2Opp.mod()) <= 50)
		return true;

	if ((fabs(me2Ball.mod()) * 1.5 > fabs(me2Opp.mod()) && (me2Ball.dir() - me2Opp.dir() < Param::Math::PI / 3))) {
		if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(450, 450), "TOO CLOSE with ball", COLOR_ORANGE);
		return true;
	}
	return false;
}
bool CAdvance::isDirOK(const CVisionModule* pVision, int vecNumber, double targetDir, int IsShoot) {
	if (IsShoot) {
		return canScore(pVision, vecNumber, OBSTACLE_RADIUS, pVision->OurPlayer(vecNumber).Dir());
	}
	double ShootPrecision = SHOOT_PRECISION;
	//double offset = 0.05;
	const BallVisionT& ball = pVision->Ball();
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
	double myDir = me.Dir();
	CVector opp2ball = ball.Pos() - opp.Pos();
	CVector ball2goal = theirCenter - ball.Pos();

	if (fabs(me.Dir() - targetDir) < ShootPrecision * 3) return true;

	else return false;
	/*
	cout << fabs(me.Dir() - targetDir) << ' ' << abs(targetDir - last_target_dir) <<' '<<SHOOT_PRECISION << endl;
	//if (!ShootOrPass) ShootPrecision = ShootPrecision * 0.8;
	/*if (myDir - targetDir > 0)targetDir -= offset;
	else targetDir += offset;
			是峪狍镝  妄荇租咛

	last_target_dir = targetDir;

	if (ShootOrPass) {
		CGeoLine start2Target = CGeoLine(me.Pos(), myDir);
		CGeoLineLineIntersection Intersection = CGeoLineLineIntersection(start2Target, GOATLINE);
		if (abs(Intersection.IntersectPoint().y()) > 60) return false;
	}
	/*交点必须位于球门里面

	if (abs(targetDir - last_target_dir) > 3.0 * SHOOT_PRECISION) {
		last_dir_deviation = 100;  //重置角度差
	}
	if (Me2OppTooclose(pVision, vecNumber)) {
		/*太近了 快射
		if (abs(myDir - targetDir) < 2.0 * SHOOT_PRECISION) {
			last_dir_deviation = 100;
			return true;
		}
	}
	if ((ShootOrPass && ball2goal.mod() < 250) || (opp2ball.mod() < 200)) {
		/*如果现在是射门且距离球门足够近 不需要过多的调整
		  如果敌人离我比较近了 再调整就无法出球了
		if (abs(myDir - targetDir) < 1.5 * SHOOT_PRECISION) {
			last_dir_deviation = 100;
			return true;
		}
	}
	if (abs(myDir - targetDir) > 1.5 * SHOOT_PRECISION) {
		/*如果角度过大 应当为false
		last_dir_deviation = myDir - targetDir;
		return false;
	}
	else if ((abs(myDir - targetDir) > abs(last_dir_deviation) || (myDir - targetDir) * last_dir_deviation <= 0)){
		/*如果相比上次角度并没有得到有效调整
		if (abs(myDir - targetDir) < 1.25 * SHOOT_PRECISION) {
			/*误差相对而言可以接受了
			last_dir_deviation = 100;
			return true;
		}
	}
	else if (abs(myDir - targetDir) < SHOOT_PRECISION) {
		/*尽管仍然在调整 但是可以在误差允许的范围内射门
		last_dir_deviation = 100;
		return true;
	}
	last_dir_deviation = myDir - targetDir;
	return false;
*/
}

/*
*		The Data List
		const double PITCH_LENGTH = OParamManager::Instance()->value(field + "/field_length", 12000).toDouble() * 0.1; //1200; // 场地长
		const double PITCH_WIDTH = OParamManager::Instance()->value(field + "/field_width", 9000).toDouble() * 0.1; //900; // 场地宽
		const double PITCH_MARGIN = OParamManager::Instance()->value(field + "/field_line_width", 10).toDouble() * 0.1; //1; // 场地的边界宽度
		const double CENTER_CIRCLE_R = OParamManager::Instance()->value(field + "/center_radius", 500).toDouble() * 0.1 * 2; //100; // 中圈半径?直径
		const double PENALTY_AREA_WIDTH = OParamManager::Instance()->value(field + "/penalty_width", 2400).toDouble() * 0.1; //240; // 禁区宽度
		const double PENALTY_AREA_DEPTH = OParamManager::Instance()->value(field + "/penalty_depth", 1200).toDouble() * 0.1; //120; // 禁区深度

		DefenceArea = 0,
		SideArea,
		CornerArea,
		KICKArea,
		CanNOTBreakArea,
		CenterArea

		*/

int CAdvance::InWhichArea(const CVisionModule* pVision, int vecNumber) {
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	const double MeX = me.X(), MeY = me.Y();
	const double DefenceLineX = 0.0 * Param::Field::PITCH_LENGTH / 900;
	const double SideLineY = 235.0 * Param::Field::PITCH_WIDTH / 600;
	const double CornerLineX = 350.0 * Param::Field::PITCH_LENGTH / 900;
	const double KICKDist = KICK_DIST * Param::Field::PITCH_LENGTH / 900;
	const double CanNotBreakAreaX = 425.0 * Param::Field::PITCH_LENGTH / 900;
	const double CanNotBreakAreaY = 260.0 * Param::Field::PITCH_WIDTH / 600;
	int NowArea = 0;
	if (MeX < DefenceLineX) {
		NowArea = DefenceArea;
		GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(525, -345), "DefenceArea", COLOR_CYAN);
	}
	else if (MeX > CanNotBreakAreaX || fabs(MeY) > CanNotBreakAreaY) {
		NowArea = CanNOTBreakArea;
		GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(525, -345), "CanNOTBreakArea", COLOR_CYAN);
	}
	else if (fabs(MeX) > CornerLineX) {
		GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(525, -345), "CornerArea", COLOR_CYAN);
		NowArea = CornerArea;
	}
	else if (fabs(MeY) > SideLineY) {
		NowArea = SideArea;
		GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(525, -345), "SideArea", COLOR_CYAN);
	}
	else if ((me.Pos() - theirCenter).mod() < KICKDist) {
		NowArea = KICKArea;
		GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(525, -345), "KICKArea", COLOR_CYAN);
	}
	else {
		NowArea = CenterArea;
		GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(525, -345), "CenterArea", COLOR_CYAN);
	}
	return NowArea;
}
bool CAdvance::isInBreakArea(const CVisionModule* pVision, int vecNumber) {
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	if (fabs(me.Y()) < 180 && me.X() < 540 && me.X() > 210)return true;
	return false;
}
bool CAdvance::isInTheCornerArea(const CVisionModule* pVision, int vecNumber) {
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	if (me.X() > 425)return true;
	if (me.X() > 240 && fabs(me.Y()) > 210)return true;
	return false;
}
bool CAdvance::MeIsInTheSide(const CVisionModule* pVision, int vecNumber) {
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	return fabs(me.Y()) > 185 && me.X() < 240;
}
bool CAdvance::JudgeIsMeSupport(const CVisionModule* pVision, int vecNumber) {
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	return fabs(me.Y()) < 200;
}
bool CAdvance::JudgePassMeIsBeBlocked(const CVisionModule* pVision, int vecNumber) {
	const BallVisionT& ball = pVision->Ball();
	const double BallVelDir = ball.Vel().dir();
	if (LastPassDirToJudge < -100)return true;
	if (fabs(Utils::Normalize((LastPassDirToJudge - BallVelDir))) < Param::Math::PI / 2)return false;
	return true;
}
bool CAdvance::AdJudgeBreakCanDo(const CVisionModule* pVision, int vecNumber, CGeoPoint TargetPoint) {
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	double MeDir = me.Dir();
	double Me2Target = (TargetPoint - me.Pos()).dir();
	if (fabs(Utils::Normalize(MeDir - Me2Target)) > Param::Math::PI / 2)return false;
	return true;
}
/**********************************************************
	* Description: 状态切换判定类函数，用于状态转化之间的判断
	* Author: 谭宇宏
	* Created Date: 2022/10/10
***********************************************************/
bool CAdvance::tendToShoot(const CVisionModule* pVision, int vecNumber) {
	/*判断现在能否射门*/
	// NEEDMODIFY
	int n = 0;
	int best_n = 0;
	const BallVisionT& ball = pVision->Ball();
	bool shootBlocked = false;
	double kickDir = KickDirection::Instance()->getPointShootDir(pVision, pVision->OurPlayer(vecNumber).Pos());
	if (fabs(kickDir - 1000.0) < 10) return false;
	/*修复tendToShoot与getPointShootDir判断不兼容的问题*/
	CGeoLine ball2ourGoal = CGeoLine(ball.Pos(), kickDir);
	CGeoPoint projectionPoint;
	double k_m = WantToLessShoot;
	double opp2LineDist = 1000;
	while (n <= Param::Field::MAX_PLAYER) {
		if (!pVision->TheirPlayer(n).Valid()) { n++; continue; }
		projectionPoint = ball2ourGoal.projection(pVision->TheirPlayer(n).Pos());
		if (opp2LineDist > (projectionPoint - pVision->TheirPlayer(n).Pos()).mod() && projectionPoint.x() >= ball.X()) {
			opp2LineDist = (projectionPoint - pVision->TheirPlayer(n).Pos()).mod();
			best_n = n;
			if (opp2LineDist < k_m * Param::Vehicle::V2::PLAYER_SIZE) {
				shootBlocked = true;
				break;
			}
		}
		n++;
	}
	if (shootBlocked) return false;
	else return true;
}

int CAdvance::CanSupportKick(const CVisionModule* pVision, int vecNumber) {
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	double SupportShootDir = 0, MeToSupportDist = 0, SupportToGoal = 0;
	for (int i = 0; i < NumberOfSupport; ++i) {
		if (!IsOurNearHere(pVision, SupportPoint[i], vecNumber)) continue;
		if (SupportPoint[i].x() < me.Pos().x() && MeIsInWhichArea != CornerArea) continue;
		double PasstoSupportDir = (SupportPoint[i] - me.Pos()).dir();
		//if (!isDirOK(pVision, vecNumber, PasstoSupportDir, 0)) continue;
		return 1;
		/*
		MeToSupportDist = (me.Pos() - SupportPoint[i]).mod();
		SupportToGoal = (CGeoPoint(Param::Field::PITCH_LENGTH / 2.0, 0) - SupportPoint[i]).mod();
		if (MeToSupportDist < CanPassToWingDist && SupportToGoal < CanWingShootDist)
			return 1;
		*/
	}
	return 0;
}
int CAdvance::toChipOrToFlat(const CVisionModule* pVision, int vecNumber, CGeoPoint TargetPoint) {
	// 0chip 1flat
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);

	if (isTheLineBlocked(pVision, me.Pos(), TargetPoint) && me.Pos().dist(TargetPoint) <= ParamManager::Instance()->maxChipDist)return 0;

	return 1;
}

/**********************************************************
	* Description: 防守类函数，仅限于GET中使用
	* Author: 谭宇宏
	* Created Date: 2022/10/10
***********************************************************/
bool CAdvance::isOppFaceOurDoor(const CVisionModule* pVision, double angle) {
	//判断是否球距离opp很近，opp距离我们球门很近
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
	* Description: 生成类函数，具有具体实义
	* Author: 谭宇宏
	* Created Date: 2022/10/10
***********************************************************/
PassDirOrPos CAdvance::PassDirInside(const CVisionModule* pVision, int vecNumber) {

	bool IsTestPass = 0;
	if (IsTestPass) {
		for (int i = 0; i < 6; ++i) {
			const PlayerVisionT& friends = pVision->OurPlayer(i);
			if (pVision->OurPlayer(i).Valid() && i != vecNumber) {
				PassDirOrPos ReturnValue;
				ReturnValue.dir = (friends.Pos() - pVision->OurPlayer(vecNumber).Pos()).dir();
				ReturnValue.pos = friends.Pos();
				//LastPassPoint = NowShootNumber;//保存当前状态
				return ReturnValue;
			}
		}
	}
	//参数定义部分

	PassDirOrPos ReturnValue;
	//返回值
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	//当前持球车号
	bool isOurNearPoint[9] = { 0 }/*, isBlockPoint[9] = { 0 }*/, isCanUse[9] = { 0 }, OneOfUsCanShoot = 0;
	/*依次为：我是否有人在传球点附近，敌人是否阻挡该传球路径，该点能否使用，我们有没有点满足要求*/
	int TheNumberOfCanShootPoint = 0, NowShootNumber = 0, TheidxOfCanShootPoint[9] = { 0 };
	/*依次为：满足要求的点的数量TheNumberOfCanShootPoint，持久化点的number，能够shoot的点的idx*/
	double ShootDir[9] = { 0 }, ChangeDir[9] = { 0 }, DistToPoint[9] = { 0 }, DistOppToTheLine[9] = { 0 }, FinalDir = 0;
	/*依次为：shootdir，需要转变方向的Dir，最终角度Dir*/

	/**********************************************************************************************************************/

	//判定支撑点是否可用

	for (int i = 0; i < NumberOfSupport; ++i) {
		isOurNearPoint[i] = IsOurNearHere(pVision, i); // IsOurNearHere(pVision, SupportPoint[i], vecNumber);						//我方有人在旁边
		//isBlockPoint[i] = isTheLineBlocked(pVision, me.Pos(), SupportPoint[i]);
		//DistOppToTheLine[i] = TheMinDistBetweenTheOppAndTheLine(pVision, me.Pos(), SupportPoint[i]);//对手队员离传球线最短距离
		ShootDir[i] = KickDirection::Instance()->getPointShootDir(pVision, SupportPoint[i]);		//射门角度
		//DistToPoint[i] = (SupportPoint[i] - me.Pos()).mod();										//支撑点到“我”的距离
		ChangeDir[i] = fabs(Utils::Normalize(me.Dir() - (SupportPoint[i] - me.Pos()).dir()));								//传球到点“我”需要改变的角度

		//最终的判定条件
		//if ()isCanUse[i] = isOurNearPoint[i] && (ShootDir[i] != 1000);
		//else isCanUse[i] = isOurNearPoint[i] && (ShootDir[i] != 1000) && (SupportPoint[i].x() > me.X());
		//当前点可以射门的条件：我方有人在旁边，//没有阻挡//，射门可以，向前传球
		if (isOurNearPoint[i])
		{
			OneOfUsCanShoot = 1;
			TheidxOfCanShootPoint[TheNumberOfCanShootPoint++] = i;
		}
	}

	/**********************************************************************************************************************/

	/*//保持系统稳定性，返回上一次决策的情况

	ReturnValue.dir = (SupportPoint[LastPassPoint] - me.Pos()).dir();
	ReturnValue.pos = SupportPoint[LastPassPoint];
	if (SupportPoint[LastPassPoint].x() > me.X()	&& //向前传球
		isOurNearPoint[LastPassPoint]				&& //我方旁边有人
		(abs(me.Dir() - (SupportPoint[LastPassPoint] - me.Pos()).dir()) < 0.5 * Param::Math::PI / SHOOT_PRECISION)) //“我”传球需要转动的角度合适
	{
		return ReturnValue;
	}*/

	/**********************************************************************************************************************/


	/*
	if (OneOfUsCanShoot) {
		//如果其中有一个满足大前提，就只考虑这些可以满足要求的点
		IsMeSupport = JudgeIsMeSupport(pVision, vecNumber);
		if (IsMeSupport) { // 着重向中心传球
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
			if (isCanUse[i] && SupportPoint[i].x() > -330)
				TheidxOfCanShootPoint[TheNumberOfCanShootPoint++] = i;

	}
	else {
		//否则就有人的地方全都可以传 因为现在已经在决定传球方向了
		IsMeSupport = JudgeIsMeSupport(pVision, vecNumber);
		if (IsMeSupport) { // 着重向中心传球
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
			if(isOurNearPoint[i] && SupportPoint[i].x() > -330)
				TheidxOfCanShootPoint[TheNumberOfCanShootPoint++] = i;
	}
	*/
	//choose the one to pass		
	/**********************************************************************************************************************/
	if (!OneOfUsCanShoot)
	{
		NowShootNumber = 1;
	}
	else {
		double final_score = -1, max_value = -100;
		double near_para = -10, shoot_para = 7, change_para = 12;
		int Maxidx = -1;
		for (int i = 0; i < TheNumberOfCanShootPoint; ++i) {
			int NowIdx = TheidxOfCanShootPoint[i];
			double shoot_dist = sqrt((SupportPoint[i].x() - Param::Field::PITCH_LENGTH / 2) * (SupportPoint[i].x() - Param::Field::PITCH_LENGTH / 2) + SupportPoint[i].y() * SupportPoint[i].y());
			final_score = near_para * Me2OppTooclose(pVision, vecNumber) + shoot_para * shoot_dist + change_para * ChangeDir[i];
			double PasstoSupportDir = -(SupportPoint[i] - me.Pos()).dir();
			if (!isDirOK(pVision, vecNumber, PasstoSupportDir, 0)) continue;
			if (final_score > max_value) Maxidx = NowIdx;
		}
		if (Maxidx < 0)
			Maxidx = 0;
		NowShootNumber = Maxidx;
	}
	//char shootnumber[100];
	//sprintf(shootnumber, "%f", NowShootNumber);
	//GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -175), shootnumber, COLOR_YELLOW);
	/**********************************************************************************************************************/
	/*
	if (!OneOfUsCanShoot)
	{
		NowShootNumber = 0;
	}
	else { //按照需要转的角度进行排序

		double NowValue = -1, MinValue = 1e9, threshValue = LARGE_ADJUST_ANGLE * Param::Math::PI / 180.0;
		int Maxidx = -1;

		for (int i = 0; i < TheNumberOfCanShootPoint; ++i) {
			int NowIdx = TheidxOfCanShootPoint[i];
			NowValue = ChangeDir[NowIdx];
			if (NowValue < threshValue) {
				Maxidx = NowIdx;
				break;
			}//角度合适立刻返回
			if (NowValue < MinValue)
				Maxidx = NowIdx, MinValue = NowValue;
		}
		if (Maxidx < 0)
			Maxidx = 0;
		NowShootNumber = Maxidx;
	}
	*/
	/**********************************************************************************************************************/

	//经过两轮筛选得到返回值

	ReturnValue.dir = (SupportPoint[NowShootNumber] - me.Pos()).dir();
	ReturnValue.pos = SupportPoint[NowShootNumber];
	//LastPassPoint = NowShootNumber;//保存当前状态
	return ReturnValue;

}

double CAdvance::PassDir(const CVisionModule* pVision, int vecNumber) {
	PassDirOrPos ReturnValue = PassDirInside(pVision, vecNumber);
	return ReturnValue.dir;
}
double CAdvance::TheMinDistBetweenTheOppAndTheLine(const CVisionModule* pVision, CGeoPoint startPoint, CGeoPoint targetPoint) {
	/*该条路径上是否会被敌人阻挡*/
	int n = 0;
	const BallVisionT& ball = pVision->Ball();
	double passDir = (targetPoint - startPoint).dir();
	CGeoLine start2Target = CGeoLine(startPoint, passDir);
	CGeoPoint projectionPoint;
	double opp2LineDist = 1000;
	while (n <= Param::Field::MAX_PLAYER) {
		if (!pVision->TheirPlayer(n).Valid()) { n++; continue; }
		projectionPoint = start2Target.projection(pVision->TheirPlayer(n).Pos());
		double r = (projectionPoint - startPoint).x() * (projectionPoint - targetPoint).x() + (projectionPoint - startPoint).y() * (projectionPoint - targetPoint).y();
		if (opp2LineDist > (projectionPoint - pVision->TheirPlayer(n).Pos()).mod() && r < 0) { // projectionPoint.x() < Param::Field::PITCH_LENGTH / 2.0 && projectionPoint.x() > startPoint.x()) {
			opp2LineDist = (projectionPoint - pVision->TheirPlayer(n).Pos()).mod();
		}
		n++;
	}
	return opp2LineDist;
}
CGeoPoint CAdvance::GenerateBreakShootPoint(const CVisionModule* pVision, int vecNumber) {
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	CGeoPoint ShootPoint = KickDirection::Instance()->GetTheShootPoint(pVision, me.Pos());
	ShootPoint.setY(ShootPoint.y() - me.VelY() * 1.5);
	if (me.Y() < -120)ShootPoint.setY(ShootPoint.y() + me.VelX() * 1 - me.Y() * 0.25);
	else if (me.Y() > 120)ShootPoint.setY(ShootPoint.y() - me.VelX() * 1 - me.Y() * 0.25);
	//double MeVel = me.VelY()
	if (ShootPoint.y() < -60)ShootPoint.setY(-60);
	if (ShootPoint.y() > 60)ShootPoint.setY(60);

	return ShootPoint;
}

CGeoPoint CAdvance::GenerateBreakPassPoint(const CVisionModule* pVision, int vecNumber) {
	PassDirOrPos ReturnValue = PassDirInside(pVision, vecNumber);
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	CGeoPoint ShootPoint = ReturnValue.pos;
	ShootPoint.setY(ShootPoint.y() - me.VelY() * 0.75);
	if (me.Y() < -90)ShootPoint.setY(ShootPoint.y() + me.VelX() * 0.4);
	else if (me.Y() > 90)ShootPoint.setY(ShootPoint.y() - me.VelX() * 0.4);
	return ShootPoint;
}

double CAdvance::GetFPassPower(CGeoPoint StartPoint, CGeoPoint targetPoint) {
	double dist = (StartPoint - targetPoint).mod() - Param::Vehicle::V2::PLAYER_FRONT_TO_CENTER;
	//cout << "dist::"<<' '<<dist << endl;
	double passPower = sqrt(powf(ParamManager::Instance()->FASTEST_RECEIVE_VEL, 2) + 2 * ParamManager::Instance()->BALL_DEC * dist) * ADV_FPASSPOWER_Alpha;
	// std::cout << "passPower" << passPower << std::endl;

	//cout << "passpower::" << ' ' << passPower << endl;
	return min(passPower, (double)Param::Rule::MAX_BALL_SPEED - 10);
	// return max(min(650.0, ADV_FPASSPOWER_Alpha* dist ), 200.0);
}
double CAdvance::GetCPassPower(CGeoPoint StartPoint, CGeoPoint targetPoint) {
	double dist = ((StartPoint - targetPoint).mod() - 9.0 * Param::Vehicle::V2::PLAYER_SIZE) * ADV_CPASSPOWER_Alpha;
	return min(ParamManager::Instance()->maxChipDist, dist);
	// return min(460.0, ADV_CPASSPOWER_Alpha * dist);
}

/*
double CAdvance::generateNormalPushDir(const CVisionModule* pVision, const int vecNumber) {
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
	const BallVisionT& ball = pVision->Ball();
	double faceDir = 0.0;
	if (!opp.Valid()) {
		return KickDirection::Instance()->getPointShootDir(pVision, ball.Pos());
	}
	if (abs(ball.Pos().y()) > Param::Field::PITCH_WIDTH / 2 * 0.70) {
		//cout << "here here here" << endl;
		faceDir = opp.Dir() + Param::Math::PI;
		return faceDir;
	}
	else {
		double kickDir = KickDirection::Instance()->getPointShootDir(pVision, ball.Pos());
		faceDir = fabs(Utils::Normalize(kickDir + opp.Dir()));
		if(me.Y() > 0)faceDir *= -1.0;
		return faceDir;
	}
}*/
CGeoPoint CAdvance::generateNormalPushPoint(const CVisionModule* pVision, const int vecNumber) {

	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
	const BallVisionT& ball = pVision->Ball();
	double faceDir = 0.0;

	const double DirEpsilon = Param::Math::PI / 180 * 5;
	const double VectorDist = 200;
	const double ThresholdForOpp = 60;

	return me.Pos() + Utils::Polar2Vector(VectorDist, (theirCenter - me.Pos()).dir());

	CGeoPoint FinalTarget = CGeoPoint(-999, -999);
	double Dist = -1e9;
	for (int step = -8; step <= 8; ++step) {
		const CGeoPoint target = me.Pos() + Utils::Polar2Vector(VectorDist, Param::Math::PI / 180 * 5 * step);
		const double SingleDist = TheMinDistBetweenTheOppAndTheLine(pVision, me.Pos(), target);
		if (SingleDist > Dist) FinalTarget = target, Dist = SingleDist;
		//GDebugEngine::Instance()->gui_debug_x(target, COLOR_BLUE);
	}
	if (Dist > ThresholdForOpp) {
		Dist = 1e9;
		for (int step = -8; step <= 8; ++step) {
			const CGeoPoint target = me.Pos() + Utils::Polar2Vector(VectorDist, Param::Math::PI / 180 * 5 * step);
			if (TheMinDistBetweenTheOppAndTheLine(pVision, me.Pos(), target) < 60)continue;
			const double SingleDist = (target - theirCenter).mod();
			if (SingleDist < Dist) FinalTarget = target, Dist = SingleDist;
			//GDebugEngine::Instance()->gui_debug_x(target, COLOR_BLUE);
		}
	}
	//GDebugEngine::Instance()->gui_debug_x(FinalTarget, COLOR_BLUE);
	FinalTarget = me.Pos() + Utils::Polar2Vector(VectorDist, (theirCenter - me.Pos()).dir());
	return FinalTarget;
}
double CAdvance::generateGetballDir(const CVisionModule* pVision, const int vecNumber) {
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
	const BallVisionT& ball = pVision->Ball();
	const CVector me2ball = me.Pos() - ball.Pos();
	double faceDir = 0.0;
	if (ball.Vel().mod() > 20) {
		faceDir = Utils::Normalize(Param::Math::PI + ball.Vel().dir());
	}
	else {
		faceDir = (ball.Pos() - me.Pos()).dir();
	}
	return faceDir;
}
int CAdvance::GenerateStateOfFoulTrouble(const CVisionModule* pVision, const int vecNumber) {
	if (tendToShoot(pVision, vecNumber)) return KICK;
	else return KICK;
}


bool CAdvance::OppIsNearThanMe(const CVisionModule* pVision, const int vecNumber) {
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
	const BallVisionT& ball = pVision->Ball();
	CVector me2Ball = ball.Pos() - me.Pos();
	CVector Ball2Opp = opp.Pos() - ball.Pos();

	const double threshold = 70;
	//cout << "ehreeeeeeee__"<<' '<<checkOppHasBall(pVision) << ' ' << (me2Ball.mod() > Ball2Opp.mod() + 10) << ' ' << (me2Ball.theta(Ball2Opp) > Utils::Normalize(Param::Math::PI / 180 * 90)) << endl;
	if (checkOppHasBall(pVision))return true;
	return false;
}

bool CAdvance::OppIsFarThanMe(const CVisionModule* pVision, const int vecNumber) {
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
	const BallVisionT& ball = pVision->Ball();
	CVector me2Ball = ball.Pos() - me.Pos();
	CVector Ball2Opp = opp.Pos() - ball.Pos();

	const double threshold = 70;
	if (me2Ball.mod() < Ball2Opp.mod())return true;
	return false;
}

double CAdvance::generateOppIsNearThanMeDir(const CVisionModule* pVision, const int vecNumber) {
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
	const BallVisionT& ball = pVision->Ball();
	//return Utils::Normalize((opp.Pos() - ball.Pos()).dir() + 20 * Param::Math::PI / 180);
	return Utils::Normalize((opp.Pos() - ball.Pos()).dir());// +20 * Param::Math::PI / 180);
}
double CAdvance::generateOppIsFarThanMeDir(const CVisionModule* pVision, const int vecNumber) {
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
	const BallVisionT& ball = pVision->Ball();
	//return Utils::Normalize((opp.Pos() - ball.Pos()).dir() + 20 * Param::Math::PI / 180);
	return Utils::Normalize((ball.Pos() - opp.Pos()).dir());// +20 * Param::Math::PI / 180);
}

bool CAdvance::canScore(const CVisionModule* pVision, const int vecNumber, const double radius, const double dir) {
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);

	bool flag = true;
	double x1 = me.X(), y1 = me.Y(), theta = dir;
	if (Param::Field::MAX_PLAYER == 0)
	{
		double projection = y1 + tan(theta) * (Param::Field::PITCH_LENGTH / 2 - x1);
		if (fabs(projection) + 2 > (Param::Field::GOAL_WIDTH) / 2) {
			flag = false;
		}

	}

	double projection = y1 + tan(theta) * (Param::Field::PITCH_LENGTH / 2 - x1);
	if (fabs(projection) + 5 > (Param::Field::GOAL_WIDTH) / 2) {
		flag = false;
		return false;
	}

	for (int i = 0; i < Param::Field::MAX_PLAYER; i++) {
		if (!pVision->TheirPlayer(i).Valid()) continue;
		auto enemy = pVision->TheirPlayer(i);
		double x = enemy.X(), y = enemy.Y();
		double r = fabs(y - y1 - tan(theta) * x + tan(theta) * x1) / sqrt(1 + tan(theta) * tan(theta));

		if (r < radius) {
			flag = false;
			break;
		}
	}
	/*
	char projection[100];
	sprintf(projection, "%f", projection);
	GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(160, 230), projection, COLOR_YELLOW);
	*/
	CGeoLine ShootLine = CGeoLine(me.Pos(), me.Dir());
	CGeoLine GoalLine = CGeoLine(theirLeft, theirRight);
	CGeoSegment Goal = CGeoSegment(theirLeft, theirRight);
	CGeoLineLineIntersection LineIntersection = CGeoLineLineIntersection(GoalLine, ShootLine);
	if (LineIntersection.Intersectant() == 0)return false;
	CGeoPoint Intersection = LineIntersection.IntersectPoint();
	if (Goal.IsPointOnLineOnSegment(Intersection) == 0) flag = false;
	/*
	GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -450), ("CanScore:" + to_string(flag)).c_str(), COLOR_YELLOW);
	*/
	return flag;

}

bool CAdvance::WeNeedBlockTheBall(const CVisionModule* pVision, const int vecNumber) {
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	const PlayerVisionT& opp = pVision->OurPlayer(opponentID);
	if (fabs(Utils::Normalize((opp.Dir() - (opp.Pos() - ourGoal).dir()))) < 30 * Param::Math::PI / 180) {
		return true;
	}
	else return false;
}


int CAdvance::opp_ahead(const CVisionModule* pVision, const int vecNumber) {
	int cnt = 0;
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	double x1 = me.X(), y1 = me.Y();
	for (int i = 0; i < Param::Field::MAX_PLAYER; i++) {
		if (!pVision->TheirPlayer(i).Valid()) continue;
		auto enemy = pVision->TheirPlayer(i);
		double x = enemy.X(), y = enemy.Y();
		if (x > x1 && fabs(y) < Param::Field::PITCH_WIDTH * 4 / 5)
			cnt++;
	}
	return cnt;
}




int CAdvance::GenerateNextState(const CVisionModule* pVision, const int vecNumber) {
	// 需要完全拿住球
	/*DefenceArea = 0,
	SideArea,
	CenterArea
	CornerArea,
	KICKArea,
	CanNOTBreakArea,*/
	/*如果我和球门之间的距离小于KICK_DIST，考虑顺序为 shoot->break->pass */
	if ((NowIsShoot == 1)) {
		return KICK;
	}
	else if ((NowIsShoot == 2) && MeIsInWhichArea != CanNOTBreakArea) {
		return BREAKSHOOT;
	} // 持久化
	else if (MeIsInWhichArea == DefenceArea) { // 在后场防守区域
		if ((!IHaveSupport) && Me2OppTooclose(pVision, vecNumber))
			return JUSTCHIPPASS;
		else if ((!Me2OppTooclose(pVision, vecNumber))) {
			// 无人防守状态 或 不存在支援车
			//return PUSHOUT;
			return BREAKSHOOT;
		}
		else { return JUSTCHIPPASS; }// 否则向前挑传
	}
	else if (MeIsInWhichArea == SideArea) { // 在双边区域
		if (IHaveSupport) { //pass first
			return PASS;
		}
		else if (Me2OppTooclose(pVision, vecNumber) && opp_ahead(pVision, vecNumber) > 3) { // 存在防守
			return BREAKPASS;
		}
		else if (Me2OppTooclose(pVision, vecNumber)) {
			return BREAKSHOOT;
		}  // 首先判断射门
		else {	// 不存在防守
			return PUSHOUT;
		}	
	}
	else if (MeIsInWhichArea == CenterArea) { // 在前中场
		if (tendToShoot(pVision, vecNumber)) {
			return BREAKSHOOT;
		}  // 首先判断射门
		else if (!IHaveSupport) {  // 不存在支援车
			//return PUSHOUT;
			//return KICK;	//changed
			//judge the opp
			if (opp_ahead(pVision, vecNumber) <= 3)
				_state = BREAKSHOOT;
			else
				_state = PUSHOUT;
		}
		else if (Me2OppTooclose(pVision, vecNumber)) { // 存在防守
			return BREAKPASS;
		}
		else {	// 不存在防守
			return PUSHOUT;
		}
	}	
	else if (MeIsInWhichArea == CornerArea) { //在角球区
		if (IHaveSupport) { 
		}
		else
		if (IHaveSupport) { //pass first
			return PASS;
		}
		else if (Me2OppTooclose(pVision, vecNumber) && opp_ahead(pVision, vecNumber) > 3) { // 存在防守
			return BREAKPASS;
		}
		else if (Me2OppTooclose(pVision, vecNumber)) {
			return BREAKSHOOT;
		} 
		else {	
			return PUSHOUT;
		}	
	}	
	else if (MeIsInWhichArea == CanNOTBreakArea) {  // 不能break
		if (IHaveSupport) { //pass first
			return PASS;
		}
		else if (Me2OppTooclose(pVision, vecNumber) && opp_ahead(pVision, vecNumber) > 3) { // 存在防守
			return BREAKPASS;
		}
		else {
			return BREAKSHOOT;
		}  
	}	
	else { // Kick区域
		if (tendToShoot(pVision, vecNumber)) {
			return KICK;
		}  // 首先判断射门
		else if (MeIsInWhichArea != CanNOTBreakArea) {  // 不存在支援车 //
			return BREAKSHOOT;
		}
		else {
			return PASS;
		}
	}
	return KICK;
}

int CAdvance::CanWeUseChaseBecauseOfGetBallV3(const CVisionModule* pVision, const int vecNumber) {
	const BallVisionT& ball = pVision->Ball();
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
	const CVector opp2ball = ball.Pos() - opp.Pos();
	double deltaTheta1 = fabs(Utils::Normalize(ball.Vel().dir() - (ball.Pos() - me.Pos()).dir()));
	// 球速方向与我面向球的方向夹角小于20度
	double deltaTheta2 = fabs(Utils::Normalize(KickorPassDir - (ball.Pos() - me.Pos()).dir()));
	// 当前射门方向与我面向球的方向夹角小于20度
	bool VelModFlag = ball.Vel().mod() < 80;
	// 球速小于80
	bool VelDirFlag = (ball.Vel().mod() < 10) || (deltaTheta1 < 20.0 / 180 * Param::Math::PI);// || deltaTheta1 > 160.0 / 180 * Param::Math::PI);
	// 球速小于10 或者 球速方向与我面向球的方向夹角小于20度
	bool OppFlag = opp2ball.mod() > 40 || opp2ball.mod() > (me.Pos() - ball.Pos()).mod() * 1.5;
	// 敌人距离球大于40 或者敌人距离球比我距离球的1.5倍更大
	bool FinalDirFlag = (deltaTheta2 < 20.0 / 180 * Param::Math::PI);// || deltaTheta2 > 160.0 / 180 * Param::Math::PI);
	// 当前射门方向与我面向球的方向夹角小于20度

	if ((theirCenter - me.Pos()).mod() < KICK_DIST * 0.65 && VelModFlag && VelDirFlag) return true;
	// 球门线前，不用考虑opp
	return VelModFlag && VelDirFlag && OppFlag && FinalDirFlag;
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

