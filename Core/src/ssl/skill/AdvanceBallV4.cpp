/************************************************************************/
/*		modified by tyh
		22.7.10
		hope : use chaseKickV2, not V1
		Write comments in English to prevent garble.

		Problem:
			1. has 'makeItNoneTrajGetBall' completed its kicking duties?
			   or just getball, need to detect.
			2. the second GOTO merge into the first
*/
/************************************************************************/
#include "AdvanceBallV4.h"
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

namespace {
	int KICKPOWER = 550;
	int CHIPPOWER = 240;
	int ADV_FPASSPOWER = 480;//ADV_FPASSPOWER
	int ADV_CPASSPOWER = 230;//ADV_CPASSPOWER
	int RELIEF_POWER = 400;
	int BACK_POWER = 300;
	int TEND_TO_SHOOT_FLAG = 25;
	double SHOOT_PRECISION = 15;
	double SHOOT_BLOCKED_RANGE = 3;   //  可以放到ini里面去
	double PASS_BLOCKED_RANGE = 7;
	//	bool DEBUG_ENGINE = false;     //  output information of debug or not
	bool DEBUG_ENGINE = true;
	bool WEAK_OPP = false;     //
	bool USE_DRIBBLE_TURN = false;

	int meHasBallMaxCnt = 1;
	int maxNormalPushCnt = 100;
	double CHIP_ANGLE = 15;
	//	int RELIEF_BIAS = 180;// the bias of relief target
	double FLAT_DIST = 300;
	double PULL_DIST = 450;
	double KICK_DIST = 600;  // Kick Dist  可以放到ini里面去
	double RELIEF_DIST = 280;
	double NORMALPUSH_DIST = 60;
	double w = Param::Field::GOAL_WIDTH;
	double goal[9] = { -0.4 * w,-0.3 * w,-0.2 * w,-0.1 * w,0,0.1 * w,0.2 * w,0.3 * w,0.4 * w };
	double last_dir_deviation = 100;
	double last_target_dir = 0;
	int last_pass = 0;   //left :1 ,right:2

	CGeoPoint theirLeft = CGeoPoint(Param::Field::PITCH_LENGTH / 2, -Param::Field::GOAL_WIDTH / 2);
	CGeoPoint theirRight = CGeoPoint(Param::Field::PITCH_LENGTH / 2, Param::Field::GOAL_WIDTH / 2);
	CGeoPoint theirCenter = CGeoPoint(Param::Field::PITCH_LENGTH / 2, 0);
	CGeoPoint ourGoal = CGeoPoint(-Param::Field::PITCH_LENGTH / 2, 0);
	//	CGeoPoint relief_l_pos = CGeoPoint(200, -RELIEF_BIAS);//left relief target
	//	CGeoPoint relief_r_pos = CGeoPoint(200, RELIEF_BIAS);//right relief target
	const int maxFrared = 100 * 1.25;  // make  number of infrared frames the maximum
	const int maxMeHasBall = int(50 * 1.25);
	double OPP_HAS_BALL_DIST = 30;
	double CIRCLE_FRONT = true;
	CGeoPoint pullBallTarget = CGeoPoint(0, 0);
	CGeoPoint dragBallTarget = CGeoPoint(0, 0);
	CGeoPoint normal_push_start = CGeoPoint(0, 0);
	CGeoPoint passPointLeft = CGeoPoint(0, 0);
	CGeoPoint passPointRight = CGeoPoint(0, 0);
}

CAdvanceBallV4::CAdvanceBallV4() {
    OPP_HAS_BALL_DIST = ParamManager::Instance()->OPP_HAS_BALL_DIST;
    WEAK_OPP = ParamManager::Instance()->WEAK_OPP;
    USE_DRIBBLE_TURN = ParamManager::Instance()->USE_DRIBBLE_TURN;
    CIRCLE_FRONT = ParamManager::Instance()->CIRCLE_FRONT;
    KICKPOWER = ParamManager::Instance()->KICKPOWER;
    CHIPPOWER = ParamManager::Instance()->CHIPPOWER;
    DEBUG_ENGINE = ParamManager::Instance()->Advance_DEBUG_ENGINE;
    //SHOOT_PRECISION = ParamManager::Instance()->SHOOT_PRECISION;
    CHIP_ANGLE = ParamManager::Instance()->CHIP_ANGLE;
    FLAT_DIST = ParamManager::Instance()->FLAT_DIST;
    PULL_DIST = ParamManager::Instance()->PULL_DIST;
    RELIEF_DIST = ParamManager::Instance()->RELIEF_DIST;
    RELIEF_POWER = ParamManager::Instance()->RELIEF_POWER;
    BACK_POWER = ParamManager::Instance()->BACK_POWER;
    ADV_FPASSPOWER = ParamManager::Instance()->ADV_FPASSPOWER;
    ADV_CPASSPOWER = ParamManager::Instance()->ADV_CPASSPOWER;
    TEND_TO_SHOOT_FLAG = ParamManager::Instance()->TEND_TO_SHOOT_FLAG;
    SHOOT_BLOCKED_RANGE = ParamManager::Instance()->SHOOT_BLOCKED_RANGE;
    PASS_BLOCKED_RANGE = ParamManager::Instance()->PASS_BLOCKED_RANGE;
	pullCnt = 0;
	turnCnt = 0;
	pullBackFlag = USE_DRIBBLE_TURN;
	normalPushCnt = 0;
	dribbleTurnFinalDir = Param::Math::PI / 2;
}

CAdvanceBallV4::~CAdvanceBallV4() {
}

void CAdvanceBallV4::plan(const CVisionModule* pVision) {
	//Cycle loop 
	pullBackFlag = USE_DRIBBLE_TURN;
	if (pVision->Cycle() - _cycle > Param::Vision::FRAME_RATE * 0.1) {
		_state = BEGINNING;
	}

	/************************************************************************/
	/*
			the basic information of task
	*/
	/************************************************************************/
	int _executor = task().executor;
	int goalieNumber = 0;
	int NumofPlayerInFrontfiled = 0; //the number of player in the front feild
	int flag = task().player.flag | PlayerStatus::DODGE_OUR_DEFENSE_BOX;
	int flag_not_dribble = flag & (~PlayerStatus::DRIBBLING);
	flag = flag | PlayerStatus::DRIBBLING;
	printf("%d\n", flag_not_dribble);
	//information of images, Get the vision
	const PlayerVisionT& me = pVision->OurPlayer(_executor);
	const BallVisionT& ball = pVision->Ball();

	double BallToOurGoal = (ball.Pos() - ourGoal).mod();
	double kickDir = (CGeoPoint(Param::Field::PITCH_LENGTH / 2.0, 0) - pVision->Ball().Pos()).dir();
	double passDir = 0;
	bool pass_dir_ok = false;
	double vel_flat = 5;   //velosity of flatpass (m/s)
	int tandemNum = task().ball.receiver;

	if (tandemNum != 0 && pVision->OurPlayer(tandemNum).Valid()) {
		const PlayerVisionT& ourTandem = pVision->OurPlayer(tandemNum);
		if (me.Pos().y() < ourTandem.Pos().y())
			dribbleTurnFinalDir = Param::Math::PI / 2;
		else dribbleTurnFinalDir = -Param::Math::PI / 2;
	}

	//information of angle
	const CVector self2Ball = ball.Pos() - me.Pos();
	CVector ball2TheirCenter = theirCenter - ball.Pos();

	//information of infrared 
	//check how many consecutive frames have detected information of infrared
	bool frared = RobotSensor::Instance()->IsInfraredOn(_executor);
	if (frared) { infraredOn = infraredOn >= maxFrared ? maxFrared : infraredOn + 1; }
	else { infraredOn = 0; }

	//	bool is_runner_dir_ok = WorldModel::Instance()->KickDirArrived(pVision->Cycle(), kickDir, 0.1*Param::Math::PI / SHOOT_PRECISION, _executor);

		// cal the _pushBallTarget
	_pushBallTarget = theirCenter;
	if ((ball.X() < -Param::Field::PITCH_LENGTH / 4) && (abs(ball.Y()) > Param::Field::PENALTY_AREA_R)) {
		// ball: x < -151.25, |y| > 85
		_pushBallTarget = ball.Pos() + Utils::Polar2Vector(100, 0);
	}
	else { calcPushTarget(pVision); }

	//Treatment of the enemy
	bool isOppHasBall = checkOppHasBall(pVision);											   //check the oppo has ball, cal the most threaten oppoID, 这里检查了对面有没有拿球，并且计算了最具威胁的opponentID
	const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);                               // The vision of the opponent's most threatening robot
	const CVector opp2ball = ball.Pos() - opp.Pos();
	double ball2oppDist = opp2ball.mod();						                               // The distance between opponent's most threatening robot and the ball
	double ball2meDist = (ball.Pos() - me.Pos()).mod();
	double Me2Receiver = (me.Pos() - pVision->OurPlayer(tandemNum).Pos()).mod();
	CVector me2goal = theirCenter - me.Pos();					                               // The distance between our robot for grabbing ball and the center of Opponent goal 
	double me2BestOppDist = CVector(pVision->TheirPlayer(opponentID).Pos() - me.Pos()).mod();  // The distance between our robot for grabbing ball and opponent's most threatening robot

	//to judge whether we have or lose ball 
	bool isMeHasBall;
	bool isMechHasBall = infraredOn >= 1;                         // Infrared sensors have detected the ball
	bool visionHasBall = isVisionHasBall(pVision, _executor);     // Vision have detected the ball

	isMeHasBall = isMechHasBall || visionHasBall;
	if (isMeHasBall) {
		meHasBall = meHasBall >= maxMeHasBall ? maxMeHasBall : meHasBall + 1;
		meLoseBall = 0;
	}
	else {
		meHasBall = 0;
		meLoseBall = meLoseBall >= maxMeHasBall ? maxMeHasBall : meLoseBall + 1;
	}

	printf("%d %d %d\n", infraredOn, isMeHasBall, meHasBall);
	// DEBUG info
	if (DEBUG_ENGINE) {
		GDebugEngine::Instance()->gui_debug_x(CGeoPoint(-545, -int(300 * meHasBall / maxMeHasBall)), COLOR_YELLOW);
		GDebugEngine::Instance()->gui_debug_x(CGeoPoint(-575, -int(300 * meLoseBall / maxMeHasBall)), COLOR_ORANGE);
		GDebugEngine::Instance()->gui_debug_x(CGeoPoint(-545, -int(300 * meHasBall / maxMeHasBall)), COLOR_YELLOW);
		GDebugEngine::Instance()->gui_debug_x(CGeoPoint(-575, -int(300 * meLoseBall / maxMeHasBall)), COLOR_ORANGE);
		GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(-545, -300), "HAS", COLOR_BLACK);
		GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(-575, -300), "LOSE", COLOR_BLACK);
	}
	// cal the Numner of Player In Front filed
	for (int i = 0; i < Param::Field::MAX_PLAYER; i++) {
		if (pVision->OurPlayer(i).Valid() && i != goalieNumber)
			if (pVision->OurPlayer(i).Pos().x() > Param::Field::PITCH_LENGTH / 10)
				NumofPlayerInFrontfiled++;

	}

	switch (_state) {
	case BEGINNING:   //开始
		_state = GOTO;
		break;
	case GOTO: //拿球

		if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(270, -400), "STILL_GOTO", COLOR_YELLOW);
		// 球距离球门太近了
		if (BallToOurGoal < RELIEF_DIST && ball2oppDist < 30) {
			_state = RELIEF;
			break;
		}
		// 当我方持球时间超过MAX 需要向前带球进行决策
		if (meHasBall > meHasBallMaxCnt) {
			checkNorPushNextState(pVision, _executor);
			break;
		}
		break;
	case RELIEF:    //解围
		if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(270, -400), "RELIEF", COLOR_YELLOW);
		// 保证解围成功 成功后进入下一步跳转
		if (meLoseBall > 10) {
			_state = GOTO;
		}
		break;
	case NORMAL_PUSH:  //前推
		if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(270, -400), "NORMAL_PUSH", COLOR_YELLOW);
		normalPushCnt++;
		if (normalPushCnt == 1)  normal_push_start = me.Pos();
		// 当我方持球时间超过MAX 需要向前持球决策
		if (meHasBall > meHasBallMaxCnt || (me.Pos() - normal_push_start).mod() > NORMALPUSH_DIST) {
			checkNorPushNextState(pVision, _executor);
		}
		else if (meLoseBall > 2) {
			_state = GOTO;
		}
		break;
	case DRIBBLE_PULL:   //后拉
		if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(270, -400), "DRIBBLE_PULL", COLOR_YELLOW);
		// 如果持球超过了一定时间，考虑KICK FLATPASS CHIPPASS
		if (meHasBall > meHasBallMaxCnt) {
			if (tendToShoot(pVision, _executor, TEND_TO_SHOOT_FLAG)) {
				_state = KICK;
				break;
			}
			if (passkick(pVision, _executor) == 1) {
				_state = FLATPASS;
				break;
			}
			if (me.Pos().dist(pullBallTarget) < 10)
			{
				_state = toChipOrToFlat(pVision, _executor);
				break;
			}
		}
		if (meLoseBall > 2) {
			_state = GOTO;
		}
		break;

	case SOMBRERO://挑球过人
		if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(270, -400), "LIGHT_KICK", SOMBRERO);
		if (meLoseBall > 2) {
			_state = GOTO;
		}
		break;
	case LIGHT_KICK:   //轻推
		if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(270, -400), "LIGHT_KICK", COLOR_YELLOW);
		if (meLoseBall > 30) {
			_state = GOTO;
		}
		break;

	case KICK:       //射门
		if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(270, -400), "KICK", COLOR_YELLOW);
		if (meLoseBall > 2) {
			_state = GOTO;
		}
		break;
	case CHIPPASS:    //挑传
		if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(270, -400), "CHIPPASS", COLOR_YELLOW);
		if (meLoseBall > 10) {
			_state = GOTO;
		}
		break;
	case FLATPASS:   //平传
		if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(270, -400), "FLATPASS", COLOR_YELLOW);
		if (meLoseBall > 20) {
			_state = GOTO;
		}
		break;
	case STOP:     //停
		if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(270, -400), "STOP", COLOR_YELLOW);
		setSubTask(PlayerRole::makeItStop(_executor));
		break;
	default:
		_state = BEGINNING;
		break;
	}
	switch (_state) {
	case GOTO:
		if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -400), "GOTO", COLOR_CYAN);

		if (Utils::InOurPenaltyArea(ball.Pos(), 10)) {  //判断ball位置是否在己方禁区，存在10的偏移量
			if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(ball.Pos(), "Ball in our pen", COLOR_CYAN);
			blockingOuterOppTime(pVision, _executor);
		}
		else if (!ball.Valid()) { // when the ball can not be detected by vision,force the top towards it
			if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(ball.Pos(), "Ball invalid", COLOR_CYAN);
			double faceDir = opp.Dir() + Param::Math::PI;
			setSubTask(PlayerRole::makeItChaseKickV2(_executor, faceDir, flag_not_dribble));
			if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -350), "finish in Ball invalid", COLOR_ORANGE);
		}

		else if (checkUsingVoilence(pVision, _executor)) {  //判断能否使用暴力
			if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(ball.Pos(), "violence", COLOR_CYAN);
			voilenceActionTime(pVision, _executor);
		}

		else if (checkTheyCanShoot(pVision, _executor)) { //when oppoent ready to shoot,force the top towards it
			//pullBackFlag = false;
			if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(ball.Pos(), "they can shoot", COLOR_CYAN);
			double faceDir = opp.Dir() + Param::Math::PI;
			setSubTask(PlayerRole::makeItChaseKickV2(_executor, faceDir, flag_not_dribble));
			//setSubTask(PlayerRole::makeItNoneTrajGetBall(_executor,faceDir));
			if (ball2meDist < 30 && ball2oppDist < 30) {
				pullBackFlag = true;
				pullCnt++;
			}
		}

		//如果球正在移向敌人，同时球速大于100，同时我距离球比敌人距离球更远，同时敌人在球与我的中间
		else if (ballMoving2Opp(pVision) && ball.Vel().mod() > 100 && !meNearballThanOpp(pVision, _executor) && oppBetweenBallAndMe(pVision, _executor)) {
			if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(ball.Pos(), "Blocking opp", COLOR_CYAN);
			//cout << "blockingBestOppTime!!!" << endl;
			blockingBestOppTime(pVision, _executor);
		}
		else {   // modify by tyh 22.9.21
			KickDirection::Instance()->GenerateShootDir(_executor, pVision->OurPlayer(_executor).Pos());
			kickDir = KickDirection::Instance()->getRealKickDir();
			if (!isPassBalltoMe(pVision, _executor)) {
				if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(ball.Pos(), "Just GOTO GetBall", COLOR_CYAN);
				setSubTask(PlayerRole::makeItNoneTrajGetBall(_executor, kickDir));
			}
			else {
				if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(ball.Pos(), "Just WAIT For the ball", COLOR_CYAN);
				setSubTask(PlayerRole::makeItReceivePass(_executor, kickDir));
			}
		}
		/*
		//此时不是防守状态 -> 进入shoot状态
		else {
			KickDirection::Instance()->GenerateShootDir(_executor, pVision->OurPlayer(_executor).Pos());
			kickDir = KickDirection::Instance()->getRealKickDir();
					pullBackFlag = false;
					if (ballBetweentMeAndOpp(pVision, _executor) && me2goal.mod() < PULL_DIST) { // (opponent's most threatening robot face to their goal) and (ball is between our robot and opponent's most threatening robot)
						pullBackFlag = true;
						pullCnt++;
						KickStatus::Instance()->clearAll();
						if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(ball.Pos(), "ball between me and opp", COLOR_CYAN);
						const CGeoSegment self2targetSeg = CGeoSegment(me.Pos(), me.Pos() + Utils::Polar2Vector(800, Utils::Normalize(kickDir)));
						const bool isOppoentOnBallMovingSeg = self2targetSeg.IsPointOnLineOnSegment(opp.Pos());
						if (NumofPlayerInFrontfiled == 1 && !isOppoentOnBallMovingSeg && pullCnt > 150) {
							KickStatus::Instance()->setKick(_executor, KICKPOWER);
							setSubTask(PlayerRole::makeItShootBallV2(_executor, kickDir, flag_not_dribble));
						}
						else { setSubTask(PlayerRole::makeItShootBallV2(_executor, kickDir, flag_not_dribble)); }
						//setSubTask(PlayerRole::makeItChaseKickV1(_executor, kickDir, flag_not_dribble));
						//setSubTask(PlayerRole::makeItNoneTrajGetBall(_executor, kickDir));
						if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -350), "finish in GOTO6", COLOR_ORANGE);

					}
					else { // easily jump to this part
						if (ball2oppDist < Param::Vehicle::V2::PLAYER_SIZE * 8) {
							if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(ball.Pos(), "Ball close to opp", COLOR_CYAN);
							double faceDir = generateNormalPushDir(pVision, _executor);
							if ( pVision->OurPlayer(task().executor).Pos().x() < 225) {
								KickStatus::Instance()->setChipKick(_executor, BACK_POWER);
								setSubTask(PlayerRole::makeItChaseKickV2(_executor, faceDir, flag_not_dribble));
							}
							KickStatus::Instance()->setKick(_executor, KICKPOWER);
							setSubTask(PlayerRole::makeItChaseKickV2(_executor, faceDir, flag_not_dribble));
							//setSubTask(PlayerRole::makeItNoneTrajGetBall(_executor, kickDir));
							if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -350), "finish in GOTO7", COLOR_ORANGE);
						}
						else {
							if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(ball.Pos(), "Ball far from opp", COLOR_CYAN);
							if ( pVision->OurPlayer(task().executor).Pos().x() < 225) {
								KickStatus::Instance()->setChipKick(_executor, BACK_POWER);
								setSubTask(PlayerRole::makeItChaseKickV2(_executor, kickDir, flag_not_dribble));
							}
							setSubTask(PlayerRole::makeItChaseKickV2(_executor, kickDir, flag_not_dribble));

						}
					}
		}
		*/
		break;
	case RELIEF:  // 解围
		if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -400), "RELIEF", COLOR_CYAN);
		KickDirection::Instance()->GenerateShootDir(_executor, pVision->OurPlayer(_executor).Pos());
		kickDir = KickDirection::Instance()->getRealKickDir();
		if (pVision->GetCurrentRefereeMsg() == "ourPenaltyKick" && pVision->OurPlayer(task().executor).Pos().x() < 225) {
			KickStatus::Instance()->setChipKick(_executor, BACK_POWER);
			setSubTask(PlayerRole::makeItChaseKickV2(_executor, kickDir, flag_not_dribble));
		}
		KickStatus::Instance()->setChipKick(_executor, RELIEF_POWER);
		setSubTask(PlayerRole::makeItChaseKickV2(_executor, kickDir, flag_not_dribble));
		break;
	case DRIBBLE_PULL:  // 后拉
		if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -400), "DRIBBLE_PULL", COLOR_CYAN);
		if ((!pVision->OurPlayer(tandemNum).Valid()) || NumofPlayerInFrontfiled == 1) {
			GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(-20, -375), "Only One Attacker", COLOR_YELLOW);
			KickDirection::Instance()->GenerateShootDir(_executor, pVision->OurPlayer(_executor).Pos());
			kickDir = KickDirection::Instance()->getRealKickDir();
			setSubTask(PlayerRole::makeItSimpleGoto(_executor, pullBallTarget, kickDir, CVector(0, 0), -4, flag));
			if (ball2meDist < 20 && ball2oppDist>25) {
				KickStatus::Instance()->setChipKick(_executor, 0.4 * ADV_CPASSPOWER);
				setSubTask(PlayerRole::makeItChaseKickV2(_executor, kickDir, flag_not_dribble));
			}
		}
		else if (me2goal.mod() > PULL_DIST && me2goal.mod() < 1.4 * PULL_DIST && ball2meDist < 15) {
			setSubTask(PlayerRole::makeItSimpleGoto(_executor, me.Pos() + Utils::Polar2Vector(40, Utils::Normalize(me.Dir() + Param::Math::PI)), me.Dir(), CVector(0, 0), -4, flag));
			KickDirection::Instance()->GenerateShootDir(_executor, pVision->OurPlayer(_executor).Pos());
			kickDir = KickDirection::Instance()->getRealKickDir();
			if (ball2meDist < 20 && ball2oppDist>25) {
				KickStatus::Instance()->setChipKick(_executor, 0.0009 * ADV_CPASSPOWER * me2goal.mod());
				setSubTask(PlayerRole::makeItChaseKickV2(_executor, kickDir, flag_not_dribble));
			}
		}
		else {
			if (me2goal.mod() <= PULL_DIST && ball2meDist < 15 && ball2oppDist>35 && pass_dir_ok) {
				_state = GOTO;
				break;
			}
			setSubTask(PlayerRole::makeItSimpleGoto(_executor, pullBallTarget, kickDir, CVector(0, 0), -4, flag));
		}
		break;
	case NORMAL_PUSH:  // 向前推进
		if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -400), "NORMAL_PUSH", COLOR_CYAN);
		passDir = ((pVision->OurPlayer(tandemNum).Pos() + pVision->OurPlayer(tandemNum).Vel() / vel_flat) - pVision->OurPlayer(_executor).Pos()).dir();//依据接球车速度做传球点修正
		if (normalPushCnt > 60 * 1.25 && USE_DRIBBLE_TURN) {
			DribbleStatus::Instance()->setDribbleCommand(_executor, 2);
			KickStatus::Instance()->setChipKick(_executor, 0.0026 * ADV_CPASSPOWER * Me2Receiver);//KICKPOWER need to correct
			setSubTask(PlayerRole::makeItChaseKickV2(_executor, passDir, flag));
			if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -350), "finish in NorPush", COLOR_ORANGE);
		}
		else {
			//cout << "here" << endl;
			//setSubTask(PlayerRole::makeItCrazyPush(_executor, kickDir));
			setSubTask(PlayerRole::makeItNoneTrajGetBall(_executor, passDir, CVector(0, 0), flag_not_dribble, (-2.0)));
			if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -350), "finish in NorPush", COLOR_ORANGE);
		}
		break;
	case LIGHT_KICK:   //挑球
		if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -400), "LIGHT_KICK", COLOR_CYAN);
		KickStatus::Instance()->clearAll();
		KickStatus::Instance()->setKick(_executor, 200); // kick lightly
		break;
	case SOMBRERO:
		if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -400), "SOMBRERO", COLOR_CYAN);
		puts("SOMBRERO!!");
		KickStatus::Instance()->clearAll();
		KickStatus::Instance()->setChipKick(_executor, 50);//KICKPOWER need to correct
		break;

	case KICK:   // 射门
		if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -400), "KICK", COLOR_CYAN);
		//KickDirection::Instance()->GenerateShootDir(_executor, pVision->OurPlayer(_executor).Pos());
		//kickDir = KickDirection::Instance()->getRealKickDir();
		kickDir = pointShootDir(pVision, pVision->OurPlayer(_executor).Pos());
		if (kickDir == 1000) {
			KickDirection::Instance()->GenerateShootDir(_executor, pVision->OurPlayer(_executor).Pos());
			kickDir = KickDirection::Instance()->getRealKickDir();
		}
		//is_runner_dir_ok = abs(me.Dir() - kickDir) < 0.2 * Param::Math::PI / SHOOT_PRECISION;
		KickStatus::Instance()->clearAll();/*
		if (pVision->GetCurrentRefereeMsg() == "ourPenaltyKick" && pVision->OurPlayer(task().executor).Pos().x() < 225) {
			KickStatus::Instance()->setChipKick(_executor, BACK_POWER);
			setSubTask(PlayerRole::makeItChaseKickV2(_executor, kickDir, flag_not_dribble));
		}*/
		if (ballInTheirPenaltyArea(pVision, _executor)) {
			KickStatus::Instance()->setKick(_executor, KICKPOWER);
			setSubTask(PlayerRole::makeItChaseKickV2(_executor, kickDir, flag_not_dribble));
			if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -350), "finish in KICK", COLOR_ORANGE);
		}
		else {
			KickStatus::Instance()->setKick(_executor, KICKPOWER);
			setSubTask(PlayerRole::makeItShootBallV2(_executor, kickDir, flag_not_dribble));
			// 此处删除的原因是 解决了不能够秒射的bug
			/*
			if (isDirOK(pVision,me.Dir(),me.Pos(),kickDir)) {
//				decideChipOrShoot(pVision, kickDir, _executor);
				KickStatus::Instance()->setKick(_executor, KICKPOWER);
				//  22.10.9 by tyh   实在搞不懂你这么保守干什么啊 射不就完了吗
				setSubTask(PlayerRole::makeItShootBallV2(_executor, kickDir, flag_not_dribble));
				if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -350), "finish in KICK", COLOR_ORANGE);
			}
			else {
				//cout << "there there there" << endl;
				setSubTask(PlayerRole::makeItNoneTrajGetBall(_executor, kickDir));
				//setSubTask(PlayerRole::makeItChaseKickV1(_executor, kickDir, flag_not_dribble));
				if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -350), "finish in KICK", COLOR_ORANGE);
			}
			*/
		}
		//KickStatus::Instance()->clearAll();
		break;
	case FLATPASS:  // 平传
		if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -400), "FLATPASS", COLOR_CYAN);
		//passDir=(CGeoPoint(0,0)- pVision->OurPlayer(_executor).Pos()).dir();
		//KickDirection::Instance()->GenerateShootDir(_executor, pVision->OurPlayer(_executor).Pos());
		//passDir = ((pVision->OurPlayer(tandemNum).Pos() + pVision->OurPlayer(tandemNum).Vel() / vel_flat) - pVision->OurPlayer(_executor).Pos()).dir();//依据接球车速度做传球点修正
		//passPointLeft = SupportPos2022::Instance()->getSupportPos(pVision).getLeftSupportPos();
		//passPointRight = SupportPos2022::Instance()->getSupportPos(pVision).getRightSupportPos();
		passDir = flatPassDir(pVision, _executor);
		//pass_dir_ok = abs(me.Dir() - passDir) < 0.2 * Param::Math::PI / SHOOT_PRECISION;
		//KickStatus::Instance()->setKick(_executor, ADV_FPASSPOWER );//KICKPOWER need to correct
		//setSubTask(PlayerRole::makeItShootBallV2(_executor, passDir, flag_not_dribble));
		if (isDirOK(pVision, me.Dir(), me.Pos(), passDir)) {
			if (ball2oppDist < 30) {
				break;
			}
			KickStatus::Instance()->setKick(_executor, ADV_FPASSPOWER);//KICKPOWER need to correct
			setSubTask(PlayerRole::makeItChaseKickV2(_executor, passDir, flag_not_dribble));
			if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -350), "finish in FLATPASS", COLOR_ORANGE);
		}
		else {
			if (ball2oppDist < 30) {
				break;
			}
			setSubTask(PlayerRole::makeItNoneTrajGetBall(_executor, passDir, CVector(0, 0), flag_not_dribble, (-2.0)));
			//setSubTask(PlayerRole::makeItSimpleGoto(_executor, me.Pos() + Utils::Polar2Vector(30, Utils::Normalize(me.Dir() + Param::Math::PI)), passDir, CVector(0, 0), 0, flag));
		}
		break;
	case CHIPPASS:  // 挑传
		if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -400), "CHIPPASS", COLOR_CYAN);
		//KickDirection::Instance()->GenerateShootDir(_executor, pVision->OurPlayer(_executor).Pos());
		passDir = ((pVision->OurPlayer(tandemNum).Pos() + pVision->OurPlayer(tandemNum).Vel() / vel_flat) - pVision->OurPlayer(_executor).Pos()).dir();//依据接球车速度做传球点修正
		pass_dir_ok = abs(me.Dir() - passDir) < 0.7 * Param::Math::PI / SHOOT_PRECISION;
		if (pass_dir_ok) {
			if (ball2oppDist < 35) {
				break;
			}
			KickStatus::Instance()->setChipKick(_executor, 0.0026 * ADV_CPASSPOWER * Me2Receiver);//KICKPOWER need to correct
			setSubTask(PlayerRole::makeItChaseKickV2(_executor, passDir, flag_not_dribble));
			if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -350), "finish in CHIPPASS", COLOR_ORANGE);
		}
		else {
			if (ball2oppDist < 35) {
				break;
			}
			setSubTask(PlayerRole::makeItNoneTrajGetBall(_executor, passDir, CVector(0, 0), flag_not_dribble, (-2.0)));
			//setSubTask(PlayerRole::makeItSimpleGoto(_executor, me.Pos() + Utils::Polar2Vector(30, Utils::Normalize(me.Dir() + Param::Math::PI)), passDir, CVector(0, 0), 0, flag));
		}
		break;
	}


	_cycle = pVision->Cycle();
	CStatedTask::plan(pVision);
}

/************************************************************/
/*
*	functions for judging position
*/
/************************************************************************/

// 应用场景：KICK中，球进入对方禁区的判断 
bool CAdvanceBallV4::ballInTheirPenaltyArea(const CVisionModule* pVision, const int vecNumber) {
	const BallVisionT& ball = pVision->Ball();
	return Utils::InTheirPenaltyArea(ball.Pos(), 0);
}

/*
bool CAdvanceBallV4::checkBallBehind(const CVisionModule* pVision, double angle) {
	const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
	const BallVisionT& ball = pVision->Ball();
	CVector ball2opp = opp.Pos() - ball.Pos();
	bool ballDirBehindOpp = abs(Utils::Normalize(opp.Dir() - ball2opp.dir())) < angle;
	bool ballDistBehindOpp = ball2opp.mod() < OPP_HAS_BALL_DIST;
	//GDebugEngine::Instance()->gui_debug_line(opp.Pos(),opp.Pos() + Utils::Polar2Vector(200 , 0),COLOR_BLACK);
	bool isBallBehindOpp = ballDirBehindOpp && ballDistBehindOpp;
	return isBallBehindOpp;
}
useless function -> deleted
*/

bool CAdvanceBallV4::checkBallFront(const CVisionModule* pVision, double angle) { //判断球是否在敌人前面 存在夹角要求
	const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
	const BallVisionT& ball = pVision->Ball();
	CVector opp2ball = ball.Pos() - opp.Pos();
	bool ballDirFrontOpp = abs(Utils::Normalize(opp.Dir() - opp2ball.dir())) < angle;
	bool ballDistFrontOpp = opp2ball.mod() < OPP_HAS_BALL_DIST + 10;
	//GDebugEngine::Instance()->gui_debug_line(opp.Pos(),opp.Pos() + Utils::Polar2Vector(200 , 0),COLOR_BLACK);
	bool isBallFrontOpp = ballDirFrontOpp && ballDistFrontOpp;
	return isBallFrontOpp;
}
// modify by tyh

bool CAdvanceBallV4::Me2OppTooclose(const CVisionModule* pVision, const int vecNumber) { //是否太近了
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
	const BallVisionT& ball = pVision->Ball();
	CVector me2Ball = ball.Pos() - me.Pos();
	CVector me2Opp = opp.Pos() - me.Pos();
	printf("%.3lf, %.3lf\n", abs(me2Ball.mod()), abs(me2Opp.mod()));
	if (abs(me2Ball.mod()) < 15 && abs(me2Opp.mod()) < 15) {
		return true;
	}
	return false;
}
bool CAdvanceBallV4::ballBetweentMeAndOpp(const CVisionModule* pVision, const int vecNumber) { //判断球是否在敌人与我中间
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
	const BallVisionT& ball = pVision->Ball();
	double kickDir = KickDirection::Instance()->getRealKickDir();
	const CGeoSegment self2targetSeg = CGeoSegment(me.Pos(), me.Pos() + Utils::Polar2Vector(800, Utils::Normalize(kickDir)));

	CVector me2Ball = ball.Pos() - me.Pos();
	CVector ball2Opp = opp.Pos() - ball.Pos();
	const bool isOppoentOnBallMovingSeg = self2targetSeg.IsPointOnLineOnSegment(opp.Pos());

	if (abs(Utils::Normalize(me2Ball.dir() - ball2Opp.dir())) < Param::Math::PI * 55 / 180 && isOppoentOnBallMovingSeg) {
		return true;
	}
	else {
		return false;
	}
}
/*
bool CAdvanceBallV4::meFaceOpp(const CVisionModule* pVision, const int vecNumber) { // two robots face to face,and very close to each other
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
	const BallVisionT& ball = pVision->Ball();

	const double meDir = me.Dir();
	const double oppDir = opp.Dir();
	const double meToOppDist = (me.Pos() - opp.Pos()).mod();

	if ((abs(Utils::Normalize(meDir - oppDir)) > Param::Math::PI / 1.5) && meToOppDist < 30.0) {
		return true;
	}
	else {
		return false;
	}
}
*/
/*
bool CAdvanceBallV4::checkBallFrontOrBehindBigPre(const CVisionModule* pVision) {
	const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
	const BallVisionT& ball = pVision->Ball();
	double opp2BallDir = CVector(ball.Pos() - opp.Pos()).dir();
	double oppDir = opp.Dir();
	double opp2BallDir_oppDir_Diff = abs(Utils::Normalize(opp2BallDir - oppDir));
	bool isBallFrontOpp;
	isBallFrontOpp = opp2BallDir_oppDir_Diff < Param::Math::PI * 2.0 / 3.0;
	return isBallFrontOpp;
}
*/
bool CAdvanceBallV4::isOppFaceTheirDoor(const CVisionModule* pVision, const double angle) {
	//判断是否球距离opp很近，opp距离他们球门很近
	//generateNormalPushDir判断使用
	const BallVisionT& ball = pVision->Ball();
	const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
	double opp2BallDist = (opp.Pos() - ball.Pos()).mod();
	double judgeAngle = abs(Utils::Normalize((opp.Dir() - CVector(CGeoPoint(Param::Field::PITCH_LENGTH / 2.0, 0) - opp.Pos()).dir()))); //Abs( the angle of (the direction of opponent's most threatening robot) and (the direction of the robot towards the center of their gate) )
	bool isFaceTheirDoor = judgeAngle < angle || judgeAngle == angle;
	bool isBallNearOpp = opp2BallDist < OPP_HAS_BALL_DIST;
	//GDebugEngine::Instance()->gui_debug_line(opp.Pos(), opp.Pos() + Utils::Polar2Vector(200,Utils::Normalize(opp.Dir() + Param::Math::PI / 3)),COLOR_WHITE);
	//GDebugEngine::Instance()->gui_debug_line(opp.Pos(), opp.Pos() + Utils::Polar2Vector(200,Utils::Normalize(opp.Dir() - Param::Math::PI / 3)),COLOR_WHITE);
	return isFaceTheirDoor && isBallNearOpp;
}

bool CAdvanceBallV4::isOppFaceOurDoor(const CVisionModule* pVision, double angle) {
	//判断是否球距离opp很近，opp距离我们球门很近
	const BallVisionT& ball = pVision->Ball();
	const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
	double opp2BallDist = (opp.Pos() - ball.Pos()).mod();
	bool isBallNearOpp = opp2BallDist < OPP_HAS_BALL_DIST;
	double judgeAngle = abs(Utils::Normalize((opp.Dir() - CVector(CGeoPoint(-Param::Field::PITCH_LENGTH / 2.0, 0) - opp.Pos()).dir())));
	bool isFaceOurDoor = judgeAngle < angle || judgeAngle == angle;
	return isFaceOurDoor && isBallNearOpp;
}

bool CAdvanceBallV4::ballOnTheirHalf(const CVisionModule* pVision) {
	if (pVision->Ball().X() > 0) {
		return true;
	}
	else {
		return false;
	}
}

bool CAdvanceBallV4::meFaceTheirDoor(const CVisionModule* pVision, const int vecNumber) {
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	double me2GoalAngle;
	me2GoalAngle = abs(Utils::Normalize((CVector(CGeoPoint(Param::Field::PITCH_LENGTH / 2.0, 0) - me.Pos()).dir() - me.Dir())));
	if (me2GoalAngle < Param::Math::PI / 3) {
		return true;
	}
	else {
		return false;
	}
}

//functions of action

bool CAdvanceBallV4::checkUsingVoilence(const CVisionModule* pVision, int vecNumber) {
	//能不能使用VoilenceActionTime
	const BallVisionT& ball = pVision->Ball();
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
	CVector opp2ball = ball.Pos() - opp.Pos();
	if (!WEAK_OPP && isOppFaceOurDoor(pVision, Param::Math::PI / 3.0) && checkBallFront(pVision, Param::Math::PI / 3.0) && (!ballBetweentMeAndOpp(pVision, vecNumber) || me.Pos().x() > opp.Pos().x())) {
		return true;
	}
	else { return false; }
}
void CAdvanceBallV4::voilenceActionTime(const CVisionModule* pVision, int vecNumber) {
	//对着对面威胁最大的车 朝它撞就完事了
	const BallVisionT& ball = pVision->Ball();
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
	CVector opp2me = CVector(me.Pos() - opp.Pos());
	CGeoPoint faceTarget = ball.Pos();
	double extr_dist = 0.0;
	CVector self2ball = ball.Pos() - me.Pos();
	if (self2ball.mod() > 60) {
		extr_dist = self2ball.mod() * self2ball.mod() / 500;
		if (extr_dist > 150) {
			extr_dist = 150;
		}
		CGeoPoint fast_point = faceTarget + Utils::Polar2Vector(extr_dist, (faceTarget - me.Pos()).dir());
		if (!Utils::OutOfField(fast_point, 0)) {
			faceTarget = fast_point;
		}
	}
	CVector finalSpeed = Utils::Polar2Vector(500, (faceTarget - me.Pos()).dir());
	TaskT task;
	task.executor = vecNumber;
	task.player.angle = Utils::Normalize(self2ball.dir() + Param::Math::PI);
	task.player.max_acceleration = 650;
	task.player.pos = opp.Pos();
	task.player.vel = finalSpeed;
	task.player.max_deceleration = 650;
	setSubTask(TaskFactoryV2::Instance()->GotoPosition(task));
}

// modified by tyh 22.7.14
// 当前球在已方禁区，修改为与地方威胁度最大球员之间连线
// 判断ball位置是在己方禁区时使用，存在10的偏移量，已经是非常紧急的情况了
void CAdvanceBallV4::blockingOuterOppTime(const CVisionModule* pVision, int vecNumber) {
	//	const BallVisionT& ball = pVision->Ball();
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
	int flag = task().player.flag;

	CGeoPoint blockingIntoPenaltyPoint = CGeoPoint(0, 0);
	//	int theirBestPlayerOutOurPenalty = opponentID;
	double theirBestPlayerOutOurPenalty2Goal = 0;

	const CBestPlayer::PlayerList& oppList = BestPlayer::Instance()->theirFastestPlayerToBallList();
	int OppNum = oppList.size();
	for (int i = 0; i < OppNum; i++) {
		CGeoPoint oppPos = pVision->TheirPlayer(oppList[i].num).Pos();
		if (!Utils::InOurPenaltyArea(oppPos, 0)) {
			//			theirBestPlayerOutOurPenalty = oppList[i].num;
			theirBestPlayerOutOurPenalty2Goal = CVector(oppPos - CGeoPoint(-Param::Field::PITCH_LENGTH / 2.0, 0)).dir();
			blockingIntoPenaltyPoint = DefendUtils::calcPenaltyLinePoint(theirBestPlayerOutOurPenalty2Goal, POS_SIDE_MIDDLE, 1);
			//  calcPenaltyLinePoint最后一个参数已经弃用了
			break;
		}
	}
	double me2TheOppDir = (opp.Pos() - me.Pos()).dir();
	setSubTask(PlayerRole::makeItSimpleGoto(vecNumber, blockingIntoPenaltyPoint, me2TheOppDir, CVector(0, 0), 0, flag));
}

//如果球正在移向敌人，同时球速大于100，同时我距离球比敌人距离球更远，同时敌人在球与我的中间
//应用条件似乎过于苛刻
void CAdvanceBallV4::blockingBestOppTime(const CVisionModule* pVision, int vecNumber) {
	const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	const BallVisionT& ball = pVision->Ball();
	CGeoSegment oppShootLine = CGeoSegment(opp.Pos(), CGeoPoint(-Param::Field::PITCH_LENGTH / 2.0, 0));
	CGeoCirlce oppCircle = CGeoCirlce(opp.Pos(), OPP_HAS_BALL_DIST * 0.8);
	CGeoSegmentCircleIntersection intersectionPoint = CGeoSegmentCircleIntersection(oppShootLine, oppCircle);
	// 此处计算shootline和circle的交点，我应当到交点上面去
	CGeoPoint movingPoint = intersectionPoint.point1();
	CVector me2Ball = ball.Pos() - me.Pos();
	//setSubTask(PlayerRole::makeItSimpleGoto(vecNumber, movingPoint, me2Ball.dir(), CVector(0, 0), 0, 0));
	setSubTask(PlayerRole::makeItGoto(vecNumber, movingPoint, me2Ball.dir(), CVector(0, 0), 0, 0));
}

//functions for conditional judgment
bool CAdvanceBallV4::oppRushMe(const CVisionModule* pVision, int vecNumber) {
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	const BallVisionT& ball = pVision->Ball();
	if (opponentID > 0) {
		const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
		CVector ball2enemy = ball.Pos() - opp.Pos();
		double diff_oppMoving2Me = Utils::Normalize(ball2enemy.dir() - ball.Vel().dir());
		if (ball.Valid()) {
			printf("fabs(ball2enemy.mod()): %.1f  opp.Vel().mod(): %.1f\n", fabs(ball2enemy.mod()), opp.Vel().mod());
			if (fabs(ball2enemy.mod()) < 200) return true;
			if (diff_oppMoving2Me < Param::Math::PI / 6 && opp.Vel().mod() > 100)return true;
		}
	}
	return false;
}
//判断球是不是正在移向opponentID，采用球与敌人连线向量和球速向量做夹角的方法
bool CAdvanceBallV4::ballMoving2Opp(const CVisionModule* pVision) {
	const BallVisionT& ball = pVision->Ball();
	if (opponentID > 0) {
		const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
		CVector ball2enemy = opp.Pos() - ball.Pos();
		double diff_ballMoving2Opp = Utils::Normalize(ball2enemy.dir() - ball.Vel().dir());
		if (ball.Valid() && abs(diff_ballMoving2Opp) < Param::Math::PI / 12) {
			//cout << "ballMoning2Opp!!!" << endl;
			return true;
		}
	}
	return false;
}
//ballMoving2Me
// Add by tyh 22.9.21
bool CAdvanceBallV4::isPassBalltoMe(const CVisionModule* pVision, int vecNumber) {
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	const BallVisionT& ball = pVision->Ball();
	CVector ball2me = me.Pos() - ball.Pos();
	double diff_ballMoving2Me = Utils::Normalize(ball2me.dir() - ball.Vel().dir());
	if (ball.Valid() && abs(diff_ballMoving2Me) < Param::Math::PI / 12 && ball.Vel().mod() > 150) {// 假设70
		//cout << "ballMoning2Opp!!!" << endl;
		return true;
	}
	return false;
}

//过人了
bool CAdvanceBallV4::isMePassedOpp(const CVisionModule* pVision, const int vecNumber) {
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
	const BallVisionT& ball = pVision->Ball();

	CVector me2opp = opp.Pos() - me.Pos();
	CVector opp2Ball = ball.Pos() - opp.Pos();
	CVector ball2Opp = opp.Pos() - ball.Pos();
	CVector me2Ball = ball.Pos() - me.Pos();
	bool meDirControlBall = abs(Utils::Normalize(me2Ball.dir() - me.Dir())) < Param::Math::PI / 4;
	bool meDistControlBall = me2Ball.mod() < Param::Vehicle::V2::PLAYER_FRONT_TO_CENTER + 10.5;
	bool mePassOpp_1 = abs(Utils::Normalize(me2Ball.dir() - me2opp.dir())) > Param::Math::PI / 2.5;
	bool mePassOpp_2 = abs(Utils::Normalize(opp.Dir() - ball2Opp.dir())) < Param::Math::PI / 3.0;
	printf("%d %d %d %d\n", meDirControlBall, meDistControlBall, (mePassOpp_1 || mePassOpp_2), me2opp.mod() > 25);
	if (meDirControlBall && meDistControlBall && (mePassOpp_1 || mePassOpp_2) || me2opp.mod() > 25) {
		// GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, 0), "PASS!!!!!!!!!!!", COLOR_WHITE);
		return true;
	}
	else {
		return false;
	}
}

bool CAdvanceBallV4::isVisionHasBall(const CVisionModule* pVision, const int vecNumber) {
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	const BallVisionT& ball = pVision->Ball();
	double visionJudgDist = 11;
	bool distVisionHasBall = CVector(me.Pos() - ball.Pos()).mod() <= visionJudgDist;
	// cout << CVector(me.Pos() - ball.Pos()).mod() << endl;
	bool dirVisionHasBall;
	double meDir = me.Dir();
	double me2Ball = (ball.Pos() - me.Pos()).dir();
	double meDir_me2Ball_Diff = abs(Utils::Normalize((meDir - me2Ball)));
	if (meDir_me2Ball_Diff < Param::Math::PI / 6.0) {
		dirVisionHasBall = true;
	}
	else {
		dirVisionHasBall = false;
	}
	bool isVisionPossession = dirVisionHasBall && distVisionHasBall;
	return isVisionPossession;
}

bool CAdvanceBallV4::isShootBlockedInPenaltyArea(const CVisionModule* pVision, const int vecNumber)
{
	//to judge whether a series of shooting angle be blocked of not 

	//imformation of images
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	const BallVisionT& ball = pVision->Ball();
	const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
	//to judge whether be blocked or not
	bool isBlocked;

	KickDirection::Instance()->GenerateShootDir(vecNumber, pVision->OurPlayer(vecNumber).Pos());
	double kickDir = KickDirection::Instance()->getRealKickDir();

	CGeoLine ball2ourGoal = /*CGeoLine(me.Pos(),ourGoal); */CGeoLine(me.Pos(), kickDir);
	CGeoPoint projectionPoint = ball2ourGoal.projection(opp.Pos());
	double opp2LineDist = (projectionPoint - opp.Pos()).mod();
	//	bool isPassOpp;//false means haven't passed
	//isPassOpp =/*Utils::Normalize( me.Dir() - (opp.Pos() - me.Pos()).dir()) > Param::Math::PI/3; */isMePassedOpp(pVision,vecNumber);

	isBlocked = opp2LineDist < (Param::Vehicle::V2::PLAYER_SIZE + 5)/*&& !isPassOpp*/;
	//cout <<"opp2LineDist"<<opp2LineDist<<"   dist:"<<opp2LineDist<<" "<<endl;
	if (isBlocked) {
		return true;
	}
	else {
		return false;
	}
}

bool CAdvanceBallV4::isShootBlocked(const CVisionModule* pVision, const int vecNumber) {
	//to judge whether a series of shooting angle be blocked of not

	//imformation of images
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	const BallVisionT& ball = pVision->Ball();
	const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);

	KickDirection::Instance()->GenerateShootDir(vecNumber, pVision->OurPlayer(vecNumber).Pos());
	double kickDir = KickDirection::Instance()->getRealKickDir();

	CGeoLine ball2ourGoal = /*CGeoLine(me.Pos(),ourGoal); */CGeoLine(me.Pos(), kickDir);
	CGeoPoint theirGoal(Param::Field::PITCH_LENGTH / 2, 0);
	CVector ball2theirGoal = theirGoal - ball.Pos();
	double shootRange = 0;
	shootRange = ball2theirGoal.mod() * abs(Param::Math::PI / 180 * CHIP_ANGLE);
	//cout << "opp2LineDist:"<<opp2LineDist<<"  "<<"isPassOpp"<<isPassOpp<<endl;
	if (shootRange > Param::Field::GOAL_WIDTH) {
		return true;
	}
	else {
		CGeoPoint projectionPoint = ball2ourGoal.projection(opp.Pos());
		double opp2LineDist = (projectionPoint - opp.Pos()).mod();
		bool isPassOpp = isMePassedOpp(pVision, vecNumber);
		bool isBlocked = opp2LineDist < 1.5 * Param::Vehicle::V2::PLAYER_SIZE && !isPassOpp;
		if (isBlocked && (ball2theirGoal.mod() > CHIPPOWER))
		{
			if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(100, 100), "want to chip", COLOR_CYAN);

			return true;
		}
		return false;
	}
}

//检查对面有没有拿球，同时计算opponentID
bool CAdvanceBallV4::checkOppHasBall(const CVisionModule* pVision) {
	int _executor = task().executor;
	const BallVisionT& ball = pVision->Ball();
	const PlayerVisionT& me = pVision->OurPlayer(_executor);
	const CVector self2ball = ball.Pos() - me.Pos();
	//compute the datas of opponent's most threatening robot
	opponentID = 0;

	const CBestPlayer::PlayerList& oppList = BestPlayer::Instance()->theirFastestPlayerToBallList();
	if (oppList.size() < 1)return false;
	else opponentID = oppList[0].num;
	if (!pVision->TheirPlayer(opponentID).Valid()) {
		opponentID = getTheirMostClosetoPosPlayerNum(pVision, pVision->Ball().Pos());
	}
	const PlayerVisionT& opponent = pVision->TheirPlayer(opponentID);

	//mark opponent's most threatening robot
	if (true && DEBUG_ENGINE)
		GDebugEngine::Instance()->gui_debug_msg(opponent.Pos(), "Best Opp!", COLOR_WHITE);

	CVector opponent2ball = ball.Pos() - opponent.Pos();
	CVector opp2ourGoal = ourGoal - opponent.Pos();
	double opponent2ball_diff = fabs(Utils::Normalize(opponent2ball.dir() - opponent.Dir()));
	double judgeDist = OPP_HAS_BALL_DIST;
	if (opponent2ball.mod() < judgeDist && opponent2ball_diff < Param::Math::PI * 70 / 180)
		return true; // take opponent's direction into consideration.If direction not towards the ball,ignore it
	else
		return false;
}

bool CAdvanceBallV4::checkTheyCanShoot(const CVisionModule* pVision, int vecNumber) {
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

//bool CAdvanceBallV4::oppblockme(const cvisionmodule* pvision, const int vecnumber) {
//	const ballvisiont& ball = pvision->ball();
//	const playervisiont& me = pvision->ourplayer(vecnumber);
//	const playervisiont& opp = pvision->theirplayer(opponentid);
//	cgeoline shootline = cgeoline(me.pos(), cvector(ball.pos() - me.pos()).dir());
//	cgeopoint projecpoint = shootline.projection(opp.pos());
//	cvector me2goal = theircenter - me.pos();
//	cvector me2ball = ball.pos() - me.pos();
//	cvector ball2opp = opp.pos() - ball.pos();
//	cvector me2opp = opp.pos() - me.pos();
//	bool isballnearopp = (ball2opp.mod() < 35);
//	bool ismefacetheirgoal = abs(utils::normalize(me.dir() - me2goal.dir())) < param::math::pi / 6.0;
//	bool oppfrontme = abs(utils::normalize(me2ball.dir() - me2opp.dir())) < param::math::pi / 2.0;
//	if (projecpoint.dist(opp.pos()) < 3*param::vehicle::v2::player_size + param::field::ball_size /*&& (ball.pos().x() > me.pos().x() && ball.pos().x() < opp.pos().x())*/) {
//	return true && /*ismefacetheirgoal &&*/ isballnearopp && oppfrontme; //;
//	}
//	else {
//	return false;
//	}
//}

bool CAdvanceBallV4::oppBlockMe(const CVisionModule* pVision, const int vecNumber) {
	const BallVisionT& ball = pVision->Ball();
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
	CGeoLine shootLine = CGeoLine(me.Pos(), CVector(ball.Pos() - me.Pos()).dir());
	CVector me2Goal = theirCenter - me.Pos();
	CVector me2Ball = ball.Pos() - me.Pos();
	CGeoPoint projectionPoint;
	int n = 0;
	double opp2LineDist = 0;
	while (n <= Param::Field::MAX_PLAYER) {
		if (!pVision->TheirPlayer(n).Valid()) { n++; continue; }
		projectionPoint = shootLine.projection(pVision->TheirPlayer(n).Pos());
		CVector ball2opp = opp.Pos() - ball.Pos();
		CVector me2Opp = opp.Pos() - me.Pos();
		bool isBallNearOpp = (ball2opp.mod() < 200);  // modify by tyh  35 -> 200
		bool isMeFaceTheirGoal = abs(Utils::Normalize(me.Dir() - me2Goal.dir())) < Param::Math::PI / 6.0;
		bool oppFrontMe = abs(Utils::Normalize(me2Ball.dir() - me2Opp.dir())) < Param::Math::PI / 2.0;

		opp2LineDist = (projectionPoint - pVision->TheirPlayer(n).Pos()).mod();
		if (opp2LineDist < 5 * Param::Vehicle::V2::PLAYER_SIZE + Param::Field::BALL_SIZE && isBallNearOpp && oppFrontMe) {
			// modify by tyh 3 -> 5
			return  true;
		}
		n++;
	}
	return false;

}

bool CAdvanceBallV4::selectChipKick(const CVisionModule* pVision, int vecNumber) {
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	const BallVisionT& ball = pVision->Ball();
	const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
	if (!opp.Valid()) { return true; }
	KickDirection::Instance()->GenerateShootDir(vecNumber, pVision->OurPlayer(vecNumber).Pos());
	double kickDir = KickDirection::Instance()->getRealKickDir();
	CVector ball2opp = opp.Pos() - ball.Pos();
	CVector me2ball = ball.Pos() - me.Pos();
	CGeoLine ball2Goal = CGeoLine(ball.Pos(), kickDir);
	CGeoPoint projPoint = ball2Goal.projection(opp.Pos());
	double blockDist = opp.Pos().dist(projPoint);
	double oppDist = opp.Pos().dist(ball.Pos());
	if (oppDist > 20) {
		return true;
	}
	return false;
}

//functions for calculation
// 
// 这个是不是可以好好利用一下
void CAdvanceBallV4::calcPushTarget(const CVisionModule* pVision) {
	int _executor = task().executor;
	const BallVisionT& ball = pVision->Ball();
	const PlayerVisionT& me = pVision->OurPlayer(_executor);
	const CVector self2ball = ball.Pos() - me.Pos();

	// priority is given to whether there is congestion in the direction of the goal
	CVector ball2theirGoal = theirCenter - ball.Pos();
	bool is_shootline_blocked = false;
	int opp_best_index = 0;
	double oppDist2Ball = 1000;
	int left_opp_num = 0;
	int right_opp_num = 0;
	for (int i = 0; i < Param::Field::MAX_PLAYER; i++) {
		if (!pVision->TheirPlayer(i).Valid())
			continue;

		const PlayerVisionT& opp = pVision->TheirPlayer(i);
		if (opp.Pos().dist(theirCenter) < 80)
			continue;

		CVector ball2opp = opp.Pos() - ball.Pos();
		double diff_block = Utils::Normalize(ball2opp.dir() - ball2theirGoal.dir());

		if (abs(diff_block) < Param::Math::PI / 4 && ball2opp.mod() < 50)
			is_shootline_blocked = true;

		if (opp.Y() > 0)
			right_opp_num++;
		else
			left_opp_num++;

		if (ball2opp.mod() < oppDist2Ball) {
			opp_best_index = i;
			oppDist2Ball = ball2opp.mod();
		}
	}

	//only if robot has controlled the ball safely, can the function  considers to change the pushTarget
	bool safe_change_dir = false;
	if (opp_best_index > 0) {
		const PlayerVisionT& bestOpp = pVision->TheirPlayer(opp_best_index);
		CVector bestOpp2ball = ball.Pos() - bestOpp.Pos();
		double usBlockAngle = Utils::Normalize(bestOpp2ball.dir() - self2ball.dir());
		if (RobotSensor::Instance()->IsInfraredOn(_executor) &&
			abs(usBlockAngle) < Param::Math::PI / 2.0)
			safe_change_dir = true;
	}

	if (!safe_change_dir) {
		_pushBallTarget = theirCenter;
		return;
	}

	if (!is_shootline_blocked)
		attack_side = ATTACK_TO_CENTER;
	else if (attack_side == ATTACK_TO_CENTER) {
		attack_side = left_opp_num > right_opp_num ? ATTACK_TO_RIGHT : ATTACK_TO_LEFT;
	}
	else if (attack_side == ATTACK_TO_LEFT) {
		attack_side = left_opp_num > right_opp_num + 1 ? ATTACK_TO_RIGHT : ATTACK_TO_LEFT;
	}
	else {
		attack_side = right_opp_num > left_opp_num + 1 ? ATTACK_TO_LEFT : ATTACK_TO_RIGHT;
	}

	// check the position of the nearest teammate
	double minDist = 100;
	int friendID = 0;
	int friendSide = 0; // 1 means right,-1 means left
	for (int i = 0; i < Param::Field::MAX_PLAYER; i++) {
		if (i == _executor)
			continue;
		if (!pVision->OurPlayer(i).Valid())
			continue;
		const PlayerVisionT& myfriend = pVision->OurPlayer(i);
		if (myfriend.Pos().dist(ourGoal) < 80)
			continue;

		double friendDist = myfriend.Pos().dist(me.Pos());
		if (friendDist < minDist) {
			friendID = i;
			minDist = friendDist;
			CVector ball2friend = myfriend.Pos() - ball.Pos();
			double diff2friend = Utils::Normalize(ball2friend.dir() - self2ball.dir());
			//cout<<"No."<<i<<" diff friend:"<<Utils::Rad2Deg(diff2friend)<<endl;
			if (abs(diff2friend) > 5 * Param::Math::PI / 12)
				continue;

			// assume that the direction when our robot catch the ball is towards opponent's half-field
			friendSide = diff2friend > 0 ? 1 : -1;
		}
	}

	double push_dir;
	if (friendSide != 0) {
		const PlayerVisionT& selected_friend = pVision->OurPlayer(friendID);
		CGeoPoint friend_front = selected_friend.Pos() + Utils::Polar2Vector(Param::Vehicle::V2::PLAYER_FRONT_TO_CENTER, selected_friend.Dir());
		CVector ball2friend = friend_front - ball.Pos();
		double temp_diff = Utils::Normalize(ball2theirGoal.dir() - ball2friend.dir());
		push_dir = Utils::Normalize(ball2friend.dir() + 0.2 * temp_diff);
		if (friendSide > 0)
			push_dir = min(Param::Math::PI / 2, push_dir);
		else if (friendSide < 0)
			push_dir = max(-Param::Math::PI / 2, push_dir);
	}

	switch (attack_side) {
	case ATTACK_TO_CENTER: // if no be blocked,attack towards the middle
		_pushBallTarget = theirCenter;
		break;
	case ATTACK_TO_LEFT:
		if (friendSide < 0 || friendSide > 0 && ball.Y() < Param::Field::PITCH_WIDTH / 2 - 80)
			_pushBallTarget = ball.Pos() + Utils::Polar2Vector(60, push_dir);
		else {
			double pd = Utils::Normalize(ball2theirGoal.dir() - Param::Math::PI / 3);
			pd = max(-Param::Math::PI / 2, pd);
			_pushBallTarget = ball.Pos() + Utils::Polar2Vector(60, pd);
			//cout<<"attack left\n";
		}
		break;
	case ATTACK_TO_RIGHT:
		if (friendSide > 0 || friendSide < 0 && ball.Y() > -Param::Field::PITCH_WIDTH / 2 + 80)
			_pushBallTarget = ball.Pos() + Utils::Polar2Vector(60, push_dir);
		else {
			double pd = Utils::Normalize(ball2theirGoal.dir() + Param::Math::PI / 3);
			pd = min(Param::Math::PI / 2, pd);
			_pushBallTarget = ball.Pos() + Utils::Polar2Vector(60, pd);
			//cout<<"attack right\n";
		}
		break;
	}
}

bool CAdvanceBallV4::checkEnoughSpaceToTurnKick(const CVisionModule* pVision) {
	const BallVisionT& ball = pVision->Ball();
	CGeoPoint goalCenter = CGeoPoint(Param::Field::PITCH_LENGTH / 2.0, 0);
	double ball2GoalCenterDist = (ball.Pos() - goalCenter).mod();
	if (ball2GoalCenterDist > 120 && ball2GoalCenterDist < 230) {
		return true;
	}
	else
		return false;
}

// 10.9 疑问 为什么要设置这样一个Chip or Shoot
void CAdvanceBallV4::decideChipOrShoot(const CVisionModule* pVision, double& kickDir, const int vecNumber)
{
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	const BallVisionT& ball = pVision->Ball();
	const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);

	bool shootBlocked; //true means be blocked,chipshoot; false means not be blocked,flatshoot

	CGeoLine ball2ourGoal = /*CGeoLine(me.Pos(),ourGoal); */CGeoLine(me.Pos(), kickDir);
	CGeoPoint theirGoal(Param::Field::PITCH_LENGTH / 2, 0);
	CVector ball2theirGoal = theirGoal - ball.Pos();
	double shootRange = 0;
	shootRange = ball2theirGoal.mod() * abs(Param::Math::PI / 180 * CHIP_ANGLE);
	//cout << "opp2LineDist:"<<opp2LineDist<<"  "<<"isPassOpp"<<isPassOpp<<endl;
	if (shootRange > Param::Field::GOAL_WIDTH) {
		if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(100, 100), "too far chip", COLOR_CYAN);
		KickStatus::Instance()->setChipKick(vecNumber, 0.4 * CHIPPOWER);
	}
	else {
		CGeoPoint projectionPoint = ball2ourGoal.projection(opp.Pos());
		double opp2LineDist = (projectionPoint - opp.Pos()).mod();
		bool isPassOpp = isMePassedOpp(pVision, vecNumber);
		bool isBlocked = opp2LineDist < Param::Vehicle::V2::PLAYER_SIZE && !isPassOpp;
		if (ball2theirGoal.mod() < FLAT_DIST)
		{
			if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(100, 100), "close flat", COLOR_CYAN);
			KickStatus::Instance()->setKick(vecNumber, KICKPOWER);
		}
		else {
			if ((me.Pos() - opp.Pos()).mod() > 25 && !checkEnoughSpaceToTurnKick(pVision)) {
				if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(100, 100), " blocked chip", COLOR_CYAN);
				KickStatus::Instance()->setChipKick(vecNumber, 0.4 * CHIPPOWER);
			}
			else
			{
				if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(100, 100), "open flat", COLOR_CYAN);
				KickStatus::Instance()->setKick(vecNumber, KICKPOWER);
			}
		}
	}
}

bool CAdvanceBallV4::faceTheirGoal(const CVisionModule* pVision, const int executor) {
	double dist = Param::Field::PITCH_LENGTH / 2 - pVision->OurPlayer(executor).Pos().x();
	double buffer = 0;
	const CGeoPoint theirLeft = CGeoPoint(Param::Field::PITCH_LENGTH / 2, -Param::Field::GOAL_WIDTH / 2 - buffer);
	const CGeoPoint theirRight = CGeoPoint(Param::Field::PITCH_LENGTH / 2, Param::Field::GOAL_WIDTH / 2 + buffer);
	double leftAngle = Utils::Normalize((theirLeft - pVision->OurPlayer(executor).Pos()).dir());
	double rightAngle = Utils::Normalize((theirRight - pVision->OurPlayer(executor).Pos()).dir());
	double myDir = pVision->OurPlayer(executor).Dir();

	double rawLeftAngle = Utils::Normalize((theirLeft - pVision->OurPlayer(executor).Pos()).dir());
	double rawRifhtAngle = Utils::Normalize((theirRight - pVision->OurPlayer(executor).Pos()).dir());
	double myRawDir = pVision->OurPlayer(executor).Dir();

	bool isToTheirGoal = ((myDir > leftAngle) && (myDir < rightAngle));
	bool isRawToTheirGoal = ((myRawDir > rawLeftAngle) && (myRawDir < rawRifhtAngle));

	if (DEBUG_ENGINE) {
		GDebugEngine::Instance()->gui_debug_line(theirLeft, pVision->OurPlayer(executor).Pos());
		GDebugEngine::Instance()->gui_debug_line(theirRight, pVision->OurPlayer(executor).Pos());
		GDebugEngine::Instance()->gui_debug_line(pVision->OurPlayer(executor).Pos() + Utils::Polar2Vector(100, myDir), pVision->OurPlayer(executor).Pos());

		GDebugEngine::Instance()->gui_debug_line(theirLeft, pVision->OurPlayer(executor).Pos());
		GDebugEngine::Instance()->gui_debug_line(theirRight, pVision->OurPlayer(executor).Pos());
		GDebugEngine::Instance()->gui_debug_line(pVision->OurPlayer(executor).Pos() + Utils::Polar2Vector(100, myRawDir), pVision->OurPlayer(executor).Pos());
	}
	return (isToTheirGoal || isRawToTheirGoal);
}

int CAdvanceBallV4::getTheirMostClosetoPosPlayerNum(const CVisionModule* pVision, CGeoPoint pos) {
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

// todo
double CAdvanceBallV4::generateNormalPushDir(const CVisionModule* pVision, const int vecNumber) {
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
	const BallVisionT& ball = pVision->Ball();
	double faceDir = 0.0;
	if (!opp.Valid()) {
		KickDirection::Instance()->GenerateShootDir(vecNumber, pVision->OurPlayer(vecNumber).Pos());
		faceDir = KickDirection::Instance()->getRealKickDir();
		return faceDir;
	}
	if (abs(ball.Pos().y()) < Param::Field::PITCH_WIDTH / 2 * 0.4 || me.Pos().x() < -50 || isOppFaceTheirDoor(pVision) || !checkBallFront(pVision, Param::Math::PI / 4.0)) {
		//cout << "there there there" << endl;
		KickDirection::Instance()->GenerateShootDir(vecNumber, pVision->OurPlayer(vecNumber).Pos());
		faceDir = KickDirection::Instance()->getRealKickDir();
		return faceDir;
	}
	else if (abs(ball.Pos().y()) > Param::Field::PITCH_WIDTH / 2 * 0.70) {
		//cout << "here here here" << endl;
		faceDir = opp.Dir() + Param::Math::PI;
		return faceDir;
	}
	else {
		KickDirection::Instance()->GenerateShootDir(vecNumber, pVision->OurPlayer(vecNumber).Pos());
		double kickDir = KickDirection::Instance()->getRealKickDir();
		double maxDir = Utils::Normalize(opp.Dir() + Param::Math::PI);
		double diffDir = Utils::Normalize(kickDir - maxDir);
		if (abs(diffDir) < Param::Math::PI / 15 || (kickDir > 0 && maxDir > kickDir) || (kickDir < 0 && maxDir < kickDir)) {
			return kickDir;
		}
		else {
			faceDir = Utils::Normalize(kickDir - diffDir * (3.33 * (abs(ball.Pos().y()) / (Param::Field::PITCH_WIDTH / 2)) - 1.33));
			return faceDir;
		}
	}
}

bool CAdvanceBallV4::meNearballThanOpp(const CVisionModule* pVision, const int vecNumber) {
	bool meNearBall = false;
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
	const BallVisionT& ball = pVision->Ball();
	double me2ballDist = (me.Pos() - ball.Pos()).mod();
	double opp2ballDist = (opp.Pos() - ball.Pos()).mod();
	if (opp2ballDist - me2ballDist > Param::Vehicle::V2::PLAYER_SIZE * 1.5) {
		meNearBall = true;
	}
	else {
		meNearBall = false;
	}
	return meNearBall;
}

bool CAdvanceBallV4::oppBetweenBallAndMe(const CVisionModule* pVision, const int vecNumber) {
	bool oppBetween = false;
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
	const BallVisionT& ball = pVision->Ball();
	CVector opp2ball = ball.Pos() - opp.Pos();
	CVector me2opp = opp.Pos() - me.Pos();
	if (abs(Utils::Normalize(opp2ball.dir() - me2opp.dir())) < Param::Math::PI / 3.0) {
		oppBetween = true;
	}
	else if (abs(Utils::Normalize(opp2ball.dir() - me2opp.dir())) < Param::Math::PI / 2.0 && me2opp.mod() < Param::Vehicle::V2::PLAYER_SIZE * 3) {
		oppBetween = true;
	}
	else {
		oppBetween = false;
	}
	return oppBetween;
}
//functions for judging passing
bool CAdvanceBallV4::tendToShoot(const CVisionModule* pVision, int vecNumber, int k_bottom) {
	int n = 0;
	int best_n = 0;
	//information of image
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	const BallVisionT& ball = pVision->Ball();
	bool shootBlocked = false;
	KickDirection::Instance()->GenerateShootDir(vecNumber, pVision->OurPlayer(vecNumber).Pos());
	double kickDir = KickDirection::Instance()->getRealKickDir();
	CGeoLine ball2ourGoal = CGeoLine(me.Pos(), kickDir);
	CGeoPoint projectionPoint;
	double k_m = SHOOT_BLOCKED_RANGE;//ball2theirbest / 15;
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

	//	if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(ball.Pos(), "Debuging", COLOR_BLACK);
	if (shootBlocked || me2goal > 800) {   // 该参数也可以放入ini
		cout << "Can NOT shoot!!!" << endl;
		return false;
	}
	else {
		if (kickValid)cout << "Can shoot!!!" << endl;
		else cout << "Can NOT shoot!!!" << endl;
		return kickValid;
	}
}

double CAdvanceBallV4::pointShootDir(const CVisionModule* pVision, CGeoPoint startPoint) {

	int n = 0;
	const BallVisionT& ball = pVision->Ball();
	bool shootBlocked = false;
	CGeoPoint projectionPoint;
	double k_m = SHOOT_BLOCKED_RANGE;//ball2theirbest / 15;
	double opp2LineDist_max = 0;
	int best_i = -1;
	for (int i = 0; i < 9; i++) {
		//去除同边角，不易进球，易撞球门
		if (i == 0) {
			if (startPoint.y() < -0.5 * Param::Field::PITCH_WIDTH + 150 ||
				(startPoint.x() > 0.5 * Param::Field::PITCH_LENGTH - 150 && startPoint.y() < -0.5 * Param::Field::PITCH_WIDTH + 200) ||
				(startPoint.x() > 0.5 * Param::Field::PITCH_LENGTH - 200 && startPoint.y() < -0.5 * Param::Field::PITCH_WIDTH + 180))
				continue;
		}
		if (i == 1) {
			if ((startPoint.x() > 0.5 * Param::Field::PITCH_LENGTH - 100 && startPoint.y() < -0.5 * Param::Field::PITCH_WIDTH + 200) ||
				(startPoint.x() > 0.5 * Param::Field::PITCH_LENGTH - 200 && startPoint.y() < -0.5 * Param::Field::PITCH_WIDTH + 180))

				continue;
		}
		if (i == 8) {
			if ((startPoint.x() > 0.5 * Param::Field::PITCH_LENGTH - 100 && startPoint.y() > 0.5 * Param::Field::PITCH_WIDTH - 200) ||
				(startPoint.x() > 0.5 * Param::Field::PITCH_LENGTH - 200 && startPoint.y() > 0.5 * Param::Field::PITCH_WIDTH - 180))

				continue;
		}
		if (i == 9) {
			if (startPoint.y() > 0.5 * Param::Field::PITCH_WIDTH - 150 ||
				(startPoint.x() > 0.5 * Param::Field::PITCH_LENGTH - 150 && startPoint.y() > 0.5 * Param::Field::PITCH_WIDTH - 200) ||
				(startPoint.x() > 0.5 * Param::Field::PITCH_LENGTH - 200 && startPoint.y() > 0.5 * Param::Field::PITCH_WIDTH - 180))
				continue;
		}
		double shootDir = (CGeoPoint(Param::Field::PITCH_LENGTH / 2.0, goal[i]) - startPoint).dir();
		CGeoLine ball2ourGoal = CGeoLine(startPoint, shootDir);
		double opp2LineDist = 1000;
		n = 0;
		while (n <= Param::Field::MAX_PLAYER) {
			if (!pVision->TheirPlayer(n).Valid()) { n++; continue; }
			projectionPoint = ball2ourGoal.projection(pVision->TheirPlayer(n).Pos());
			if (opp2LineDist > (projectionPoint - pVision->TheirPlayer(n).Pos()).mod() && projectionPoint.x() >= startPoint.x()) {
				opp2LineDist = (projectionPoint - pVision->TheirPlayer(n).Pos()).mod();
			}
			n++;
		}
		if (opp2LineDist_max < opp2LineDist) {
			opp2LineDist_max = opp2LineDist;
			best_i = i;
		}
	}
	double shootDir = (CGeoPoint(Param::Field::PITCH_LENGTH / 2.0, goal[best_i]) - startPoint).dir();
	if (opp2LineDist_max < k_m * Param::Vehicle::V2::PLAYER_SIZE) {
		shootBlocked = true;
		return 1000;
	}
	else return shootDir;
}
bool CAdvanceBallV4::isBlocked(const CVisionModule* pVision, CGeoPoint startPoint, CGeoPoint targetPoint) {
	int n = 0;
	const BallVisionT& ball = pVision->Ball();
	bool passBlocked = false;
	double passDir = (targetPoint - startPoint).dir();
	CGeoLine start2Target = CGeoLine(startPoint, passDir);
	CGeoPoint projectionPoint;
	double k_m = 4;
	double opp2LineDist = 1000;
	while (n <= Param::Field::MAX_PLAYER) {
		if (!pVision->TheirPlayer(n).Valid()) { n++; continue; }
		projectionPoint = start2Target.projection(pVision->TheirPlayer(n).Pos());
		if (opp2LineDist > (projectionPoint - pVision->TheirPlayer(n).Pos()).mod() && projectionPoint.x() < Param::Field::PITCH_LENGTH / 2.0 && projectionPoint.x() > startPoint.x()) {
			opp2LineDist = (projectionPoint - pVision->TheirPlayer(n).Pos()).mod();
			if (opp2LineDist < k_m * Param::Vehicle::V2::PLAYER_SIZE) {
				passBlocked = true;
				break;
			}
		}
		n++;
	}//to judge whether be blocked or not
	return passBlocked;
}

bool CAdvanceBallV4::isDirOK(const CVisionModule* pVision, double myDir, CGeoPoint myPos, double targetDir) {


	if (abs(targetDir - last_target_dir) > 0.3 * Param::Math::PI / SHOOT_PRECISION) {
		last_dir_deviation = 100;  //重置角度差
	}
	last_target_dir = targetDir;
	const BallVisionT& ball = pVision->Ball();
	double me2ballDir = (ball.Pos() - myPos).dir();
	CGeoLine start2Target = CGeoLine(myPos, me2ballDir);
	CGeoPoint projectionPoint;
	double k_m = 0.9;
	double opp2LineDist = 1000;
	int n = 0;
	//while (n <= Param::Field::MAX_PLAYER) {
	//	if (!pVision->TheirPlayer(n).Valid()) { n++; continue; }
	//	projectionPoint = start2Target.projection(pVision->TheirPlayer(n).Pos());
	//	if (opp2LineDist > (projectionPoint - pVision->TheirPlayer(n).Pos()).mod() && projectionPoint.x() < Param::Field::PITCH_LENGTH / 2.0 && projectionPoint.x() > myPos.x()) {
	//		opp2LineDist = (projectionPoint - pVision->TheirPlayer(n).Pos()).mod();
	//		if (opp2LineDist < k_m * Param::Vehicle::V2::PLAYER_SIZE) {
	//			last_dir_deviation = myDir - targetDir;
	//			return false;
	//		}
	//	}
	//	n++;
	//}
	//if (opp2LineDist > 1.6 * Param::Vehicle::V2::PLAYER_SIZE && abs(myDir - targetDir) < 0.1 * Param::Math::PI / SHOOT_PRECISION) {
	//	return true;
	//}
	if (abs(myDir - targetDir) > 0.2 * Param::Math::PI / SHOOT_PRECISION) {
		last_dir_deviation = myDir - targetDir;
		return false;
	}
	else {
		if (((abs(myDir - targetDir) > abs(last_dir_deviation) || (myDir - targetDir) * last_dir_deviation <= 0) && abs(myDir - targetDir) < 0.15 * Param::Math::PI / SHOOT_PRECISION) ||
			abs(myDir - targetDir) < 0.08 * Param::Math::PI / SHOOT_PRECISION) {
			last_dir_deviation = 100;
			return true;
		}
	}
	last_dir_deviation = myDir - targetDir;
	return false;
}


int CAdvanceBallV4::passkick(const CVisionModule* pVision, int vecNumber) {
	passPointLeft = SupportPos2022::Instance()->getSupportPos(pVision).getLeftSupportPos();
	passPointRight = SupportPos2022::Instance()->getSupportPos(pVision).getRightSupportPos();
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	bool isBlockLeft = isBlocked(pVision, me.Pos(), passPointLeft);
	bool isBlockRight = isBlocked(pVision, me.Pos(), passPointRight);
	double LeftShootDir = pointShootDir(pVision, passPointLeft);
	double RightShootDir = pointShootDir(pVision, passPointRight);
	double me2left = (me.Pos() - passPointLeft).mod();
	double me2right = (me.Pos() - passPointRight).mod();
	double left2goal = (CGeoPoint(Param::Field::PITCH_LENGTH / 2.0, 0) - passPointLeft).mod();
	double right2goal = (CGeoPoint(Param::Field::PITCH_LENGTH / 2.0, 0) - passPointRight).mod();

	if ((!isBlockLeft && LeftShootDir != 1000 && me2left < 450 && left2goal < 700) || (!isBlockRight && RightShootDir != 1000 && me2right < 450 && right2goal < 700))
	{
		//  此处传球的参数需要动态化！！！
		//  22.9.21 by tyh modify
		// 300 500
		return 1;
	}
	return 0;
}

int CAdvanceBallV4::toChipOrToFlat(const CVisionModule* pVision, int vecNumber) {//参考isShootBlocked()

	passPointLeft = SupportPos2022::Instance()->getSupportPos(pVision).getLeftSupportPos();
	passPointRight = SupportPos2022::Instance()->getSupportPos(pVision).getRightSupportPos();
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	bool isBlockLeft = isBlocked(pVision, me.Pos(), passPointLeft);
	bool isBlockRight = isBlocked(pVision, me.Pos(), passPointRight);
	double LeftShootDir = pointShootDir(pVision, passPointLeft);
	double RightShootDir = pointShootDir(pVision, passPointRight);
	if (isBlockLeft && isBlockRight) {
		return CHIPPASS;
	}
	else {
		return FLATPASS;
	}
}
double CAdvanceBallV4::flatPassDir(const CVisionModule* pVision, int vecNumber) {
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	passPointLeft = SupportPos2022::Instance()->getSupportPos(pVision).getLeftSupportPos();
	passPointRight = SupportPos2022::Instance()->getSupportPos(pVision).getRightSupportPos();
	bool isBlockLeft = isBlocked(pVision, me.Pos(), passPointLeft);
	bool isBlockRight = isBlocked(pVision, me.Pos(), passPointRight);
	double LeftShootDir = pointShootDir(pVision, passPointLeft);
	double RightShootDir = pointShootDir(pVision, passPointRight);
	double passDir = 0;
	int pass = 0;
	if (!isBlockLeft && isBlockRight) {
		pass = 1;
		passDir = (passPointLeft - me.Pos()).dir();
	}
	if (isBlockLeft && !isBlockRight) {
		pass = 2;
		passDir = (passPointRight - me.Pos()).dir();
	}
	if (!isBlockLeft && !isBlockRight) {
		if (LeftShootDir != 1000 && RightShootDir == 1000) {
			pass = 1;
			passDir = (passPointLeft - me.Pos()).dir();
		}
		if (LeftShootDir == 1000 && RightShootDir != 1000) {
			pass = 2;
			passDir = (passPointRight - me.Pos()).dir();
		}
		if (LeftShootDir != 1000 && RightShootDir != 1000) {
			double me2left = abs(me.Y() - passPointLeft.y());
			double me2right = abs(me.Y() - passPointRight.y());
			if (me2left > me2right) {
				pass = 1;
				passDir = (passPointLeft - me.Pos()).dir();
			}
			if (me2left < me2right) {
				pass = 2;
				passDir = (passPointRight - me.Pos()).dir();
			}
			if (abs(me2left - me2right) < 20) {
				if (last_pass == 1) {
					passDir = (passPointLeft - me.Pos()).dir();
					pass = 1;
				}
				if (last_pass == 2) {
					passDir = (passPointRight - me.Pos()).dir();
					pass = 2;
				}
				if (last_pass == 0) {
					passDir = (passPointLeft - me.Pos()).dir();
					pass = 1;
				}

			}

		}
	}

	if (isBlockLeft && isBlockRight) {
		double me2left = abs(me.Y() - passPointLeft.y());
		double me2right = abs(me.Y() - passPointRight.y());
		if (me2left > me2right) {
			pass = 1;
			passDir = (passPointLeft - me.Pos()).dir();
		}
		if (me2left < me2right) {
			pass = 2;
			passDir = (passPointRight - me.Pos()).dir();
		}
		if (abs(me2left - me2right) < 20) {
			if (last_pass == 1) {
				pass = 1;
				passDir = (passPointLeft - me.Pos()).dir();
			}
			if (last_pass == 2) {
				pass = 2;
				passDir = (passPointRight - me.Pos()).dir();
			}
			if (last_pass == 0) {
				passDir = (passPointLeft - me.Pos()).dir();
				pass = 1;
			}

		}
	}
	last_pass = pass;
	return passDir;

}


CPlayerCommand* CAdvanceBallV4::execute(const CVisionModule* pVision) {
	if (subTask()) {
		return subTask()->execute(pVision);
	}
	if (_directCommand) {
		return _directCommand;
	}
	return 0;
}

void CAdvanceBallV4::checkNorPushNextState(const CVisionModule* pVision, const int vecNumber) {
	const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
	const BallVisionT& ball = pVision->Ball();
	CVector me2goal = theirCenter - me.Pos();
	// 如果距离相对于球门达到了射程范围
	if (me2goal.mod() < KICK_DIST) {
		//判断能否进行射门 如果可以就shoot
		//如果不行 采用DRIBBLE技术 or 传球
		if (tendToShoot(pVision, vecNumber, TEND_TO_SHOOT_FLAG)) {
			if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, 400), (tendToShoot(pVision, vecNumber, TEND_TO_SHOOT_FLAG) ? "1" : "0"), COLOR_ORANGE);
			_state = KICK;
			return;
		}
		else {
			//如果可以采用DRIBBLE，即pullBackFlag=1
			if (pullBackFlag == true && me2goal.mod() < KICK_DIST - 50) {
				pullBackFlag = false;
				pullBallTarget = me.Pos() + Utils::Polar2Vector(70, Utils::Normalize(me.Dir() + Param::Math::PI));
				if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -375), "turn to Dribblepull", COLOR_YELLOW);
				_state = DRIBBLE_PULL;
				return;
			}
			//否则考虑传球
			_state = toChipOrToFlat(pVision, vecNumber);
			return;
		}
	}
	//如果未达到射程 考虑传球
	if (passkick(pVision, vecNumber) == 1) {
		_state = FLATPASS;
		return;
	}
	if (oppBlockMe(pVision, vecNumber)) {
		_state = toChipOrToFlat(pVision, vecNumber);
		return;
	}
	if (!isMePassedOpp(pVision, vecNumber)) {
		_state = SOMBRERO;
		return;
	}
	if (Me2OppTooclose(pVision, vecNumber)) {
		_state = DRIBBLE_PULL;
	}/*
	if (oppRushMe(pVision, vecNumber)) {
		_state = SOMBRERO;
		return;
	}*/

	if (ball.X() <= 0)_state = NORMAL_PUSH;
	else _state = LIGHT_KICK;
}
