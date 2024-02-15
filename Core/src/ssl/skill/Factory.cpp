#include "Factory.h"
#include <TaskMediator.h>

#include <misc_types.h>
#include <WorldModel/WorldModel.h>
#include "CommandFactory.h"
#include "GDebugEngine.h"

/************************************************************************/
/*                                concrete skill                            */
/************************************************************************/
//0.working skills
#include "Break.h"

//1.atom skills(TODO)
#include "GotoPosition.h"
#include "SmartGotoPosition.h"

//2.basic skills(TODO)
#include "StopRobot.h"
#include "Speed.h"
#include "OpenSpeed.h"
#include "SpeedTest.h"
#include "InterceptBallV3.h"
#include "CrazyPush.h"
#include "CircleAndPass.h"
#include "GoAndTurnKick.h"
#include "GoAndTurnKickV3.h"
#include "GoAndTurnKickV4.h"
#include "GoAroundRobot.h"
#include "GoAndTurn.h"
#include "ChaseKickV1.h"
#include "ChaseKickV2.h"
#include "TouchKick.h"
#include "PenaltyDefV1.h"
#include "PenaltyDefV2.h"
#include "TestCircleBall.h"
#include "AdvanceBallV1.h"
#include "AdvanceBallV2.h"
#include "AdvanceBallV3.h"
#include "AdvanceBallV4.h"
#include "Advance.h"
#include "PenaltyKickV2.h"
#include "Tandem.h"
#include "JustKick.h"
#include "WaitTouch.h"
#include "GetBallV3.h"
#include "GetBallV4.h"
#include "GetBallV5.h"
#include "SlowGetBall.h"
#include "TimeDelayTest.h"
#include "Marking.h"
#include "Goalie2013.h"
#include "Goalie2022.h"
#include "ShootoutGoalie.h"
#include "shootoutGoalieV2.h"
#include "InterceptBall.h"
#include "ShootBall.h"
#include "ShootBallV2.h"
#include "PassBall.h"
#include "ReceivePass.h"
#include "FetchBall.h"
#include "PenaltyKick2013.h"
#include "PenaltyKick2014.h"
#include "PenaltyGoalie2012.h"
#include "ProtectBall.h"
#include "ForceStartRush.h"
#include "DribbleTurn.h"
#include "DriftKick.h"
#include "MarkingTouch.h"
#include "MarkingFront.h"
#include "DribbleTurnKick.h"
#include "DribbleTurnKickV2.h"
#include "InterceptBallV4.h"
#include "InterceptBallV2.h"
#include "GoSupport.h"
#include "GoTechChalPos.h"
#include "GotoPositionNew.h"
#include "GoPIDCircle.h"
#include "ChaseToGetBall.h"
#include "lightkick.h"
#include "TechDefenceRobot.h"
#include "Tech3PassRobot.h"

/************************************************************************/
/*                      TaskFactoryV2                                    */
/* ?????:                                                                */
/* ???CStatedScriptPlay) ??????? CBasicPlay ?§Ö? setTask() ?Ú…????    */
/* ??????????????§³????????????????? TaskMediator ????                */
/* ??§µ?ActionModule??????????? execute() ???????????§³??????????    */
/************************************************************************/




//////////////////////////////////////////////////////////////////////////
// an template used as a task pool for storing tasks for executing player
template < class CTaskType >
CPlayerTask* CTaskFactoryV2::MakeTask(const TaskT& task)
{
	static CTaskType taskPool[Param::Field::MAX_PLAYER];
	taskPool[task.executor].reset(task);
	return &taskPool[task.executor];
}

//working
//
// CPlayerTask* CTaskFactoryV2::GofourRobot(const TaskT& task){
// 	return MakeTask< CGofourRobot >(task);
// }

CPlayerTask* CTaskFactoryV2::lightkick(const TaskT& task)
{
	return MakeTask<Clightkick>(task);
}
CPlayerTask* CTaskFactoryV2::Break(const TaskT& task)
{
	return MakeTask<CBreak>(task);
}
CPlayerTask* CTaskFactoryV2::SpeedTest(const TaskT& task)
{
	return MakeTask<CSpeedTest>(task);
}
CPlayerTask* CTaskFactoryV2::TechDefenceRobot(const TaskT& task){
	return MakeTask<CTechDefence>(task);
}
CPlayerTask* CTaskFactoryV2::Tech3PassRobot(const TaskT& task){
	return MakeTask<CTech3Pass>(task);
}
//////////////////////////////////////////////////////////////////////////
// current now debugged skill for game
CPlayerTask* CTaskFactoryV2::GoPIDCircle(const TaskT& task) {
	return MakeTask< CGoPIDCircle >(task);
}
CPlayerTask* CTaskFactoryV2::GotoPosition(const TaskT& task) {
	return MakeTask< CGotoPosition >(task);
}

CPlayerTask* CTaskFactoryV2::SmartGotoPosition(const TaskT& task) {
	return MakeTask< CSmartGotoPosition >(task);
}

CPlayerTask* CTaskFactoryV2::GotoPositionNew(const TaskT& task) {
	return MakeTask< CGotoPositionNew >(task);
}

CPlayerTask* CTaskFactoryV2::StopRobot(const TaskT& task) {
	return MakeTask< CStopRobotV2 >(task);
}

CPlayerTask* CTaskFactoryV2::Speed(const TaskT& task) {
	return MakeTask< CSpeed >(task);
}

CPlayerTask* CTaskFactoryV2::OpenSpeed(const TaskT& task) {
	return MakeTask< COpenSpeed >(task);
}

CPlayerTask* CTaskFactoryV2::NoneTrajGetBallV4(const TaskT& task) {
	return MakeTask< CGetBallV4 >(task);
}
CPlayerTask* CTaskFactoryV2::NoneTrajGetBall(const TaskT& task) {
	return MakeTask< CGetBallV5 >(task);
}
CPlayerTask* CTaskFactoryV2::NoneTrajGetBallV3(const TaskT& task) {
	return MakeTask< CGetBallV3 >(task);
}
CPlayerTask* CTaskFactoryV2::ForceStartRush(const TaskT& task) {
	return MakeTask< CForceStartRush >(task);
}

CPlayerTask* CTaskFactoryV2::ChaseKickV1(const TaskT& task) {
	return MakeTask< CChaseKickV1 >(task);
}

CPlayerTask* CTaskFactoryV2::ChaseKickV2(const TaskT& task) {
	return MakeTask< CChaseKickV2>(task);
}
CPlayerTask* CTaskFactoryV2::ChaseToGetBall(const TaskT& task) {
	return MakeTask< CChaseToGetBall>(task);
}

CPlayerTask* CTaskFactoryV2::ProtectBall(const TaskT& task) {
	return MakeTask< CProtectBall>(task);
}

CPlayerTask* CTaskFactoryV2::TouchKick(const TaskT& task) {
	return MakeTask< CTouchKick >(task);
}

CPlayerTask* CTaskFactoryV2::InterceptBallV2(const TaskT& task) {
	return MakeTask< CInterceptBallV2>(task);
}
CPlayerTask* CTaskFactoryV2::InterceptBallV3(const TaskT& task) {
	return MakeTask< CInterceptBallV3>(task);
}

CPlayerTask* CTaskFactoryV2::MarkingTouch(const TaskT& task) {
	return MakeTask< CMarkingTouch >(task);
}

CPlayerTask* CTaskFactoryV2::MarkingFront(const TaskT& task) {
	return MakeTask< CMarkingFront >(task);
}

CPlayerTask* CTaskFactoryV2::GoSupport(const TaskT& task) {
	return MakeTask< CGoSupport >(task);
}

CPlayerTask* CTaskFactoryV2::ShootBall(const TaskT& task) {
	return MakeTask< CShootBall >(task);
}

CPlayerTask* CTaskFactoryV2::ShootBallV2(const TaskT& task) {
	return MakeTask< CShootBallV2 >(task);
}

CPlayerTask* CTaskFactoryV2::DriftKick(const TaskT& task) {
	return MakeTask< CDriftKick >(task);
}


CPlayerTask* CTaskFactoryV2::CarzyPush(const TaskT& task) {
	return MakeTask<CCrazyPush>(task);
}

CPlayerTask* CTaskFactoryV2::GoAroundRobot(const TaskT& task) {
	return MakeTask< CGoAroundRobot >(task);
}
CPlayerTask* CTaskFactoryV2::GoTechChalPos(const TaskT& task) {
	return MakeTask< CGoTechChalPos >(task);
}
CPlayerTask* CTaskFactoryV2::PenaltyDefV1(const TaskT& task) {
	return MakeTask< CPenaltyDefV1 >(task);
}

CPlayerTask* CTaskFactoryV2::PenaltyDefV2(const TaskT& task) {
	return MakeTask< CPenaltyDefV2 >(task);
}

CPlayerTask* CTaskFactoryV2::AdvanceBallV1(const TaskT& task) {
	return MakeTask< CAdvanceBallV1 >(task);
}

CPlayerTask* CTaskFactoryV2::AdvanceBallV2(const TaskT& task) {
	return MakeTask< CAdvanceBallV2 >(task);
}

CPlayerTask* CTaskFactoryV2::AdvanceBallV3(const TaskT& task) {
	return MakeTask< CAdvanceBallV3 >(task);
}
CPlayerTask* CTaskFactoryV2::AdvanceBallV4(const TaskT& task) {
	return MakeTask< CAdvanceBallV4 >(task);
}
CPlayerTask* CTaskFactoryV2::Advance(const TaskT& task) {
	return MakeTask< CAdvance >(task);
}
CPlayerTask* CTaskFactoryV2::PenaltyKickV2(const TaskT& task) {
	return MakeTask< CPenaltyKickV2 >(task);
}

CPlayerTask* CTaskFactoryV2::Tandem(const TaskT& task)
{
	return MakeTask< CTandem >(task);
}

CPlayerTask* CTaskFactoryV2::GoAndTurnKick(const TaskT& task) {
	return MakeTask< CGoAndTurnKick >(task);
}

CPlayerTask* CTaskFactoryV2::CircleAndPass(const TaskT& task) {
	return MakeTask< CCircleAndPass >(task);
}


//CPlayerTask* CTaskFactoryV2::GoAndTurnKickV2(const TaskT& task) {
//    return MakeTask< CGoAndTurnKickV2 >(task);
//}
CPlayerTask* CTaskFactoryV2::GoAndTurnKickV3(const TaskT& task) {
	return MakeTask< CGoAndTurnKickV3 >(task);
}

CPlayerTask* CTaskFactoryV2::GoAndTurnKickV4(const TaskT& task) {
	return MakeTask< CGoAndTurnKickV4 >(task);
}

CPlayerTask* CTaskFactoryV2::GoAndTurn(const TaskT& task) {
	return MakeTask< CGoAndTurn >(task);
}
CPlayerTask* CTaskFactoryV2::SlowGetBall(const TaskT& task) {
	return MakeTask< CSlowGetBall >(task);
}

CPlayerTask* CTaskFactoryV2::timeDelayTest(const TaskT& task) {
	return MakeTask< CTimeDelayTest >(task);
}

CPlayerTask* CTaskFactoryV2::Marking(const TaskT& task) {
	return MakeTask< CMarking >(task);
}

CPlayerTask* CTaskFactoryV2::TestCircleBall(const TaskT& task) {
	return MakeTask<CTestCircleBall>(task);
}


CPlayerTask* CTaskFactoryV2::PassBall(const TaskT& task) {
	return MakeTask< CPassBall >(task);
}

CPlayerTask* CTaskFactoryV2::ReceivePass(const TaskT& task) {
	return MakeTask< CReceivePass >(task);
}

CPlayerTask* CTaskFactoryV2::FetchBall(const TaskT& task) {
	return MakeTask< CFetchBall >(task);
}

CPlayerTask* CTaskFactoryV2::PenaltyGoalie(const TaskT& task) {
	return MakeTask< CPenaltyGoalie2012 >(task);
}

CPlayerTask* CTaskFactoryV2::Goalie2013(const TaskT& task) {
	return MakeTask< CGoalie2013 >(task);
}

CPlayerTask* CTaskFactoryV2::Goalie2022(const TaskT& task) {
	return MakeTask< CGoalie2022 >(task);
}

CPlayerTask* CTaskFactoryV2::ShootoutGoalie(const TaskT& task) {
	return MakeTask< CShootoutGoalie >(task);
}

CPlayerTask* CTaskFactoryV2::ShootoutGoalieV2(const TaskT& task) {
	return MakeTask< CShootoutGoalieV2 >(task);
}

CPlayerTask* CTaskFactoryV2::InterceptBall(const TaskT& task)
{
	return MakeTask< CInterceptBall >(task);
}

CPlayerTask* CTaskFactoryV2::InterceptBallV4(const TaskT& task)
{
	return MakeTask< CInterceptBallV4 >(task);
}

CPlayerTask* CTaskFactoryV2::JustKick(const TaskT& task) {
	return MakeTask<CJustKick>(task);
}

CPlayerTask* CTaskFactoryV2::WaitTouch(const TaskT& task) {
	return MakeTask<CWaitTouch>(task);
}

CPlayerTask* CTaskFactoryV2::PenaltyKick2013(const TaskT& task) {
	return MakeTask<CPenaltyKick2013>(task);
}

CPlayerTask* CTaskFactoryV2::PenaltyKick2014(const TaskT& task) {
	return MakeTask<CPenaltyKick2014>(task);
}

CPlayerTask* CTaskFactoryV2::DribbleTurn(const TaskT& task) {

	return MakeTask<CDribbleTurn>(task);
}

CPlayerTask* CTaskFactoryV2::DribbleTurnKick(const TaskT& task) {
	return MakeTask<CDribbleTurnKick>(task);
}

CPlayerTask* CTaskFactoryV2::DribbleTurnKickV2(const TaskT& task) {
	return MakeTask<CDribbleTurnKickV2>(task);
}

//////////////////////////////////////////////////////////////////////////
// define the namespace used to provide interface for task calling
namespace PlayerRole {
	CPlayerTask* makeItTechDefence(const int num)
	{
		static TaskT playerTask;
		playerTask.executor =num;
		return TaskFactoryV2::Instance()->TechDefenceRobot(playerTask);

	}
	CPlayerTask* makeItTech3Pass(const int num,const int idx)
	{
		static TaskT playerTask;
		playerTask.executor =num;
		return TaskFactoryV2::Instance()->Tech3PassRobot(playerTask);
	}

	CPlayerTask* makeItlightkick(const int num, const double dir, const double power)
	{	
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.angle = dir;
		playerTask.player.kickpower = power;
		return TaskFactoryV2::Instance()->lightkick(playerTask);
	}
	CPlayerTask* makeItBreak(const int num, const double dir, const bool needkick,  const bool isPenalty, const bool isSpin, const CGeoPoint DribblePoint)
	{	// needkick = 1 £ºÄÃÇòÍ»ÆÆ£»target£ºÃ»ÓÃÁË£»
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.needkick = needkick;
		playerTask.player.angle = dir;
		playerTask.player.pos = DribblePoint;
		if (isPenalty)playerTask.player.flag = playerTask.player.flag | PlayerStatus::PENALTY_KICK;
		if (isSpin)playerTask.player.flag = playerTask.player.flag | PlayerStatus::SPIN;

		return TaskFactoryV2::Instance()->Break(playerTask);
	}
	CPlayerTask* makeItSpeedTest(const int num, const CGeoPoint& p1, const CGeoPoint& p2, double v1, double v2, double v_step)
	{
		TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.pos = p1;
		playerTask.ball.pos = p2;
		playerTask.player.speed_x = v1;
		playerTask.player.speed_y = v2;
		playerTask.player.rotvel = v_step;

		CPlayerTask* pTask = TaskFactoryV2::Instance()->SpeedTest(playerTask);
		TaskMediator::Instance()->setPlayerTask(num, pTask, 1);

		return 0;
	}


	CPlayerTask* makeItStop(const int num, const int flags)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.flag = flags;
		return TaskFactoryV2::Instance()->StopRobot(playerTask);
	}
	CPlayerTask* makeItGoto(const int num, const CGeoPoint& target, const double dir, const int flags, const int sender)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.pos = target;
		playerTask.player.vel = CVector(0.0, 0.0);
		playerTask.player.rotvel = 0;
		playerTask.player.angle = dir;
		playerTask.player.flag = flags;
		playerTask.ball.Sender = sender;
		return TaskFactoryV2::Instance()->SmartGotoPosition(playerTask);
	}
	CPlayerTask* makeItGoto(const int num, const CGeoPoint& target, const double dir, const CVector& vel, const double rotvel, const int flags, const int sender)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.pos = target;
		playerTask.player.angle = dir;
		playerTask.player.vel = vel;
		playerTask.player.rotvel = rotvel;
		playerTask.player.flag = flags;
		playerTask.ball.Sender = sender;
		return TaskFactoryV2::Instance()->SmartGotoPosition(playerTask);
	}

	CPlayerTask* makeItGoto(const int num, const CGeoPoint& target, const double dir, const CVector& vel, const double rotVel, const double maxAcc, const double maxRotAcc, const double maxSpeed, const double maxRotSpeed, const int flags) {
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.pos = target;
		playerTask.player.angle = dir;
		playerTask.player.vel = vel;
		playerTask.player.rotvel = rotVel;
		playerTask.player.max_acceleration = maxAcc;
		playerTask.player.max_rot_acceleration = maxRotAcc;
		playerTask.player.max_speed = maxSpeed;
		playerTask.player.max_rot_speed = maxRotSpeed;
		playerTask.player.flag = flags;
		return TaskFactoryV2::Instance()->SmartGotoPosition(playerTask);
	}

	CPlayerTask* makeItSimpleGoto(const int num, const CGeoPoint& target, const double dir, const int flags)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.pos = target;
		playerTask.player.vel = CVector(0.0, 0.0);
		playerTask.player.rotvel = 0;
		playerTask.player.angle = dir;
		playerTask.player.flag = flags;
		return TaskFactoryV2::Instance()->GotoPosition(playerTask);
	}

	CPlayerTask* makeItSimpleGoto(const int num, const CGeoPoint& target, const double dir, const CVector& vel, const double rotvel, const int flags)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.pos = target;
		playerTask.player.angle = dir;
		playerTask.player.vel = vel;
		playerTask.player.rotvel = rotvel;
		playerTask.player.flag = flags;
		return TaskFactoryV2::Instance()->GotoPosition(playerTask);
	}


	CPlayerTask* makeItCrazyPush(const int num, double faceDir)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.angle = faceDir;
		return TaskFactoryV2::Instance()->CarzyPush(playerTask);
	}
	CPlayerTask* makeItRun(const int num, const double speedX, const double speedY, const double rotSpeed, int flags)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.speed_x = speedX;
		playerTask.player.speed_y = speedY;
		playerTask.player.rotate_speed = rotSpeed;
		playerTask.player.flag = flags;
		return TaskFactoryV2::Instance()->Speed(playerTask);
	}

	CPlayerTask* makeItRunLocalVersion(const int num, const double speedX, const double speedY, const double rotSpeed)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.speed_x = speedX;
		playerTask.player.speed_y = speedY;
		playerTask.player.rotate_speed = rotSpeed;
		return TaskFactoryV2::Instance()->OpenSpeed(playerTask);
	}

	CPlayerTask* makeItNoneTrajGetBall(const int num, const double dir, CVector finalVel, int flags, double StopDist, CTRL_METHOD mode)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.angle = dir;
		playerTask.player.vel = finalVel;
		playerTask.player.flag = flags;
		playerTask.player.rotvel = StopDist;
		playerTask.player.specified_ctrl_method = mode;
		return TaskFactoryV2::Instance()->NoneTrajGetBall(playerTask);
	}
	CPlayerTask* makeItNoneTrajGetBallForStatic(const int num, const double dir, CVector finalVel, int flags, double StopDist, CTRL_METHOD mode)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.angle = dir;
		playerTask.player.vel = finalVel;
		playerTask.player.flag = flags;
		playerTask.player.rotvel = StopDist;
		playerTask.player.specified_ctrl_method = mode;
		return TaskFactoryV2::Instance()->NoneTrajGetBallV4(playerTask);
	}
	CPlayerTask* makeItNoneTrajGetBallV3(const int num, const double dir, CVector finalVel, int flags, double StopDist, CTRL_METHOD mode)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.angle = dir;
		playerTask.player.vel = finalVel;
		playerTask.player.flag = flags;
		playerTask.player.rotvel = StopDist;
		playerTask.player.specified_ctrl_method = mode;
		return TaskFactoryV2::Instance()->NoneTrajGetBallV3(playerTask);
	}

	CPlayerTask* makeItGoAndTurnKickV4(const int num, const double dir, CVector finalVel, int flags, double StopDist, CTRL_METHOD mode)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.angle = dir;
		playerTask.player.vel = finalVel;
		playerTask.player.flag = flags;
		playerTask.player.rotvel = StopDist;
		playerTask.player.specified_ctrl_method = mode;
		return TaskFactoryV2::Instance()->GoAndTurnKickV4(playerTask);
	}
	CPlayerTask* makeItForceStartRush(const int num, double faceDir, int flags)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.angle = faceDir;
		playerTask.player.flag = flags;
		return TaskFactoryV2::Instance()->ForceStartRush(playerTask);
	}
	CPlayerTask* makeItChaseKickV1(const int num, double faceDir, int flags)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.angle = faceDir;
		playerTask.player.flag = flags;
		return TaskFactoryV2::Instance()->ChaseKickV1(playerTask);
	}

	CPlayerTask* makeItChaseKickV2(const int num, double faceDir, int flags, int power)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.angle = faceDir;
		playerTask.player.flag = flags;
		playerTask.player.kickpower = power;
		return TaskFactoryV2::Instance()->ChaseKickV2(playerTask);
	}
	CPlayerTask* makeItChaseToGetBall(const int num, double faceDir, int flags, int isneedkick, int power)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.angle = faceDir;
		playerTask.player.flag = flags;
		playerTask.player.needkick = isneedkick;
		playerTask.player.kickpower = power;
		return TaskFactoryV2::Instance()->ChaseToGetBall(playerTask);
	}

	CPlayerTask* makeItProtectBall(const int num, const int flags)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.flag = flags;
		return TaskFactoryV2::Instance()->ProtectBall(playerTask);
	}

	CPlayerTask* makeItTouchKick(const int num, double kickDir, bool isPass, int flags)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.angle = kickDir;
		playerTask.player.ispass = isPass;
		playerTask.player.flag = flags;
		return TaskFactoryV2::Instance()->TouchKick(playerTask);
	}

	CPlayerTask* makeItInterKickV2(const int num, const double dir, const int flags, const bool kickway)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.angle = dir;
		playerTask.player.flag = flags;
		playerTask.player.ischipkick = kickway;


		//        playerTask.player.pos = target;
		return TaskFactoryV2::Instance()->InterceptBallV2(playerTask);
	}


	CPlayerTask* makeItInterKickV3(const int num, const double dir, const int flags, const bool kickway)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.angle = dir;
		playerTask.player.flag = flags;
		playerTask.player.ischipkick = kickway;

		//        playerTask.player.pos = target;
		return TaskFactoryV2::Instance()->InterceptBallV3(playerTask);
	}

	CPlayerTask* makeItInterKickV4(const int num, const double dir, const bool kickway, const double ipower, const int flags)        //??C++???žD?????InterceptBall
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.angle = dir;
		playerTask.player.flag = flags;
		playerTask.player.ischipkick = kickway;
		playerTask.player.kickpower = ipower;
		playerTask.player.chipkickpower = ipower;
		//        playerTask.player.pos = target;
		return TaskFactoryV2::Instance()->InterceptBallV4(playerTask);
	}


	CPlayerTask* makeItMarkingTouch(const int num, const double kickDir, const CGeoPoint leftUpPos, const CGeoPoint rightDownPos, const int flags)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.angle = kickDir;
		playerTask.player.flag = flags;
		playerTask.ball.pos = leftUpPos;
		playerTask.player.pos = rightDownPos;
		return TaskFactoryV2::Instance()->MarkingTouch(playerTask);
	}

	CPlayerTask* makeItMarkingFront(const int num, const int markNum, const double kickDir, const int flags)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.ball.Sender = markNum;
		playerTask.player.angle = kickDir;
		playerTask.player.flag = flags;
		return TaskFactoryV2::Instance()->MarkingFront(playerTask);
	}

	CPlayerTask* makeItDriftKick(const int num, double faceDir, int flags)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.angle = faceDir;
		playerTask.player.flag = flags;
		return TaskFactoryV2::Instance()->DriftKick(playerTask);
	}

	CPlayerTask* makeItAdvanceBallV1(const int num, const int flags)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.flag = flags;
		return TaskFactoryV2::Instance()->AdvanceBallV1(playerTask);
	}
	CPlayerTask* makeItAdvanceBallV2(const int num, const int flags, const int tendemNum)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.flag = flags;
		playerTask.ball.receiver = tendemNum;
		return TaskFactoryV2::Instance()->AdvanceBallV2(playerTask);
	}
	CPlayerTask* makeItAdvanceBallV3(const int num, const int flags, const int tendemNum)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.flag = flags;
		playerTask.ball.receiver = tendemNum;
		return TaskFactoryV2::Instance()->AdvanceBallV3(playerTask);

	}
	CPlayerTask* makeItAdvanceBallV4(const int num, const int flags, const int tendemNum)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.flag = flags;
		playerTask.ball.receiver = tendemNum;
		return TaskFactoryV2::Instance()->AdvanceBallV4(playerTask);

	}
	CPlayerTask* makeItAdvance(const int num, const int flags, const int tendemNum)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.flag = flags;
		playerTask.ball.receiver = tendemNum;
		return TaskFactoryV2::Instance()->Advance(playerTask);

	}
	CPlayerTask* makeItPenaltyKickV2(const int num, const int flags, const int tendemNum)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.flag = flags;
		playerTask.ball.receiver = tendemNum;
		return TaskFactoryV2::Instance()->PenaltyKickV2(playerTask);
	}
	CPlayerTask* makeItTandem(const int num, const int robber, const int flags)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.ball.receiver = robber;
		playerTask.player.flag = flags;
		return TaskFactoryV2::Instance()->Tandem(playerTask);
	}
	CPlayerTask* makeItGoAndTurnKick(const int num, const double targetdir, const int itype, const int power)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.angle = targetdir;
		playerTask.player.flag = itype;
		playerTask.player.chipkickpower = power;
		return TaskFactoryV2::Instance()->GoAndTurnKick(playerTask);
	}
	CPlayerTask* makeItCircleAndPass(const int num, const double targetdir, const int itype, const int power)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.angle = targetdir;
		playerTask.player.flag = itype;
		playerTask.player.chipkickpower = power;
		return TaskFactoryV2::Instance()->CircleAndPass(playerTask);
	}
	CPlayerTask* makeItTestCircleBall(const int num, const double targetdir, const double kickprecision, const int flags)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.angle = targetdir;
		playerTask.player.flag = flags;
		playerTask.player.kickprecision = kickprecision;
		return TaskFactoryV2::Instance()->TestCircleBall(playerTask);
	}
	CPlayerTask* makeItGoAndTurnKickV3(const int num, const double targetdir, int circleNum, const double fixAngle, const double maxAcc, const int radius, const int numPerCir, const double gotoPre, const double gotoDist, const double adjustPre, const double kickprecision, const int flags)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.angle = targetdir;
		playerTask.player.flag = flags;
		playerTask.ball.angle = circleNum;
		playerTask.player.kickprecision = kickprecision;
		playerTask.player.rotate_speed = fixAngle;
		playerTask.player.max_acceleration = maxAcc;
		playerTask.player.rotvel = radius;
		playerTask.player.chipkickpower = numPerCir;
		playerTask.player.kickpower = gotoPre;
		playerTask.player.max_deceleration = gotoDist;
		playerTask.player.speed_x = adjustPre;
		return TaskFactoryV2::Instance()->GoAndTurnKickV3(playerTask);
	}
	CPlayerTask* makeItGoAndTurn(const int num, const double targetdir, const double kickprecision, const int flags)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.angle = targetdir;
		playerTask.player.flag = flags;
		playerTask.player.kickprecision = kickprecision;
		return TaskFactoryV2::Instance()->GoAndTurn(playerTask);
	}
	CPlayerTask* makeItGoAroundRobot(const int num, const double faceDir, const CGeoPoint circleCenter, int circleDir, double radius, int flag)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.angle = faceDir;
		playerTask.player.rotdir = circleDir;
		playerTask.player.pos = circleCenter;
		playerTask.player.rotvel = radius;
		return TaskFactoryV2::Instance()->GoAroundRobot(playerTask);
	}
	CPlayerTask* makeItGoTechChalPos(const int num, const int flag)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.flag = flag;
		return TaskFactoryV2::Instance()->GoTechChalPos(playerTask);
	}
	CPlayerTask* makeItSlowGetBall(const int num, const double dir, const int flags)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.angle = dir;
		playerTask.player.flag = flags;
		return TaskFactoryV2::Instance()->SlowGetBall(playerTask);
	}
	CPlayerTask* makeItTimeDelayTest(const int num, const int flags)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.flag = flags;
		return TaskFactoryV2::Instance()->timeDelayTest(playerTask);
	}
	CPlayerTask* makeItMarkEnemy(const int num, const int enemy, const bool front, const int flags, const CGeoPoint pos, const double dir)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.ball.Sender = enemy;
		playerTask.ball.front = front;
		playerTask.player.flag = flags;
		playerTask.player.pos = pos;
		playerTask.player.angle = dir;
		return TaskFactoryV2::Instance()->Marking(playerTask);
	}
	CPlayerTask* makeItGoSupport(const int num, bool leftOrRight)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.left_or_right = leftOrRight;
		return TaskFactoryV2::Instance()->GoSupport(playerTask);
	}
	CPlayerTask* makeItGoPIDCircle(const int runner, const CGeoPoint pos, const double r, const bool opt)
	{
		static TaskT playerTask;
		playerTask.executor = runner;
		playerTask.player.left_or_right = opt;
		playerTask.player.pos = pos;
		playerTask.player.GoRadius = r;
		return TaskFactoryV2::Instance()->GoPIDCircle(playerTask);
	}
	CPlayerTask* makeItShootBall(const int num, const double dir, const bool ischipkick, const double precision, const double kp, const double cp, const int flags)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.ischipkick = ischipkick;
		playerTask.player.angle = dir;
		playerTask.player.kickprecision = precision;
		playerTask.player.kickpower = kp;
		playerTask.player.chipkickpower = cp;
		playerTask.player.flag = flags;
		return TaskFactoryV2::Instance()->ShootBall(playerTask);
	}
	CPlayerTask* makeItShootBallV2(const int num, const double dir, const int flags)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.angle = dir;
		playerTask.player.flag = flags;
		return TaskFactoryV2::Instance()->ShootBallV2(playerTask);
	}
	CPlayerTask* makeItReceivePass(const int num, const double dir, const int flags)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.angle = dir;
		playerTask.player.flag = flags;
		return TaskFactoryV2::Instance()->ReceivePass(playerTask);
	}
	CPlayerTask* makeItFetchBall(const int num, const CGeoPoint target, double kickPower, const double angle)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.ball.pos = target;
		playerTask.player.kickpower = kickPower;
		playerTask.player.angle = angle;
		return TaskFactoryV2::Instance()->FetchBall(playerTask);
	}
	CPlayerTask* makeItPenaltyGoalie(const int num, const int flags)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.flag = flags;
		return TaskFactoryV2::Instance()->PenaltyDefV1(playerTask);
		//    return TaskFactoryV2::Instance()->PenaltyGoalie(playerTask);
	}

	CPlayerTask* makeItGoalie2013(const int num, const int flags) {
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.flag = flags;
		return TaskFactoryV2::Instance()->Goalie2013(playerTask);
	}

	CPlayerTask* makeItGoalie2022(const int num, const bool isPenalty, const int flags) {
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.isPenalty = isPenalty;
		playerTask.player.flag = flags;
		return TaskFactoryV2::Instance()->Goalie2022(playerTask);
	}

	CPlayerTask* makeItShootoutGoalie(const int num, const int flags) {
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.flag = flags;
		return TaskFactoryV2::Instance()->ShootoutGoalie(playerTask);
	}

	CPlayerTask* makeItShootoutGoalieV2(const int num, const int flags) {
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.flag = flags;
		return TaskFactoryV2::Instance()->ShootoutGoalieV2(playerTask);
	}

	CPlayerTask* makeItInterceptBall(const int num, const double add_angle)
	{
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.angle = add_angle;
		return TaskFactoryV2::Instance()->InterceptBall(playerTask);
	}

	CPlayerTask* makeItJustKick(const int num, const int mode, const double power, const int flag) {
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.ischipkick = mode;
		playerTask.player.kickpower = power;
		playerTask.player.flag = flag;
		return TaskFactoryV2::Instance()->JustKick(playerTask);
	}

	CPlayerTask* makeItWaitTouch(const int num, const CGeoPoint& target, const double dir, const double angle, const int flag /* = 0 */) {
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.ball.pos = target;
		playerTask.ball.angle = angle;
		playerTask.player.angle = dir;
		playerTask.player.flag = flag;
		return TaskFactoryV2::Instance()->WaitTouch(playerTask);
	}

	CPlayerTask* makeItPenaltyKick2013(const int num, const int flag /* = 0 */) {
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.flag = flag;
		return TaskFactoryV2::Instance()->PenaltyKick2013(playerTask);
	}

	CPlayerTask* makeItPenaltyKick2014(const int num, const int flag /* = 0 */) {
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.flag = flag;
		return TaskFactoryV2::Instance()->PenaltyKick2014(playerTask);
	}

	CPlayerTask* makeItDribbleTurn(const int num, const double finalDir, const double pre) {
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.angle = finalDir;
		playerTask.player.speed_x = pre;
		return TaskFactoryV2::Instance()->DribbleTurn(playerTask);
	}

	CPlayerTask* makeItDribbleTurnKick(const int num, const double finalDir, const double turnRotVel, const double kickPower) {
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.angle = finalDir;
		playerTask.player.rotvel = turnRotVel;
		playerTask.player.kickpower = kickPower;
		return TaskFactoryV2::Instance()->DribbleTurnKick(playerTask);
	}

	CPlayerTask* makeItDribbleTurnKickV2(const int num, const double finalDir, const double precision, const int mode, const double power, const CGeoPoint Target) {
		static TaskT playerTask;
		playerTask.executor = num;
		playerTask.player.angle = finalDir;
		playerTask.player.kickprecision = precision;
		playerTask.player.ischipkick = mode;
		playerTask.player.kickpower = power;
		playerTask.player.pos = Target;

		return TaskFactoryV2::Instance()->DribbleTurnKickV2(playerTask);
	}

}
