#ifndef _TASK_FACTORY_V2_H_
#define _TASK_FACTORY_V2_H_

#include <singleton.h>
#include <misc_types.h>
#include "param.h"
/************************************************************************/
/*                         CTaskFactoryV2                               */
/************************************************************************/

class CPlayerTask;
class CTaskFactoryV2 {
public:
	// CPlayerTask* CtaskFactoryV2::GofourRobot(const TaskT& task);
	CPlayerTask* TechDefenceRobot(const TaskT& task);
	CPlayerTask* Tech3PassRobot(const TaskT& task);
	CPlayerTask* GoPIDCircle(const TaskT& task);
	CPlayerTask* Break(const TaskT& task);
	CPlayerTask* lightkick(const TaskT& task);
	//跑位的Skill. by HXY
	CPlayerTask* SpeedTest(const TaskT& task);
	CPlayerTask* GotoPosition(const TaskT& task);
	CPlayerTask* SmartGotoPosition(const TaskT& task);
	CPlayerTask* GotoPositionNew(const TaskT& task);

	CPlayerTask* GoAroundRobot(const TaskT& task);
	CPlayerTask* GoTechChalPos(const TaskT& task);
	//进攻的Skill. by HXY
	CPlayerTask* ChaseKickV1(const TaskT& task);
	CPlayerTask* ChaseKickV2(const TaskT& task);
	CPlayerTask* ChaseToGetBall(const TaskT& task);
	
	CPlayerTask* AdvanceBallV1(const TaskT& task);
	CPlayerTask* AdvanceBallV2(const TaskT& task);
	CPlayerTask* AdvanceBallV3(const TaskT& task);
	CPlayerTask* AdvanceBallV4(const TaskT& task);
	CPlayerTask* Advance(const TaskT& task);
	CPlayerTask* PenaltyKickV2(const TaskT& task);
	CPlayerTask* GoSupport(const TaskT& task);

	//射门的Skill. by HXY
	CPlayerTask* ShootBall(const TaskT& task);
	CPlayerTask* ShootBallV2(const TaskT& task);

	CPlayerTask* DriftKick(const TaskT& task);
	CPlayerTask* DribbleTurnKick(const TaskT& task);
	CPlayerTask* DribbleTurnKickV2(const TaskT& task);

	CPlayerTask* JustKick(const TaskT& task);

	CPlayerTask* WaitTouch(const TaskT& task);
	CPlayerTask* TouchKick(const TaskT& task);

	//拿球的Skill. by HXY
	CPlayerTask* SlowGetBall(const TaskT& task);
	CPlayerTask* NoneTrajGetBall(const TaskT& task);

	CPlayerTask* NoneTrajGetBallV4(const TaskT& task);
	CPlayerTask* NoneTrajGetBallV3(const TaskT& task);

	//转身的Skill. by HXY
	CPlayerTask* DribbleTurn(const TaskT& task);
	CPlayerTask* GoAndTurn(const TaskT& task);
	CPlayerTask* GoAndTurnKick(const TaskT& task);
	CPlayerTask* GoAndTurnKickV3(const TaskT& task);
	CPlayerTask* GoAndTurnKickV4(const TaskT& task);

	//放球的Skill. by HXY
	CPlayerTask* FetchBall(const TaskT& task);

	//传球的Skill. by HXY
	CPlayerTask* PassBall(const TaskT& task);
	CPlayerTask* ReceivePass(const TaskT& task);

	//点球的Skill. by HXY
	CPlayerTask* PenaltyDefV1(const TaskT& task);
	CPlayerTask* PenaltyDefV2(const TaskT& task);
	CPlayerTask* PenaltyKick2013(const TaskT& task);
	CPlayerTask* PenaltyKick2014(const TaskT& task);

	//盯人的Skill. by HXY
	CPlayerTask* Marking(const TaskT& task);
	CPlayerTask* MarkingTouch(const TaskT& task);
	CPlayerTask* MarkingFront(const TaskT& task);

	//守门的Skill. by HXY
	CPlayerTask* Goalie2013(const TaskT& task);
	CPlayerTask* Goalie2022(const TaskT& task);

	//其他的Skill. by HXY
	CPlayerTask* InterceptBall(const TaskT& task);
	CPlayerTask* InterceptBallV2(const TaskT& task);
	CPlayerTask* InterceptBallV3(const TaskT& task);
	CPlayerTask* InterceptBallV4(const TaskT& task);
	CPlayerTask* StopRobot(const TaskT& task);
	CPlayerTask* Speed(const TaskT& task); // by zc
	CPlayerTask* OpenSpeed(const TaskT& task);
	CPlayerTask* ForceStartRush(const TaskT& task);
	CPlayerTask* ProtectBall(const TaskT& task);

	CPlayerTask* Tandem(const TaskT& task);
	CPlayerTask* CircleAndPass(const TaskT& task);

	CPlayerTask* timeDelayTest(const TaskT& task);
	CPlayerTask* TestCircleBall(const TaskT& task);
	CPlayerTask* CarzyPush(const  TaskT& task);

	CPlayerTask* PenaltyGoalie(const TaskT& task);
	CPlayerTask* ShootoutGoalie(const TaskT& task);
	CPlayerTask* ShootoutGoalieV2(const TaskT& task);

protected:
	template < class CTaskType >
	CPlayerTask* MakeTask(const TaskT& task);
};
typedef Falcon::NormalSingleton<CTaskFactoryV2> TaskFactoryV2;
/************************************************************************/
/*                 一些包裹(wrap)函数, 方便调用tasks                    */
/************************************************************************/

//! 调用skill的接口
namespace PlayerRole {
	CPlayerTask* makeItTechDefence(const int num);
	CPlayerTask* makeItTech3Pass(const int num,const int idx);
	CPlayerTask* makeItlightkick(const int num, const double dir = 0, const double power = 0);
	CPlayerTask* makeItBreak(const int num, const double dir, const bool needkick = true, const bool isPenalty = false, const bool isSpin = false, const CGeoPoint DribblePoint = CGeoPoint(-1000, -1000));
	CPlayerTask* makeItStop(const int num, const int flags = 0);
	CPlayerTask* makeItGoto(const int num, const CGeoPoint& target, const double dir, const int flags = 0, const int sender = 0);
	CPlayerTask* makeItGoto(const int num, const CGeoPoint& target, const double dir, const CVector& vel, const double rotvel, const int flags = 0, const int sender = 0);
	CPlayerTask* makeItSimpleGoto(const int num, const CGeoPoint& target, const double dir, const int flags = 0);
	CPlayerTask* makeItSimpleGoto(const int num, const CGeoPoint& target, const double dir, const CVector& vel, const double rotvel, const int flags = 0);
	CPlayerTask* makeItSpeedTest(const int num, const CGeoPoint& p1, const CGeoPoint& p2, double v1, double v2, double v_step);
	CPlayerTask* makeItInterKickV2(const int num, const double dir, const int flags, const bool kickway = 0);
	CPlayerTask* makeItInterKickV3(const int num, const double dir, const int flags, const bool kickway = 0);
	CPlayerTask* makeItInterKickV4(const int num, const double dir, const bool kickway = 0, const double ipower = 9999, const int flags = 0);
	CPlayerTask* makeItRun(const int num, const double speedX, const double speedY, const double rotSpeed, int flags = 0); // 直接发速度,by zc
	CPlayerTask* makeItRunLocalVersion(const int num, const double speedX, const double speedY, const double rotSpeed);  // 发开环的小车坐标系速度
	CPlayerTask* makeItCrazyPush(const int num, double faceDir);
	CPlayerTask* makeItNoneTrajGetBall(const int num, const double dir, CVector finalVel = CVector(0.0, 0.0), int flags = 0, double StopDist = -2, CTRL_METHOD mode = CMU_TRAJ);
	CPlayerTask* makeItNoneTrajGetBallForStatic(const int num, const double dir, CVector finalVel = CVector(0.0, 0.0), int flags = 0, double StopDist = -2, CTRL_METHOD mode = CMU_TRAJ);
	CPlayerTask* makeItNoneTrajGetBallV3(const int num, const double dir, CVector finalVel = CVector(0.0, 0.0), int flags = 0, double StopDist = -2, CTRL_METHOD mode = CMU_TRAJ);
	CPlayerTask* makeItGoAndTurnKickV4(const int num, const double dir, CVector finalVel = CVector(0.0, 0.0), int flags = 0, double StopDist = -2, CTRL_METHOD mode = CMU_TRAJ);
	CPlayerTask* makeItForceStartRush(const int num, double faceDir, int flags = 0);
	CPlayerTask* makeItChaseKickV1(const int num, double faceDir, int flags = 0);
	CPlayerTask* makeItChaseKickV2(const int num, double faceDir, int flags = 0, int power = 0);
	CPlayerTask* makeItChaseToGetBall(const int num, double faceDir, int flags, int isneedkick = 0, int power = 650);

	CPlayerTask* makeItDriftKick(const int num, double faceDir, int flags = 0);
	CPlayerTask* makeItProtectBall(const int num, int flags = 0);
	CPlayerTask* makeItTouchKick(const int num, double kickDir, bool isPass = false, int flags = 0);
	CPlayerTask* makeItMarkingTouch(const int num, const double kickDir, const CGeoPoint leftUpPos, const CGeoPoint rightDownPos, const int flags);
	CPlayerTask* makeItMarkingFront(const int num, const int markNum, const double kickDir, const int flags);
	CPlayerTask* makeItAdvanceBallV1(const int num, const int flags = 0);
	CPlayerTask* makeItAdvanceBallV2(const int num, const int flags = 0, const int tendemNum = 0);
	CPlayerTask* makeItAdvanceBallV3(const int num, const int flags = 0, const int tendemNum = 0);
	CPlayerTask* makeItAdvanceBallV4(const int num, const int flags = 0, const int tendemNum = 0);
	CPlayerTask* makeItAdvance(const int num, const int flags = 0, const int tendemNum = 0);
	CPlayerTask* makeItPenaltyKickV2(const int num, const int flags = 0, const int tendemNum = 0);
	CPlayerTask* makeItTandem(const int num, const int robber = 1, const int flags = 0);
	CPlayerTask* makeItGoAndTurnKick(const int num, const double targetdir, const int itype, const int power);
	CPlayerTask* makeItCircleAndPass(const int num, const double targetdir, const int itype, const int power);
	CPlayerTask* makeItGoAndTurnKickV3(const int num, const double targetdir, int circleNum, const double fixAngle, const double maxAcc, const int radius, const int numPerCir, const double gotoPre, const double gotoDist, const double adjustPre, const double kickprecision = Param::Math::PI * 5 / 180, const int flags = 0);

	CPlayerTask* makeItGoAndTurn(const int num, const double targetdir, const double kickprecision = Param::Math::PI * 5 / 180, const int flags = 0);
	CPlayerTask* makeItTestCircleBall(const int num, const double targetdir, const double kickprecision = Param::Math::PI * 5 / 180, const int flags = 0);
	CPlayerTask* makeItGoAroundRobot(const int num, const double faceDir, const CGeoPoint circleCenter, int circleDir, double radius = 25, int flags = 0);

	CPlayerTask* makeItGoTechChalPos(const int num, const int flag);

	CPlayerTask* makeItSlowGetBall(const int num, const double dir, const int flags = 0);
	CPlayerTask* makeItTimeDelayTest(const int num, const int flags = 0);
	CPlayerTask* makeItMarkEnemy(const int num, const int enemy, const bool front = false, const int flags = 0, const CGeoPoint pos = CGeoPoint(1000, 1000), const double dir = 999);
	CPlayerTask* makeItShootBall(const int num, const double dir, const bool ischipkick, const double precision, const double kp, const double cp, const int flags = 0);
	CPlayerTask* makeItShootBallV2(const int num, const double dir, const int flags = 0);
	CPlayerTask* makeItReceivePass(const int num, const double dir, const int flags = 0);
	CPlayerTask* makeItFetchBall(const int num, const CGeoPoint target, double kickPower, const double angle);
	CPlayerTask* makeItPenaltyGoalie(const int num, const int flags = 0);
	CPlayerTask* makeItGoalie2013(const int num, const int flag = 0);
	CPlayerTask* makeItGoalie2022(const int num, const bool isPenalty, const int flag = 0);
	CPlayerTask* makeItShootoutGoalie(const int num, const int flag = 0);
	CPlayerTask* makeItShootoutGoalieV2(const int num, const int flag = 0);
	CPlayerTask* makeItInterceptBall(const int num, const double add_angle);
	CPlayerTask* makeItJustKick(const int num, const int mode = 0, const double power = 550, const int flag = 0);
	CPlayerTask* makeItWaitTouch(const int num, const CGeoPoint& target, const double dir, const double angle, const int flag = 0);
	CPlayerTask* makeItPenaltyKick2013(const int num, const int flag = 0);
	CPlayerTask* makeItPenaltyKick2014(const int num, const int flag = 0);
	CPlayerTask* makeItDribbleTurn(const int num, const double finalDir, const double pre = Param::Math::PI / 90);
	CPlayerTask* makeItDribbleTurnKick(const int num, const double finalDir, const double turnRotVel, const double kickPower);
	CPlayerTask* makeItDribbleTurnKickV2(const int num, const double finalDir = 0, const double precision = 5 * Param::Math::PI / 180, const int mode = 0, const double power = 550, const CGeoPoint Target = CGeoPoint(0, 0));
	CPlayerTask* makeItGoSupport(const int num, const bool leftOrRight);
	CPlayerTask* makeItGoPIDCircle(const int runner, const CGeoPoint pos, const double r, const bool opt);
}

#endif // _TASK_FACTORY_V2_H_
