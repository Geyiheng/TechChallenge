#include "GetBallV3.h"
#include "GDebugEngine.h"
#include <Vision/VisionModule.h>
#include "skill/Factory.h"
#include <utils.h>
#include <WorldModel/DribbleStatus.h>
#include "BallSpeedModel.h"
#include <RobotSensor.h>
#include <WorldModel/KickStatus.h>

namespace {
	// 拿球的主状态机
	enum get_ball_state {
		DIRECTGOTO = 1,
		GETBALL,
		AVOIDBALL
	};
	// AVOIDBALL状态下的避球状态分布
	enum avoid_ball_state {
		NOAVOID = 1,
		BALLBEHINDME,
		BALLBESIDEME
		//待添加
	};
	avoid_ball_state ab_state = NOAVOID;
	// GETBALL的状态分布
	enum getball_state {
		SMALLANGLE = 1,
		LARGEANGLE
	};
	getball_state gb_state = LARGEANGLE;
	// 状态执行查看器 TODO

	// 需要用到的常量
	const double newVehicleBuffer = 0.6;               // 小嘴巴机器人PLAYER_FRONT_TO_CENTER补偿
	const double ballPredictBaseTime = 0.2;            // 球预测的基础时间
	const double SpeedLimitForPredictTime = 120;       // 球在该速度以上则加大预测时间
	const double nearBallRadius = 12;                  // 小车半径+球半径+2~3视频误差裕度,判定是否需要AVOIDBALL的半径
	double ball2myheadLimit = 3;                       // 小嘴巴车嘴长7cm，球半径2cm，想要稳定拿住球需要(7-2)/2=2.5cm 再加1.0的余值
	const double directGetBallDist = 20;               // 直接冲上去拿球的距离
	const double avoidBallSuccessDist = 25;            // 判定避球成功的距离 25
	const double minGetBallDist = 17;                  // 最小拿球距离 17
	const double maxGetBallDist = 15;				   // 最大拿球距离 25
	const double AllowFaceToFinalDist = 200;           // 进入该距离后开始转向
	const double transverseBallSpeed = 20;             // 对拿球产生影响的最低横向球速 ori：30
	const bool Allow_Start_Dribble = true;             // 控球控制阈值
	const double DribbleLevel = 3;                     // 控球力度等级	
	const double extremeAngle = Param::Math::PI/* * 176 / 180.0*/;
	const double directGetBballDirLimit = Param::Math::PI / 20.0;
	const int maxFrared = 125;	//红外极大值

	// 开关量
	bool DEBUG_ENGINE = false;                          // 调试模式
	bool IS_DRIBBLE = false;                            // 吸球
	const bool USE_BALL_SPEED_MODEL = false;            // 应用球速模型

	//用到的静态变量
	bool trueNeedAvoidBall = false;
	int avoidBallCount = 0;
	int fraredOn = 0;
	int fraredOff = 0;
	double GETBALL_BIAS = -2.5;
	double BALL_NEAR_ROBOT = 20;

    double roll_acc = 200;
    double slide_acc = 500;
    double transition_speed = 400;

    double HEAD_LIMIT = 3;
}

CGetBallV3::CGetBallV3()
{
    GETBALL_BIAS = ParamManager::Instance()->GETBALL_BIAS;
    BALL_NEAR_ROBOT = ParamManager::Instance()->BALL_NEAR_ROBOT;
    HEAD_LIMIT = ParamManager::Instance()->HEAD_LIMIT;
    DEBUG_ENGINE = ParamManager::Instance()->GetBall_Debug;
    IS_DRIBBLE = ParamManager::Instance()->IS_DRIBBLE;

    roll_acc = ParamManager::Instance()->roll_acc;
    slide_acc = ParamManager::Instance()->slide_acc;
    transition_speed = ParamManager::Instance()->transition_speed;

	_lastCycle = 0;
}

void CGetBallV3::plan(const CVisionModule* pVision)
{
	ball2myheadLimit = HEAD_LIMIT;
	// 内部状态进行重置
	if (pVision->Cycle() - _lastCycle > Param::Vision::FRAME_RATE * 0.1) {
		setState(BEGINNING);
		ab_state = NOAVOID;
		gb_state = LARGEANGLE;
		trueNeedAvoidBall = false;
		avoidBallCount = 0;
	}

	/********************************************************************/
	/* 视觉初步处理  by lsp */
	/********************************************************************/
	const BallVisionT& ball = pVision->Ball();
	const int robotNum = task().executor;
	const PlayerVisionT& me = pVision->OurPlayer(robotNum);
	const CGeoPoint myhead = me.Pos() + Utils::Polar2Vector(Param::Vehicle::V2::PLAYER_FRONT_TO_CENTER + newVehicleBuffer, me.Dir());
	const CVector self2ball = ball.Pos() - me.Pos();
	const CVector ball2self = me.Pos() - ball.Pos();
	const CVector head2ball = ball.Pos() - myhead;

	const double StopDist = task().player.rotvel;
//	cout << "StopDist:\t" << StopDist << endl;
	const double finalDir = task().player.angle;
	const double reverse_finalDir = Utils::Normalize(finalDir + Param::Math::PI);
	
	// 是否开启debug模式
	if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_line(me.Pos(), me.Pos() + Utils::Polar2Vector(1000, finalDir), COLOR_RED);
	if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_line(me.Pos(), me.Pos() + Utils::Polar2Vector(1000, self2ball.dir()), COLOR_PURPLE);

	/********************************************************************/
	/* 球的预测时间  by lsp */
	/********************************************************************/
	double BallPosWithVelFactorTmp = ballPredictBaseTime;

	double dAngle_ball2myhead_ballvel = Utils::Normalize(Utils::Normalize(head2ball.dir() + Param::Math::PI) - ball.Vel().dir());
	double dAngle_self2ball_medir = Utils::Normalize(self2ball.dir() - me.Dir());
	double dAngle_finalDir2ballVel = Utils::Normalize(finalDir - ball.Vel().dir());

	// 球在车正前方对球预测时间的影响，考虑红外
	bool frared = RobotSensor::Instance()->IsInfraredOn(robotNum);

	// 带有小缓存功能的红外
	if (frared) {
		fraredOn = fraredOn >= maxFrared ? maxFrared : fraredOn + 1;
		fraredOff = 0;
	}
	else {
		fraredOn = 0;
		fraredOff = fraredOff >= maxFrared ? maxFrared : fraredOff + 1;
	}
	
	CVector ballVel = ball.Vel();
//	// 如果球速过大并且背向拿球，加大预测
//	if (ball.Vel().mod() > SpeedLimitForPredictTime && fabs(dAngle_finalDir2ballVel) > Param::Math::PI * 2 / 3) {
//		BallPosWithVelFactorTmp += 0.2;
//	}

//	if (fabs(dAngle_self2ball_medir) < Param::Vehicle::V2::KICK_ANGLE) {
//		if (fabs(dAngle_ball2myhead_ballvel) < Param::Math::PI / 6.0) {                      // 球朝车头滚过来,预测时间减小
//			BallPosWithVelFactorTmp *= sin(fabs(dAngle_ball2myhead_ballvel));
//		}
//		else if (fabs(dAngle_ball2myhead_ballvel) > Param::Math::PI * 5 / 6.0 &&
//			fabs(Utils::Normalize(ball.Vel().dir() - finalDir)) < Param::Math::PI / 6.0)               // 车追球，预测时间增大
//		{
//			if (fraredOn < 20)
//			{
//				BallPosWithVelFactorTmp += 0.2;
//				BallPosWithVelFactorTmp *= cos(fabs(dAngle_ball2myhead_ballvel)) * -1;
//			}
//			else {
//				//红外出现时特殊处理：将球速大小预测值改为0
//				ballVel = Utils::Polar2Vector(0.01, ball.Vel().dir());
//			}
//		}
//	}

	/********************************************************************/
	/* 预测BallPosWithVelFactorTmp时间之后的球位置  by lsp */
	/********************************************************************/
	CGeoPoint ballPosWithVel = CGeoPoint(0, 0);
    if (me.Pos().dist(ball.Pos()) < BALL_NEAR_ROBOT || ballVel.mod() < 70) {
        //if (ballVel.mod() < 20) {
        ballPosWithVel = ball.Pos();
        GDebugEngine::Instance()->gui_debug_msg(ballPosWithVel, "too slow ball");
        /*}
        else {
            CGeoSegment medir = CGeoSegment(me.Pos(), me.Pos() + me.Vel());
            ballPosWithVel = medir.projection(ball.Pos() + ballVel);
        }*/
    }
    else {
        // 粗略计算预测球的位置
        // 修改了拿球的预测时间，从三个角度考虑，车到球运动轨迹的距离，车的朝向，车的运动速度
        CGeoLine ball_line(ball.Pos(), ball.Pos() + ballVel);
        CGeoPoint projection_point = ball_line.projection(me.Pos());
        double projection2me_dist = projection_point.dist(me.Pos());
        double angle_diff = Param::Math::PI - abs(abs(me.Dir()) - abs(ballVel.dir()));
        if (me.Vel().mod() > 20) {
            double vel_dir = me.Vel().dir();
            double vel_dir_diff = abs(vel_dir - (projection_point - me.Pos()).dir());
            BallPosWithVelFactorTmp += (vel_dir_diff - Param::Math::PI/2) / 5;
            GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, 0), QString("vel dir diff time:%5").arg((vel_dir_diff - Param::Math::PI/2) / 5).toStdString().c_str());
        }

        BallPosWithVelFactorTmp += projection2me_dist / 500;
        BallPosWithVelFactorTmp += angle_diff / 15;

        // 粗略计算预测球的位置
        if (ballVel.mod() < transition_speed) {
            if (ballVel.mod() / roll_acc < BallPosWithVelFactorTmp) {
                ballPosWithVel = ball.Pos() + ballVel.unit() * (ballVel.mod() * BallPosWithVelFactorTmp  - 0.5 * roll_acc * BallPosWithVelFactorTmp * BallPosWithVelFactorTmp);
                GDebugEngine::Instance()->gui_debug_msg(ballPosWithVel, QString("short roll ball, time:%5").arg(BallPosWithVelFactorTmp).toStdString().c_str());
            }
            else {
                ballPosWithVel = ball.Pos() + ballVel.unit() * (ballVel.mod2() / roll_acc * 0.5);
                GDebugEngine::Instance()->gui_debug_msg(ballPosWithVel, QString("long roll ball, time:%5").arg(BallPosWithVelFactorTmp).toStdString().c_str());
            }
        }
        else {
            float time2roll = (ballVel.mod() - transition_speed) / slide_acc;
            float timeslide = transition_speed / roll_acc;
            if (BallPosWithVelFactorTmp < time2roll) {
                ballPosWithVel = ball.Pos() + ballVel.unit() * (ballVel.mod() * BallPosWithVelFactorTmp - 0.5 * slide_acc * BallPosWithVelFactorTmp * BallPosWithVelFactorTmp);
            }
            else if (BallPosWithVelFactorTmp < time2roll + timeslide) {
                float slide_dist = (ballVel.mod2() - transition_speed * transition_speed) / slide_acc * 0.5;
                float roll_time = BallPosWithVelFactorTmp - time2roll;
                float roll_dist = transition_speed * roll_time - 0.5 * roll_acc * roll_time * roll_time;
                ballPosWithVel = ball.Pos() + ballVel.unit() * (roll_dist + slide_dist);
            }
            else {
                float slide_dist = (ballVel.mod2() - transition_speed * transition_speed) / slide_acc * 0.5;
                float roll_dist = transition_speed * transition_speed / roll_acc * 2;
                ballPosWithVel = ball.Pos() + ballVel.unit() * (roll_dist + slide_dist);
            }
            GDebugEngine::Instance()->gui_debug_msg(ballPosWithVel, "slide ball");
        }
//		ballPosWithVel = ball.Pos() + ballVel * BallPosWithVelFactorTmp;
//		//小球速预测
//		if (ball.Vel().mod() < 20.0 && ball.Vel().mod() * sin(fabs(Utils::Normalize(ball.Vel().dir() - finalDir))) > 10)
//		{
//			ballPosWithVel = ball.Pos() + Utils::Polar2Vector(Param::Field::BALL_SIZE * 1.2, ball.Vel().dir());
//		}
    }
	if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_arc(ballPosWithVel, 20, 0, 360, COLOR_YELLOW);

	/********************************************************************/
	/* 拿球Task初始化  by lsp */
	/********************************************************************/
	TaskT getball_task(task());
	getball_task.player.rotvel = 0.0;
	getball_task.player.needdribble = false;

	// 标签设置说明如下
	/*
	拿球标签: PlayerStatus::GOTO_GRASP_BALL	 --> 不管调用该任务的上层是否有设置，都强行予以设置，表征拿球
	避球标签: PlayerStatus::DODGE_BALL		 --> 不管调用该任务的上层是否有设置，都强行不予设置，表征不避球
	控球标签: PlayerStatus::DRIBBLING		 --> 结合上层决定是否开启控球
	*/
	getball_task.player.flag = getball_task.player.flag | PlayerStatus::DODGE_OUR_DEFENSE_BOX;//躲避我方禁区!!
	getball_task.player.flag = getball_task.player.flag;	//设置拿球标签
	getball_task.player.flag = getball_task.player.flag & (~PlayerStatus::DODGE_BALL);		//取消避球标签
	if (!(getball_task.player.flag & PlayerStatus::DRIBBLING))
		getball_task.player.flag = getball_task.player.flag & (~PlayerStatus::DRIBBLING);	//取消控球标签

	/********************************************************************/
	/* 状态判断模块  by lsp */
	/********************************************************************/
	const CGeoLine myheadLine = CGeoLine(myhead, Utils::Normalize(me.Dir() + Param::Math::PI / 2.0));
	const CGeoPoint ball2myheadLine_ProjPoint = myheadLine.projection(ballPosWithVel);
	double dAngDiff_self2ball_finaldir = fabs(Utils::Normalize(self2ball.dir() - finalDir));
	bool isBallFrontOfMyhead = myhead.dist(ball2myheadLine_ProjPoint) < ball2myheadLimit ? true : false;  //球到嘴巴线的投影点在可以拿球的范围内
	bool isInDirectGetBallCircle = me.Pos().dist(ballPosWithVel) < directGetBallDist ? true : false;     //是否在直接冲上去拿球距离之内
	bool isGetBallDirReached = fabs(dAngDiff_self2ball_finaldir) < directGetBballDirLimit;
	bool isCanDirectGetBall = isBallFrontOfMyhead && isInDirectGetBallCircle && isGetBallDirReached;      //重要布尔量:是否能直接上前拿球
	bool dirOut = fabs(dAngDiff_self2ball_finaldir) > Param::Math::PI / 18.0;
	bool canNOTDirectGetBall = !isBallFrontOfMyhead || !isInDirectGetBallCircle || dirOut;
	bool isInNearBallCircle = me.Pos().dist(ball.Pos()) < nearBallRadius ? true : false;                 //是否在AVOIDBALL小圈之内
	bool isBallBesideMe = false;
	bool isBallBehindMe = false;
	if (isInNearBallCircle)
	{
		if (canNOTDirectGetBall)
		{
			if (fabs(dAngDiff_self2ball_finaldir) > Param::Math::PI / 2.0)
			{
				isBallBehindMe = true;
			}
			else isBallBesideMe = true;
		}
	}
	bool needAvoidBall = isBallBehindMe || isBallBesideMe;                                                //重要布尔量：是否需要躲避球
	//通过红外进一步检测是否需要avoidBall
	if (RobotSensor::Instance()->IsInfraredOn(robotNum))
	{
		needAvoidBall = false;
	}
	double isAvoidBallSuccess = nearBallRadius * (1.2 + dAngDiff_self2ball_finaldir / Param::Math::PI);
	bool avoidBallSuccess = me.Pos().dist(ball.Pos()) > isAvoidBallSuccess ? true : false;          //重要布尔量：是否躲避球成功，避球点在执行模块中计算

	if (AVOIDBALL != getState())
	{
		if (needAvoidBall)
		{
			avoidBallCount++;
		}
		else avoidBallCount = 0;
		if (avoidBallCount > 15 * 1.25)//15
		{
			trueNeedAvoidBall = true;
		}
	}

	/********************************************************************/
	/* 状态跳转管理模块，增加当前状态输出，便于查看状态跳转TODO  by lsp */
	/********************************************************************/
	if (BEGINNING == getState())             //当前状态为BEGINNING
	{
		if (isCanDirectGetBall)
		{
			setState(DIRECTGOTO);
		}
		else if (trueNeedAvoidBall)
		{
			setState(AVOIDBALL);
			trueNeedAvoidBall = false;
			avoidBallCount = 0;
			if (isBallBehindMe)
			{
				ab_state = BALLBEHINDME;
			}
			else if (isBallBesideMe)
			{
				ab_state = BALLBESIDEME;
			}
		}
		else {
			setState(GETBALL);
			gb_state = LARGEANGLE;
		}
	}
	else if (DIRECTGOTO == getState())     //当前状态为DIRECTGOTO
	{
		if (RobotSensor::Instance()->IsInfraredOn(robotNum))
		{ //什么都不做，不跳转状态
		}
		else if (trueNeedAvoidBall)
		{
			setState(AVOIDBALL);
			if (DEBUG_ENGINE)
			{
				cout << "-->AvoidBall";
			}
			trueNeedAvoidBall = false;
			avoidBallCount = 0;
			if (isBallBehindMe)
			{
				ab_state = BALLBEHINDME;
			}
			else if (isBallBesideMe)
			{
				ab_state = BALLBESIDEME;
			}
		}
		else if (canNOTDirectGetBall)
		{
			setState(GETBALL);
			gb_state = LARGEANGLE;
			if (DEBUG_ENGINE)
			{
				cout << "-->GetBall";
			}
		}
	}
	else if (GETBALL == getState())        //当前状态为GETBALL
	{
		if (isCanDirectGetBall)
		{
			setState(DIRECTGOTO);
			if (DEBUG_ENGINE)
			{
				cout << "-->DirectGoto";
			}
		}
		else if (RobotSensor::Instance()->IsInfraredOn(robotNum))
		{ //什么都不做，不跳转状态
		}
		else if (trueNeedAvoidBall)
		{
			setState(AVOIDBALL);
			if (DEBUG_ENGINE)
			{
				cout << "-->AvoidBall";
			}
			trueNeedAvoidBall = false;
			avoidBallCount = 0;
			if (isBallBehindMe)
			{
				ab_state = BALLBEHINDME;
			}
			else if (isBallBesideMe)
			{
				ab_state = BALLBESIDEME;
			}
		}
	}
	else if (AVOIDBALL == getState())      //当前状态为AVOIDBALL
	{
		if (avoidBallSuccess || isCanDirectGetBall)
		{
			setState(GETBALL);
			gb_state = LARGEANGLE;
			if (DEBUG_ENGINE)
			{
				cout << "-->GetBall";
			}
			ab_state = NOAVOID;
		}
	}
	
	/********************************************************************/
	/* 状态执行管理模块，根据当前状态做出具体动作  by lsp */
	/********************************************************************/
	int nowState = getState();
	switch (nowState)
	{
	case DIRECTGOTO: {
		// 拿球点
		getball_task.player.pos = ballPosWithVel + Utils::Polar2Vector(Param::Vehicle::V2::PLAYER_FRONT_TO_CENTER + newVehicleBuffer + Param::Field::BALL_SIZE + StopDist + GETBALL_BIAS, reverse_finalDir); //  5.85
		// 是否吸球
		getball_task.player.needdribble = IS_DRIBBLE;
		break; }
	case GETBALL: {
		getball_task.player.pos = ball.Pos();
		if (LARGEANGLE == gb_state)
		{
			if (fabs(dAngDiff_self2ball_finaldir) <= Param::Math::PI - extremeAngle + Param::Math::PI * 5 / 180)
			{
				gb_state = SMALLANGLE;
			}
		}
		else if (SMALLANGLE == gb_state)
		{
			if (fabs(dAngDiff_self2ball_finaldir) > Param::Math::PI / 9.0)
			{
				gb_state = LARGEANGLE;
			}
		}
		if (SMALLANGLE == gb_state) {
			double getBallBuffer = -3 + 1 * me.Pos().dist(ballPosWithVel) / 50;  //拿球时设计的余量-2 + 7 * me.Pos().dist(ballPosWithVel)/50;
			if (getBallBuffer > 2)
			{
				getBallBuffer = 2;
			}
			double getBallDist = Param::Vehicle::V2::PLAYER_FRONT_TO_CENTER + newVehicleBuffer + Param::Field::BALL_SIZE + getBallBuffer;
			if (getBallDist > me.Pos().dist(ballPosWithVel))
			{
				getBallDist = Param::Vehicle::V2::PLAYER_FRONT_TO_CENTER + newVehicleBuffer + Param::Field::BALL_SIZE + StopDist - 2.5;
			}
			getball_task.player.pos = ballPosWithVel + Utils::Polar2Vector(getBallDist, reverse_finalDir); //该距离要小于directGetBallDist
			// 吸球
			getball_task.player.needdribble = IS_DRIBBLE;
		}
		else if (LARGEANGLE == gb_state)
		{
			//角度计算
			double theta_Dir = ball2self.dir();
			int sign = Utils::Normalize((theta_Dir - finalDir)) > 0 ? 1 : -1;
			theta_Dir = Utils::Normalize(theta_Dir + sign * Param::Math::PI * 65 / 180.0); //Param::Math::PI * 75 / 180.0
			int sign2 = Utils::Normalize((theta_Dir - finalDir)) > 0 ? 1 : -1;
			if (ball.Vel().mod() * sin(fabs(Utils::Normalize(ball.Vel().dir() - finalDir))) < transverseBallSpeed)//球速相对目标方向较小的时候
			{
				if (sign * sign2 < 0 || fabs(Utils::Normalize(theta_Dir - finalDir)) > extremeAngle)
				{
					if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(me.Pos() + CVector(0, 50), "extreme angle", COLOR_WHITE);
					theta_Dir = Utils::Normalize(finalDir + sign * extremeAngle);
				}
			}
			//距离计算
			//if (DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_line(me.Pos(), me.Pos() + Utils::Polar2Vector(1000, theta_Dir), COLOR_WHITE);
			double theta = fabs(Utils::Normalize(theta_Dir - finalDir));
			double getBallDist = minGetBallDist + (maxGetBallDist - minGetBallDist) * (1 + cos(theta));
			if (getBallDist > maxGetBallDist)
			{
				getBallDist = maxGetBallDist;
			}
			getball_task.player.pos = ballPosWithVel + Utils::Polar2Vector(getBallDist, theta_Dir);
		}
		break; }
	case AVOIDBALL: {//TODO 加入球速影响，球在有速度的情况下修正躲避点
		if (BALLBEHINDME == ab_state)
		{
			double theta_Dir = ball2self.dir();
			double theta = Utils::Normalize(theta_Dir - finalDir);
			int sign = theta > 0 ? 1 : -1;
			theta_Dir = Utils::Normalize(theta_Dir + sign * Param::Math::PI * 60 / 180);
			getball_task.player.pos = ballPosWithVel + Utils::Polar2Vector(15, theta_Dir);
		}
		else if (BALLBESIDEME == ab_state)
		{
			double theta_Dir = reverse_finalDir;
			getball_task.player.pos = ballPosWithVel + Utils::Polar2Vector(15, theta_Dir);
		}
		break; }
	default: break;
	}

	// 计算给定的朝向，比较远靠近时朝向先取车速的方向，之后再转
	double diffAngleVel2Final = fabs(dAngDiff_self2ball_finaldir);
	int sign = diffAngleVel2Final > Param::Math::PI / 2.0 ? 1 : 0;
	if (ball.Pos().dist(me.Pos()) > AllowFaceToFinalDist) 
		getball_task.player.angle = Utils::Normalize(self2ball.dir() + sign * Param::Math::PI);
	else getball_task.player.angle = finalDir;

	// 面版图形绘制
	if (DEBUG_ENGINE) { //叉叉
		if (DIRECTGOTO == getState())
		{
			GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(170, -160), "DirectGoto", COLOR_CYAN);
		}
		else if (GETBALL == getState())
		{
			GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(170, -160), "GetBall", COLOR_CYAN);
			if (SMALLANGLE == gb_state) {
				GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(120, -170), "small angle", COLOR_CYAN);
			}
			else if (LARGEANGLE == gb_state) {
				GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(120, -170), "large angle", COLOR_CYAN);
			}
		}
		else if (AVOIDBALL == getState())
		{
			GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(170, -160), "AvoidBall", COLOR_CYAN);
		}

		//GDebugEngine::Instance()->gui_debug_x(ball.Pos(),COLOR_WHITE);
		GDebugEngine::Instance()->gui_debug_x(getball_task.player.pos, COLOR_BLACK);
		GDebugEngine::Instance()->gui_debug_line(getball_task.player.pos, getball_task.player.pos + getball_task.player.vel, COLOR_ORANGE);
	}

	// 调用底层控制
	CTRL_METHOD mode = task().player.specified_ctrl_method;
	getball_task.player.is_specify_ctrl_method = true;
	getball_task.player.specified_ctrl_method = mode;
	setSubTask(TaskFactoryV2::Instance()->SmartGotoPosition(getball_task));

	_lastCycle = pVision->Cycle();
	CStatedTask::plan(pVision);
}

CPlayerCommand* CGetBallV3::execute(const CVisionModule* pVision)
{
	if (subTask()) {
		return subTask()->execute(pVision);
	}

	return NULL;
}
