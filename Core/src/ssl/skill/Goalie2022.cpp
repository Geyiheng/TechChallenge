#include "Goalie2022.h"
#include <random>

#include "GDebugEngine.h"
#include "Vision/VisionModule.h"
#include "skill/Factory.h"
#include "utils.h"
#include "WorldModel/DribbleStatus.h"
#include "RobotSensor.h"
#include "param.h"
#include "WorldModel/WorldModel.h"
#include "TaskMediator.h"
#include "defenceNew/DefenceInfoNew.h"
#include "gpuBestAlgThread.h"

#define DEBUG_EVALUATE(x) {if(goalie_debug) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(-Param::Field::PITCH_LENGTH*0.7,0), x);}

namespace {
	bool goalie_debug, is_penalty;
	int this_shoot_cycle = 0;
	const CGeoPoint goalCenter(-Param::Field::PITCH_LENGTH / 2, 0);
	CGeoPoint trickPoint;
	CGeoPoint rescuePoint(goalCenter);
	CGeoLine BaseLine(goalCenter, Param::Math::PI / 2);
	CGeoLine moveLine(goalCenter + CVector(Param::Field::PENALTY_AREA_DEPTH / 4.0, 0), Param::Math::PI / 2);
	const double PENALTY_BUFFER = 0.05 * Param::Field::PENALTY_AREA_DEPTH;
	double SLOW_BALL_SPD;
	int KICKPOWER;
	double HAVE_BALL_DIST;
	double CLOSE_DIST;
	bool AGGRESSIVE_GOALIE;
	double CHALLENGE_BALL_DIST;
	double BLOCK_DIST;
}

CGoalie2022::CGoalie2022() :last_penalty_status(PENALTY_WAIT)
{
	goalie_debug = paramManager->GOALIE_DEBUG;
	trickPoint = CGeoPoint(-Param::Field::PITCH_LENGTH / 2 + Param::Field::PENALTY_AREA_DEPTH * paramManager->TRICKPOS_DIV_PENALTY, 0);
	SLOW_BALL_SPD = paramManager->SLOW_BALL_SPD;
	KICKPOWER = paramManager->KICKPOWER_GOALIE;
	HAVE_BALL_DIST = paramManager->HAVE_BALL_DIST;
	CLOSE_DIST = paramManager->CLOSE_DIST;
	AGGRESSIVE_GOALIE = paramManager->AGGRESSIVE_GOALIE;
	CHALLENGE_BALL_DIST = paramManager->CHALLENGE_BALL_DIST;
	BLOCK_DIST = paramManager->BLOCK_DIST;
}

void CGoalie2022::plan(const CVisionModule* pVision)
{
	is_penalty = task().player.isPenalty;
	Tevaluate purpose = evaluate(pVision);
	CPlayerTask* pTask;
	switch (purpose) {
	case TEST:
		break;
	case NORMAL:
		if (is_penalty)
			pTask = penaltyTask(pVision);
		else
			pTask = normalTask(pVision);
		break;
	case RESCUE:
		if (is_penalty)
			last_penalty_status = PENALTY_TRICK_FINISH;
		pTask = rescueTask(pVision);
		break;
	case SUPPORT:
		pTask = supportTask(pVision);
		break;
	case CLEAR_BALL:
		if (is_penalty)
			last_penalty_status = PENALTY_TRICK_FINISH;
		pTask = clearBallTask(pVision);
		break;
	case ATTACK_ENEMY:
		pTask = attackEnemyTask(pVision);
		break;
	}
	setSubTask(pTask);
	CStatedTask::plan(pVision);
}

CPlayerCommand* CGoalie2022::execute(const CVisionModule* pVision)
{
	if (subTask())
	{
		return subTask()->execute(pVision);
	}
	return NULL;
}

CGoalie2022::Tevaluate CGoalie2022::evaluate(const CVisionModule* pVision)
{
	int robotNum = task().executor;
	const PlayerVisionT& me = pVision->OurPlayer(robotNum);
	const PlayerVisionT& enemy = pVision->TheirPlayer(DefenceInfoNew::Instance()->getBestBallChaser());
	const BallVisionT& ball = pVision->Ball();

	if (WorldModel::Instance()->CurrentRefereeMsg() == "gameStop") {
		DEBUG_EVALUATE("evaluate: game stop");
		return NORMAL;
	} else if (!ball.Valid()) {
		DEBUG_EVALUATE("evaluate: invalid ball");
		return NORMAL;
	} else if (IsFarFromBack(ball.Pos())) {
		DEBUG_EVALUATE("evaluate: far ball");
		return NORMAL;
	} else if (!is_penalty && ShouldAttack(pVision)) {
		DEBUG_EVALUATE("evaluate: danger, attack enemy");
		return ATTACK_ENEMY;
	} else if (ball.Vel().mod() > SLOW_BALL_SPD) {
		if (isBallShot2Goal(pVision)) {
			DEBUG_EVALUATE("evaluate: fast ball, to goal, emergency");
			return RESCUE;
		} else {
			DEBUG_EVALUATE("evaluate: fast ball, not to goal");
			return NORMAL;
		}
	} else if (Utils::InOurPenaltyArea(ball.Pos(), PENALTY_BUFFER)) {
		if (is_penalty || enemy.Pos().dist(ball.Pos()) < HAVE_BALL_DIST) {
			DEBUG_EVALUATE("evaluate: slow ball, enemy inside, clear ball");
			return CLEAR_BALL;
		} else {
			DEBUG_EVALUATE("evaluate: slow ball, safe to support");
			return SUPPORT;
		}
	} else {
		DEBUG_EVALUATE("evaluate: normal");
		return NORMAL;
	}
}

bool CGoalie2022::isBallShot2Goal(const CVisionModule* pVision)
{
	const BallVisionT& Ball = pVision->Ball();
	if (Ball.VelX() >= 0)
		return false;
	CGeoLine BallVelLine(Ball.Pos(), Ball.Vel().dir());
	CGeoLineLineIntersection interseciton(BaseLine, BallVelLine);
	if (interseciton.Intersectant() == true) {
		CGeoPoint point = interseciton.IntersectPoint();
		if (fabs(point.y()) < Param::Field::PENALTY_AREA_WIDTH / 2 + Param::Vehicle::V2::PLAYER_SIZE)
			return true;
	}
	return false;
}

bool CGoalie2022::ShouldAttack(const CVisionModule* pVision)
{
	/************************************************************************/
	/* �������ڿ���ǰ��λ�ô��㵥�������ҷ�����֧Ԯ���ѣ�����ʱ���Ҫ����������
	   ͬʱ�������������սϴ������return false��������߼�Ϊ��  by SYLG */
	   /************************************************************************/
	if (!AGGRESSIVE_GOALIE)
		return false;
	int robotNum = task().executor;
	const PlayerVisionT& enemy = pVision->TheirPlayer(DefenceInfoNew::Instance()->getBestBallChaser());
	const BallVisionT& ball = pVision->Ball();

	if (abs(enemy.Pos().y()) > Param::Field::PENALTY_AREA_WIDTH / 2.0 + 30
		|| enemy.Pos().dist(ball.Pos()) > CLOSE_DIST)
		return false;
	for (int i = 0; i < Param::Field::MAX_PLAYER; i++)
	{
		if (robotNum != i)
		{
			const PlayerVisionT& helper_i = pVision->OurPlayer(i);
			if (!helper_i.Valid())
				continue;
			if (helper_i.Pos().dist(enemy.Pos()) < CLOSE_DIST
				|| helper_i.Pos().dist(ball.Pos()) < CLOSE_DIST)//�Ѿ��а�����
				return false;
		}
	}
	//���з��ؿ���£��к���������֧Ԯ
	if (TaskMediator::Instance()->leftBack() || TaskMediator::Instance()->rightBack() || TaskMediator::Instance()->singleBack())
		return false;
	for (int i = 0; i < Param::Field::MAX_PLAYER; i++) {
		if (TaskMediator::Instance()->isMultiBack(i))
			return false;
	}
	//�ۺ��жϵ������ơ�Ŀǰ��һ�λ��Ƚϴֲ� by SYLG
	int free_enemy = 0;
	for (int i = 0; i < Param::Field::MAX_PLAYER; i++)
	{
		const PlayerVisionT& enemy_i = pVision->TheirPlayer(i);
		if (IsFarFromBack(enemy_i.Pos()) == false)
		{
			free_enemy++;
			for (int j = 0; j < Param::Field::MAX_PLAYER; j++)
			{
				if (pVision->OurPlayer(j).Pos().dist(enemy_i.Pos()) < CLOSE_DIST)
				{
					free_enemy--;
					break;
				}
			}
		}
	}
	return free_enemy <= 1;
}

inline bool CGoalie2022::IsFarFromBack(const CGeoPoint& pos, int x)
{
	return pos.x() > x;
}

CPlayerTask* CGoalie2022::normalTask(const CVisionModule* pVision)
{
	int robotNum = task().executor;

	//CGeoPoint DefPoint = DefPos2015::Instance()->getDefPos2015(pVision).getGoaliePos();
	CGeoPoint DefPoint;
	const BallVisionT& ball = pVision->Ball();
	const PlayerVisionT& enemy = pVision->TheirPlayer(DefenceInfoNew::Instance()->getBestBallChaser());
	CGeoPoint defenceTarget = ball.Valid() ? ball.Pos() : enemy.Pos();
	CGeoLine defenceLine(defenceTarget, goalCenter);
	CGeoLineLineIntersection intersect(defenceLine, moveLine);
	if (!intersect.Intersectant())
		DefPoint = goalCenter;
	else
		DefPoint = intersect.IntersectPoint();
	double limit_y = Param::Field::GOAL_WIDTH / 2.0 - 5.0;
	if (fabs(DefPoint.y()) > limit_y)
		DefPoint.setY(DefPoint.y() / fabs(DefPoint.y()) * limit_y);

	double dir;
	const PlayerVisionT& me = pVision->OurPlayer(robotNum);
	if (ball.Valid()) {
		dir = CVector(ball.Pos() - me.Pos()).dir();
	} else {
		dir = CVector(me.Pos() - goalCenter).dir();
	}

	int flag = task().player.flag;
	flag |= PlayerStatus::QUICKLY;

	return PlayerRole::makeItGoto(robotNum, DefPoint, dir, flag);
}

CPlayerTask* CGoalie2022::rescueTask(const CVisionModule* pVision)
{
	/************************************************************************/
	/* ����ʱ����������������٣�����Ҫ�����������ҵ�P��ʹ��P���˽�������Զ����ֵ����С��
	   ���׷��֣�P-��-��Ϊֱ�ǡ�ͬʱ�������������ǡ���˵���ĵ㣬����������е���������ȶ���
	   ���ǣ���ʵ�У�����Ա��Ҫ���٣����ٶ����һ��ʼ���Ļ��ᵼ�����ɵ��λ�øı䣨���ƣ����̶���������Ա���򡢼��ٲ���ʱ
	   ��ˣ�ά���з����ŵļ�״̬�����������ſ�ʼ��ʱ���������� by SYLG */
	   /************************************************************************/
	//�ܵ�ס���ǰ����һ�����Ž�����һ�ε�λ
	const BallVisionT& ball = pVision->Ball();
	CGeoLine ballVelLine(ball.Pos(), ball.Vel().dir());
	CGeoPoint projection = ballVelLine.projection(rescuePoint);
	if (projection.dist(rescuePoint) > Param::Vehicle::V2::PLAYER_SIZE || pVision->Cycle() - this_shoot_cycle > 100)
		generateRescuePoint(pVision);
	this_shoot_cycle = pVision->Cycle();
	if (goalie_debug)
		GDebugEngine::Instance()->gui_debug_msg(rescuePoint, "r", COLOR_BLUE);

	int robotNum = task().executor;
	const PlayerVisionT& me = pVision->OurPlayer(robotNum);

	int flag = task().player.flag;
	flag |= PlayerStatus::QUICKLY;
	flag |= PlayerStatus::DRIBBLING;

	return PlayerRole::makeItGoto(robotNum, rescuePoint, me.Dir(), flag);
}


CGoalie2022::supportStruct::supportStruct(const CVisionModule* pVision, int i, int robotNum)
{
	const PlayerVisionT& me = pVision->OurPlayer(robotNum);
	const BallVisionT& ball = pVision->Ball();
	supporterNum = TaskMediator::Instance()->supporter(i);
	supportPoint = GPUBestAlgThread::Instance()->getBestPointFromArea(i);
	CVector ball2supporter(supportPoint - ball.Pos());
	angleDiff = fabs(me.Dir() - ball2supporter.dir());
}
CPlayerTask* CGoalie2022::supportTask(const CVisionModule* pVision)
{
	//����ʵ��Ч�������� �ݻ� by SYLG 5.15
	//��ʱ���������ʼ
	{
		int robotNum = task().executor;
		const PlayerVisionT& me = pVision->OurPlayer(robotNum);
		double dir = CVector(CGeoPoint(400, 0) - me.Pos()).dir();
		int flag = task().player.flag;
		flag |= PlayerStatus::QUICKLY;
		flag |= PlayerStatus::DRIBBLING;
		return PlayerRole::makeItNoneTrajGetBall(robotNum, dir, CVector(0, 0), flag);
	}
	//��ʱ�����������

	int robotNum = task().executor;
	const BallVisionT& ball = pVision->Ball();

	vector<supportStruct> supportStructs;
	supportStructs.reserve(AREANUM);
	for (int i = 0; i < AREANUM; i++)
		supportStructs[i] = supportStruct(pVision, i, robotNum);
	//֧Ԯ��ɸѡ��֧Ԯ�ߵ�λ��ת���ǶȲ�󣨲�Ȼ̫������һ��Ҫ��ȫ
	supportStructs.erase(remove_if(supportStructs.begin(), supportStructs.end(),
		[&](const auto& item) {
		int supporterNum = item.supporterNum;
		const PlayerVisionT& supporter = pVision->OurPlayer(supporterNum);
		CGeoPoint supportPoint = item.supportPoint;
		double angleDiff = item.angleDiff;
		if (supporterNum == 0 || supporter.Pos().dist(supportPoint) > 100 || angleDiff > Param::Math::PI / 2)
			return true;

		CGeoLine passLine(ball.Pos(), supporter.Pos());
		for (int i = 0; i < Param::Field::MAX_PLAYER; i++)
		{
			const PlayerVisionT& blocker = pVision->TheirPlayer(i);
			if (!blocker.Valid())
				continue;
			CGeoPoint blockerProj = passLine.projection(blocker.Pos());
			double blocker2projDist = blocker.Pos().dist(blockerProj);
			double ball2projDist = ball.Pos().dist(blockerProj);
			if (blocker2projDist > 300 || ball2projDist < 10)
				continue;
			double ratio_dist = blocker2projDist / ball2projDist;
			if (ratio_dist < 1.2)
				return true;
		}
		return false;
	}), supportStructs.end());
	sort(supportStructs.begin(), supportStructs.end(), [&](const auto& a, const auto& b) {return a.angleDiff < b.angleDiff;	});

	if (supportStructs.empty())
		return clearBallTask(pVision);

	double finalDir = supportStructs[0].angleDiff;

	int flag = task().player.flag;
	flag |= PlayerStatus::QUICKLY;
	flag |= PlayerStatus::DRIBBLING;

	double dist = (ball.Pos() - supportStructs[0].supportPoint).mod() - Param::Vehicle::V2::PLAYER_FRONT_TO_CENTER;
	double power = sqrt(powf(ParamManager::Instance()->FASTEST_RECEIVE_VEL, 2) + 2 * ParamManager::Instance()->BALL_DEC * dist);
	power = min(power, (double)Param::Rule::MAX_BALL_SPEED);

	return PlayerRole::makeItDribbleTurnKickV2(robotNum, finalDir, 7.0 * Param::Math::PI / 180.0, 0, power);
	//todo1 isAdvance��target��ɶ�� ��zyj
	//todo2 �ı�flag����Ȼ����Ա�ò���
	//todo3 ��ô����supporter��Ҫ������
}

CPlayerTask* CGoalie2022::clearBallTask(const CVisionModule* pVision)
{
	//��������ûд����

	const BallVisionT& ball = pVision->Ball();
	int robotNum = task().executor;

	double dir = CalClearBallDir(pVision);

	int flag = task().player.flag;
	flag |= PlayerStatus::QUICKLY;
	flag |= PlayerStatus::DRIBBLING;

	return PlayerRole::makeItGoto(robotNum, ball.Pos(), dir, flag);//todo need test
	//return PlayerRole::makeItNoneTrajGetBall(robotNum, dir, CVector(0, 0), flag);
	//�õ����ͨ��lua��������Լ��߳�ȥ
}

CPlayerTask* CGoalie2022::attackEnemyTask(const CVisionModule* pVision)
{
	int robotNum = task().executor;
	const PlayerVisionT& me = pVision->OurPlayer(robotNum);
	const PlayerVisionT& enemy = pVision->TheirPlayer(DefenceInfoNew::Instance()->getBestBallChaser());
	const BallVisionT& ball = pVision->Ball();

	//����˹���ʱ����·�����ɴ�����ᣬ��ʱֱ�����򼴿�
	CGeoPoint pos;
	if (me.Pos().dist(enemy.Pos()) > CHALLENGE_BALL_DIST) {
		pos = enemy.Pos();
	} else {
		pos = ball.Pos();
	}

	double dir;
	if (ball.Valid()) {
		dir = CVector(ball.Pos() - me.Pos()).dir();
	} else {
		dir = CVector(me.Pos() - goalCenter).dir();
	}

	int flag = task().player.flag;
	flag |= PlayerStatus::NOT_AVOID_OUR_VEHICLE;
	flag |= PlayerStatus::NOT_AVOID_THEIR_VEHICLE;
	flag |= PlayerStatus::QUICKLY;
	flag |= PlayerStatus::DRIBBLING;

	return PlayerRole::makeItGoto(robotNum, pos, dir, flag);
}

CPlayerTask* CGoalie2022::penaltyTask(const CVisionModule* pVision)
{
	CGeoPoint defCenter;//�������ģ�Ӱ�������ķֲ�
	const PlayerVisionT& enemy = pVision->TheirPlayer(DefenceInfoNew::Instance()->getBestBallChaser());
	CGeoLine defLine(enemy.Pos(), enemy.Dir());
	if (fabs(enemy.Dir()) <= Param::Math::PI / 2)
		defLine = CGeoLine(enemy.Pos(), goalCenter);
	CGeoLineLineIntersection intersect(defLine, BaseLine);
	if (intersect.Intersectant())
		defCenter = intersect.IntersectPoint();
	else
		defCenter = goalCenter;

	static int random_num = paramManager->RANDOM_NUM;
	static double random_start = paramManager->RANDOM_MIN;
	static double random_end = paramManager->RANDOM_MAX;
	double random_step = (random_end - random_start) / double(random_num - 1);
	double random_current = random_start - random_step;

	vector<CGeoPoint> random_points(random_num);
	generate(random_points.begin(), random_points.end(), //generate points between [begin,end]
		[&]() {
		random_current += random_step;
		return CGeoPoint(-Param::Field::PITCH_LENGTH / 2, random_current * Param::Field::GOAL_WIDTH); });
	if (goalie_debug)
		for (auto& p : random_points)
			GDebugEngine::Instance()->gui_debug_x(p, COLOR_GREEN);
	default_random_engine generator{ (unsigned int)pVision->Cycle() };
	vector<int> weight_vector;
	weight_vector.reserve(random_num);
	for (auto& p : random_points)
		weight_vector.push_back(100000 / sqrt(defCenter.dist(p)) / std::log10(30 + p.dist(goalCenter)));
	discrete_distribution<int> weight(weight_vector.begin(), weight_vector.end());
	//�̶����֡�������������
	static int generate_index = weight(generator);
	static int interval_cnt = 0;
	static const int generate_interval = paramManager->STABLE_FRAME_INTERVAL;
	interval_cnt++;
	if (interval_cnt > generate_interval)
	{
		interval_cnt = 0;
		generate_index = weight(generator);
	}
	int robotNum = task().executor;

	CGeoPoint finalPos = random_points[generate_index];

	//������ǰ����
	Tpenalty penalty_status = last_penalty_status;
	const PlayerVisionT& me = pVision->OurPlayer(robotNum);
	switch (last_penalty_status)
	{
	case PENALTY_WAIT:
		if (pVision->gameState().gameOn() && enemy.Pos().x() < 0)
			penalty_status = PENALTY_TRICK_START;
		break;
	case PENALTY_TRICK_START:
		if (me.Pos().dist(trickPoint) < 20)
			penalty_status = PENALTY_TRICK_FINISH;
		break;
	case PENALTY_TRICK_FINISH:
		if (pVision->gameState().gameOff())
			penalty_status = PENALTY_WAIT;
		break;
	}
	last_penalty_status = penalty_status;
	if (penalty_status == PENALTY_TRICK_START)
		finalPos = trickPoint;

	double dir;
	const BallVisionT& ball = pVision->Ball();
	//if (ball.Valid()) {
	//	dir = CVector(ball.Pos() - me.Pos()).dir();
	//} else {
	//	dir = CVector(me.Pos() - goalCenter).dir();
	//}
	dir = 0;

	int flag = task().player.flag;
	flag |= PlayerStatus::QUICKLY;

	return PlayerRole::makeItGoto(robotNum, finalPos, dir, flag);
}

void CGoalie2022::generateRescuePoint(const CVisionModule* pVision)
{
	const BallVisionT& ball = pVision->Ball();
	const PlayerVisionT& me = pVision->OurPlayer(task().executor);
	CGeoLine ballVelLine(ball.Pos(), ball.Vel().dir());
	CVector ball2me(ball.Pos() - me.Pos());
	CGeoLine me_vertical2_me2ballLine(me.Pos(), ball2me.dir() + Param::Math::PI / 2);
	CGeoLineLineIntersection intersect(ballVelLine, me_vertical2_me2ballLine);
	if (intersect.Intersectant() && Utils::InOurPenaltyArea(intersect.IntersectPoint(), 0))
		rescuePoint = intersect.IntersectPoint();
	else
	{
		intersect = CGeoLineLineIntersection(ballVelLine, moveLine);
		if (!intersect.Intersectant())
			rescuePoint = goalCenter;
		else
			rescuePoint = intersect.IntersectPoint();
		double limit_y = Param::Field::GOAL_WIDTH / 2.0 - 5.0;
		if (fabs(rescuePoint.y()) > limit_y)
			rescuePoint.setY(rescuePoint.y() / fabs(rescuePoint.y()) * limit_y);
	}
}

double CGoalie2022::CalClearBallDir(const CVisionModule* pVision)
{
	/************************************************************************/
	/* ����Ƕȵļ��㡣Ŀǰս�����ž�����˫������leftBack+rightBack���ԣ�
	   ��˽���д��ز��ֵ��߼����������Ĵ���̳���Goalie2013
	   ����Ŀǰ������������£������ϲ���Ҫ�˲��ִ��룬����Ϊ���������ڴ� by SYLG */
	   /************************************************************************/
	int robotNum = task().executor;
	const PlayerVisionT& me = pVision->OurPlayer(robotNum);
	const BallVisionT& ball = pVision->Ball();
	const PlayerVisionT& enemy = pVision->TheirPlayer(DefenceInfoNew::Instance()->getBestBallChaser());
	double clearBallDir = 0;
	if (TaskMediator::Instance()->singleBack() == 0 && TaskMediator::Instance()->leftBack() != 0) {
		CGeoPoint leftpos = DefPos2015::Instance()->getDefPos2015(pVision).getLeftPos();
		CGeoPoint rightpos = DefPos2015::Instance()->getDefPos2015(pVision).getRightPos();
		double ball2leftDir = (DefPos2015::Instance()->getDefPos2015(pVision).getLeftPos() - ball.Pos()).dir();// -0.3;
		double ball2rightDir = (DefPos2015::Instance()->getDefPos2015(pVision).getRightPos() - ball.Pos()).dir();// +0.3;
		double goal2ballDir = CVector(ball.Pos() - goalCenter).dir();
		GDebugEngine::Instance()->gui_debug_line(ball.Pos(), ball.Pos() + Utils::Polar2Vector(200, ball2leftDir), COLOR_BLACK);
		GDebugEngine::Instance()->gui_debug_line(ball.Pos(), ball.Pos() + Utils::Polar2Vector(200, ball2rightDir), COLOR_BLACK);
		if (goal2ballDir > ball2leftDir && goal2ballDir < ball2rightDir)
		{
			if (ball2rightDir - ball2leftDir > Param::Math::PI / 2) {
				clearBallDir = Utils::Normalize((ball2leftDir + ball2rightDir) / 2);
			} else {
				vector<double> blockDirList{ -Param::Math::PI,Param::Math::PI };
				for (int i = 0; i < Param::Field::MAX_PLAYER; i++)
				{
					CGeoPoint pos = pVision->OurPlayer(i).Pos();
					if (pos.dist(ball.Pos()) < BLOCK_DIST && i != robotNum)
						blockDirList.push_back((pos - ball.Pos()).dir());
					pos = pVision->TheirPlayer(i).Pos();
					if (pos.dist(ball.Pos()) < BLOCK_DIST)
						blockDirList.push_back((pos - ball.Pos()).dir());
				}
				sort(blockDirList.begin(), blockDirList.end());
				double maxAngle = 0;
				for (int i = 0; i < blockDirList.size() - 1; i++)
					if (blockDirList[i + 1] - blockDirList[i] > maxAngle) {
						maxAngle = blockDirList[i + 1] - blockDirList[i];
						clearBallDir = (blockDirList[i + 1] + blockDirList[i]) / 2.0;
					}
			}
		} else {
			clearBallDir = Utils::Normalize(goal2ballDir);
		}
	} else if (TaskMediator::Instance()->singleBack() != 0) {
		double goal2singledir = (DefPos2015::Instance()->getDefPos2015(pVision).getSinglePos() - ball.Pos()).dir();
		if (std::abs(goal2singledir - clearBallDir) < Param::Math::PI / 18) {
			if (std::abs(clearBallDir) > Param::Math::PI / 10) {
				clearBallDir = -clearBallDir;
				if (std::abs(clearBallDir) > Param::Math::PI * 70 / 180) {
					if (clearBallDir < 0) {
						clearBallDir = clearBallDir + Param::Math::PI / 9;
					} else {
						clearBallDir = clearBallDir - Param::Math::PI / 9;
					}
				}
			} else {
				clearBallDir = clearBallDir + Param::Math::PI * 7 / 18;
			}
		}
	} else {
		clearBallDir = CVector(ball.Pos() - goalCenter).dir();
	}

	//̫���˻���ֱ����ԭʼ����Ƚ�����
	if (enemy.Pos().dist(ball.Pos()) < CLOSE_DIST) {
		clearBallDir = CVector(ball.Pos() - goalCenter).dir();
	}

	//�Ƕ�����
	if (clearBallDir >= Param::Math::PI * 80 / 180.0) {
		clearBallDir = Param::Math::PI * 80 / 180.0;
	} else if (clearBallDir <= -Param::Math::PI * 80 / 180.0) {
		clearBallDir = -Param::Math::PI * 80 / 180.0;
	}

	GDebugEngine::Instance()->gui_debug_line(ball.Pos(), ball.Pos() + Utils::Polar2Vector(600, clearBallDir), COLOR_BLUE);
	return clearBallDir;
}
