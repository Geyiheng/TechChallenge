#include "ballChaserAttributeSet.h"

//attribute define
DEFINE_ATTRIBUTE_NEW(ballChaserTest);//for test 
DEFINE_ATTRIBUTE_NEW(Dist2BallNormalized);//��һ����ͬһ֡���г����ֵΪ1���ĵ���ľ���
DEFINE_ATTRIBUTE_NEW(Dist2BallProjModified);//��һ��������80��Ϊ1����СΪ0.125���ĵ�������ͶӰ�ľ��룬�ٸ������ٵ�λ����ϵ��
DEFINE_ATTRIBUTE_NEW(BallMovingCost);//׷��ģ�ͣ�Դ������ֲ�������Դ���

//attribute evaluate
EVALUATE_ATTRIBUTE_NEW(ballChaserTest)
{
		setValue(vision->TheirPlayer(num).Valid());
}

EVALUATE_ATTRIBUTE_NEW(Dist2BallNormalized)
{
	vector<double> Dist2BallList;
	for (int theirNum = 0; theirNum < Param::Field::MAX_PLAYER; ++theirNum)
	{
		const PlayerVisionT enemy = pVision->TheirPlayer(theirNum);
		if (enemy.Valid())
			Dist2BallList.push_back((enemy.Pos() - pVision->Ball().Pos()).mod());
		else
			Dist2BallList.push_back(0);
	}
	if (Dist2BallList.empty())
		setValue(1);
	else
		setValue(Dist2BallList[num] / (*max(Dist2BallList.begin(), Dist2BallList.end()) + 1e-8));
}

EVALUATE_ATTRIBUTE_NEW(Dist2BallProjModified)
{
	const BallVisionT& ball = pVision->Ball();
	const PlayerVisionT& player = pVision->TheirPlayer(num);

	double limited_ball_speed = 800;
	if (!ball.Valid()) {
		limited_ball_speed = 50;
	}
	CVector applicable_ball_vel = ball.Vel();
	if (ball.Vel().mod() > limited_ball_speed) {
		applicable_ball_vel = Utils::Polar2Vector(limited_ball_speed, ball.Vel().dir());
	}

	double maxProjDist = 80;
	CGeoLine ballVelLine = CGeoLine(ball.Pos(), ball.Pos() + applicable_ball_vel);
	CGeoPoint proj = ballVelLine.projection(player.Pos());
	double dist = player.Pos().dist(proj);
	if (ball.Vel().mod() < 30 || !ball.Valid()) {
		dist = player.Pos().dist(ball.Pos());
	}
	dist = min(dist, maxProjDist);
	dist = max(dist, 10.0);

	double yDistFactor = 0;
	if (ball.Vel().mod() > 600) {
		yDistFactor = 1.3;
	} else if (ball.Vel().mod() > 400) {
		yDistFactor = 1.1;
	} else if (ball.Vel().mod() > 350) {
		yDistFactor = 1;
	} else if (ball.Vel().mod() > 280) {
		yDistFactor = 0.8;
	} else if (ball.Vel().mod() > 220) {
		yDistFactor = 0.65;
	} else if (ball.Vel().mod() > 160) {
		yDistFactor = 0.45;
	} else if (ball.Vel().mod() > 100) {
		yDistFactor = 0.3;
	} else if (ball.Vel().mod() > 70) {
		yDistFactor = 0.1;
	}

	setValue(yDistFactor * dist / maxProjDist);
}

EVALUATE_ATTRIBUTE_NEW(BallMovingCost)
{
	const BallVisionT& ball = vision->Ball();
	const PlayerVisionT& player = vision->TheirPlayer(num);

	double limited_ball_speed = 800;
	if (!ball.Valid())
		limited_ball_speed = 50;
	CVector applicable_ball_vel = ball.Vel();
	if (ball.Vel().mod() > limited_ball_speed)
		applicable_ball_vel = Utils::Polar2Vector(limited_ball_speed, ball.Vel().dir());
	CVector playerToBall = ball.Pos() - player.Pos();
	double angleDiff = applicable_ball_vel.dir() - playerToBall.dir();
	double ballMovingCost = applicable_ball_vel.mod() * cos(angleDiff);
	//ballMovingCost += diffVel.mod() * fabs(sin(Param::Math::PI-fabs(angleDiff)));

	double maxDist = 1;//todo ��һ��Ӧ���ø�����ķ�ʽ
	vector<double> Dist2BallList;
	for (int theirNum = 0; theirNum < Param::Field::MAX_PLAYER; ++theirNum)
	{
		const PlayerVisionT enemy = pVision->TheirPlayer(theirNum);
		if (enemy.Valid())
			Dist2BallList.push_back((enemy.Pos() - pVision->Ball().Pos()).mod());
		else
			Dist2BallList.push_back(0);
	}
	if (Dist2BallList.empty())
		maxDist = ballMovingCost;
	else
		maxDist = *max(Dist2BallList.begin(), Dist2BallList.end());
	setValue((ballMovingCost) / maxDist);
}

void CballChaserAttributeSet::setAttributes()
{
	ADD_ATTRIBUTE_NEW(ballChaserTest);
	ADD_ATTRIBUTE_NEW(Dist2BallNormalized);
	ADD_ATTRIBUTE_NEW(Dist2BallProjModified);
	ADD_ATTRIBUTE_NEW(BallMovingCost);
}
