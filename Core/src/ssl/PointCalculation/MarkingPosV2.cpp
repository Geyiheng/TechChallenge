/************************************************************************/
/* Copyright (c) CSC-RL, Zhejiang University							*/
/* Team��		SSL-ZJUNlict											*/
/* HomePage:	http://www.nlict.zju.edu.cn/ssl/WelcomePage.html		*/
/************************************************************************/
/* File:	  MarkingPosV2.cpp											*/
/* Func:	  ����	attacker death marked								*/
/* Author:	  ��Ⱥ 2012-08-18											*/
/* Refer:	  ###														*/
/* E-mail:	  wangqun1234@zju.edu.cn									*/
/************************************************************************/
#include "PointCalculation/MarkingPosV2.h"
#include "param.h"
#include "utils.h"
#include "WorldModel/WorldModel.h"
#include "GDebugEngine.h"
#include "CornerAreaPos.h"
#include "defenceNew/DefenceInfoNew.h"
#include <ssl/ParamManagerNew.h>

namespace NameSpaceMarkingPosV2 {
	// const
	CGeoPoint ourGoal;
	const double REF_AVOID_BALL = Param::Field::FREE_KICK_AVOID_BALL_DIST + Param::Vehicle::V2::PLAYER_SIZE * 3.5;
	const double PUSH_ENEMY_BUFFER = 0;//original : PUSH_ENEMY_BUFFER = 15;
	const double CHASE_JUDGE_VALUE = 300;//�ж�chase������ֵ��ֵ
	const double CHASE_MARK_BUFFER = 20.0;//���׷��chase��

	const double NORMAL_DIST = 50.0;
	const double ENEMY_SIN_MARK_BUFFER = 25;//���������������ƶ��������̶�

	//const for DENY_BALL
	const double DENY_TIME_LIMIT = 0.5;//0.9	!!��ǰ��ʱ���ж�
	const double DENY_ANGLE_LIMIT = Param::Math::PI * 120 / 180;// !!��ǰ�Ŀ��ж��Ƕ��ж�
	const double DENY_DIST_LIMIT_1 = 25.0;// !!
	const double DENY_DIST_LIMIT_2 = 10;// !!
	const double DENY_BUFFER = 3.0;
	const double EXIT_DENY_DIST = 25;

	const double NO_ADVANCE_BALL_VEL = 200;//!!!!!��ǰʱ��ʹ��advanceball�ĶԷ���С�����ٶ�

	//const for BACK_LINE
	//const double BACK_BUFFER_IN = 160;//!!!!!
	//const double BACK_BUFFER_OUT = 180;//!!!!!
	const double BACK_BUFFER_IN = 210;//!!!!!
	const double BACK_BUFFER_OUT = 240;//!!!!!
	const double SPECIAL_AREA_PRE_TIME = 0.25;//����BACK_LINEģʽ�£��Է���Ԥ��ʱ��
	//const double SPECIAL_AREA_X_BUFFER = -100;//!!!!!��������BACK_LINEģʽ�£�x�������ֵ -120
	const double SPECIAL_AREA_X_BUFFER = -200;

	//��̬����
	bool DENY_LOG[Param::Field::MAX_PLAYER] = { false,false,false,false,false,false };
	bool BACK_LOG[Param::Field::MAX_PLAYER] = { false,false,false,false,false,false };
	int DEFENDER_NUM_LOG[Param::Field::MAX_PLAYER] = { 0,0,0,0,0,0 };
	//�ؼ����أ�
	bool DENY_BALL_MODE = false;//��ǰ����
	bool BACK_LINE_MODE = false;//���˷���
	bool SPECIAL_AREA_BACK_LINE_MODE = false;//��Խ�����������Ķ���ȫ������BACK_LINEģʽ

	double RECEIVER_MARK_BUFFER = -3.0;//receiver ���˾���

	//���Կ���
	bool debug = false;
};

using namespace NameSpaceMarkingPosV2;

CMarkingPosV2::CMarkingPosV2()
{
	DENY_BALL_MODE = ParamManager::Instance()->DENY_BALL_MODE;
	BACK_LINE_MODE = ParamManager::Instance()->BACK_LINE_MODE;
	SPECIAL_AREA_BACK_LINE_MODE = ParamManager::Instance()->SPECIAL_AREA_BACK_LINE_MODE;
	for (int i = 0; i < Param::Field::MAX_PLAYER; ++i)logCycle[i] = 0;
	for (int i = 0; i < Param::Field::MAX_PLAYER; ++i)markingPoint[i] = CGeoPoint(0, 0);
	oppPriority = 0;
	oppNum = 0;
	clearPos();
	ourGoal = CGeoPoint(-Param::Field::PITCH_LENGTH / 2, 0);
	_logCycle = 0;
}

CMarkingPosV2::~CMarkingPosV2()
{}

//���ݷ������ȼ�ѡ��,�ڳ���receiverʱ��������ȼ�˳��
CGeoPoint CMarkingPosV2::getMarkingPos(const CVisionModule* pVision, const int pri)
{
	//oppPriority = pri;
	//int bestEnemy = DefenceInfo::Instance()->getAttackOppNumByPri(0);
	////��receiverΪ������ȼ���ʱ����仰�������Ϊ���϶Է��Ƿ���receiver
	//if (DefenceInfo::Instance()->getOppPlayerByNum(bestEnemy)->isTheRole("RReceiver"))
	//{
	//	if (oppPriority > 0)
	//	{
	//		oppPriority -= 1;
	//	}		
	//}
	//oppNum = DefenceInfo::Instance()->getSteadyAttackOppNumByPri(oppPriority);
	//if (pVision->Cycle() > logCycle[oppNum])
	//{
	//	logCycle[oppNum] = pVision->Cycle();
	//	markingPoint[oppNum] = generatePos(pVision);
	//}
	//return markingPoint[oppNum];
	checkAllMarkingPos(pVision);
	oppPriority = pri;
	//int bestEnemy = DefenceInfo::Instance()->getAttackOppNumByPri(0);
	////��receiverΪ������ȼ���ʱ����仰�������Ϊ���϶Է��Ƿ���receiver
	//if (DefenceInfo::Instance()->getOppPlayerByNum(bestEnemy)->isTheRole("RReceiver"))
	//{
	//	if (oppPriority > 0)
	//	{
	//		oppPriority -= 1;
	//	}		
	//}
	oppNum = DefenceInfoNew::Instance()->getSteadyBallReceiverList()[oppPriority - 1];
	return markingPoint[oppNum];
}

CGeoPoint CMarkingPosV2::getMarkingPosByAbsolutePri(const CVisionModule* pVision, const int pri)
{
	oppPriority = pri;
	oppNum = DefenceInfo::Instance()->getSteadyAttackOppNumByPri(oppPriority);
	if (pVision->Cycle() > logCycle[oppNum])
	{
	logCycle[oppNum] = pVision->Cycle();
	markingPoint[oppNum] = generatePos(pVision);
	}
	return markingPoint[oppNum];
	checkAllMarkingPos(pVision);
	oppNum = DefenceInfoNew::Instance()->getSteadyBallReceiverList()[pri];
	return markingPoint[oppNum];
}

//���ݶ��ֳ���ѡ��
CGeoPoint CMarkingPosV2::getMarkingPosByNum(const CVisionModule* pVision, const int num)
{
	oppNum = num;
	if (pVision->Cycle() > logCycle[oppNum])
	{
		logCycle[oppNum] = pVision->Cycle();
		markingPoint[oppNum] = generatePos(pVision);
	}
	return markingPoint[oppNum];
	checkAllMarkingPos(pVision);
	return markingPoint[num];
}

void CMarkingPosV2::checkAllMarkingPos(const CVisionModule* pVision)
{
	//�÷������ڵĳ�����
	if (pVision->Cycle() > _logCycle) {
		int attackCnt = DefenceInfoNew::Instance()->getSteadyBallReceiverNum();
		areaList.clear();
		for (int i = 0; i < attackCnt; i++) {
			int bestEnemy = DefenceInfoNew::Instance()->getBestBallChaser();
			oppNum = DefenceInfoNew::Instance()->getSteadyBallReceiverList()[i];
			markingPoint[oppNum] = generatePos(pVision);
		}
	}
	vector<int>::iterator ir;
	int cnt = 0;
	//cout<<"areaList size is "<<areaList.size()<<endl;
	if (areaList.size() > 1) {
		std::sort(areaList.begin(), areaList.end());
		for (ir = areaList.begin(); ir != areaList.end(); ir++) {
			markingPoint[*ir] = markingPoint[*ir] + Utils::Polar2Vector(20 * cnt, (ourGoal - pVision->TheirPlayer(*ir).Pos()).dir());
			cnt = cnt + 1;
			//cout<<*ir<<" "<<cnt<<endl;
		}
	}
	_logCycle = pVision->Cycle();
}

bool CMarkingPosV2::isNearestBallReceiverBeDenied(const CVisionModule* pVision)
{
	double minDist = 1000;
	int receiverNum = 0;
	int attackNum = DefenceInfoNew::Instance()->getSteadyBallReceiverNum();
	if (pVision->Ball().Vel().mod() > NO_ADVANCE_BALL_VEL)
	{
		if (debug)
		{
			cout << "ball vel mod is  " << pVision->Ball().Vel().mod() << endl;
		}
		for (int i = 0; i < attackNum; ++i)
		{
			int oppNum = DefenceInfoNew::Instance()->getSteadyBallReceiverList()[i];
			double dist_opp_ball = pVision->TheirPlayer(oppNum).Pos().dist(pVision->Ball().Pos());
			if (dist_opp_ball < minDist)
			{
				minDist = dist_opp_ball;
				receiverNum = oppNum;
			}
		}
		if (debug)
		{
			cout << "receiver is  " << receiverNum << endl;
			cout << "deny   is   " << DENY_LOG[receiverNum] << endl;
		}
		if (0 != receiverNum)//��������receiver�������
		{
			if (true == DENY_LOG[receiverNum])
			{
				return true;
			}
		}
	}
	return false;
}

CGeoPoint CMarkingPosV2::generatePos(const CVisionModule* pVision)
{
	if (pVision->TheirPlayer(oppNum).Valid())//����
	{
		const int defenderNum = DefenceInfo::Instance()->getOurMarkDenfender(oppNum);
		const PlayerVisionT& opp = pVision->TheirPlayer(oppNum);
		const BallVisionT& ball = pVision->Ball();
		const CGeoPoint oppPos = opp.Pos();
		const CVector oppVel = opp.Vel();
		CVector opp2ourGoalVector = CVector(ourGoal - oppPos);

		//���˵���ʱ��
		CGeoPoint finalPoint = oppPos;
		CGeoPoint basePoint = oppPos;
		double markBuffer = 0;
		//�����ǶԶ��ֵĶ�̬Ԥ��
		double predictTime = 0.3;				//�ԶԷ�С���Ķ�̬Ԥ��ʱ��
		double angle_oppVel_opp2Goal = fabs(Utils::Normalize(opp2ourGoalVector.dir() - oppVel.dir()));

		//immortalKick
		if (false)//todo �·�����ϵ��ֲ������û��̫��(DefenceInfo::Instance()->getOppPlayerByNum(oppNum)->getAttributeValue("AChaseAbility") > CHASE_JUDGE_VALUE)
		{
			bool defenderOK = (defenderNum != 0) && pVision->OurPlayer(defenderNum).Valid();
			CGeoPoint preOppPos = oppPos + Utils::Polar2Vector(oppVel.mod() * predictTime, oppVel.dir());//ר����ImmortalKick������Ҫ���ݶ����ٶȳ����޸�predictTime
			bool oppFrontMe = preOppPos.dist(ourGoal) - Param::Vehicle::V2::PLAYER_SIZE > pVision->OurPlayer(defenderNum).Pos().dist(ourGoal);
			if (defenderOK && !oppFrontMe)//�Է�chaseKicker������ǰ��
			{
				markBuffer = CHASE_MARK_BUFFER;
				basePoint = preOppPos;
			} else
			{
				markBuffer = RECEIVER_MARK_BUFFER;
				basePoint = oppPos;
			}
			finalPoint = basePoint + Utils::Polar2Vector(2 * Param::Vehicle::V2::PLAYER_SIZE + markBuffer, opp2ourGoalVector.dir());
		} else//����һ����touchģʽ
		{
			markBuffer = RECEIVER_MARK_BUFFER;
			//���ٶ�ʱbasePoint������
			if (oppVel.mod() > 30 && angle_oppVel_opp2Goal > Param::Math::PI / 2.0)
				basePoint = oppPos + Utils::Polar2Vector(oppVel.mod() * predictTime, oppVel.dir());//ר����ImmortalKick������Ҫ���ݶ����ٶȳ����޸�predictTime
			finalPoint = basePoint + Utils::Polar2Vector(2 * Param::Vehicle::V2::PLAYER_SIZE + markBuffer, opp2ourGoalVector.dir());
			if (true == DENY_BALL_MODE && false == BACK_LOG[oppNum])//��ǰģʽ
			{
				CGeoLine ballVelLine = CGeoLine(ball.Pos(), ball.Vel().dir());
				CGeoPoint opp2ballVelLineProj = ballVelLine.projection(oppPos);
				if (debug)
				{
					GDebugEngine::Instance()->gui_debug_x(opp2ballVelLineProj, COLOR_YELLOW);
					GDebugEngine::Instance()->gui_debug_line(ball.Pos(), opp2ballVelLineProj, COLOR_WHITE);
				}
				if (false == DENY_LOG[oppNum])//������ǰ
				{
					//��ǰ����
					//�򵽴����ʱ������
					bool timeOK = opp2ballVelLineProj.dist(ball.Pos()) / (ball.Vel().mod() + 0.1) > DENY_TIME_LIMIT;
					//�Ƕ�����
					double angle_rBallVel_opp2Goal = fabs(Utils::Normalize(opp2ourGoalVector.dir() - Utils::Normalize(ball.Vel().dir() + Param::Math::PI)));
					bool angleOK = angle_rBallVel_opp2Goal < DENY_ANGLE_LIMIT;
					//���ϸ���������ȡ��ǰ�ж�֮ǰ��λ��������Ҫ���Ѿ��ﵽԭ���ص������ǰ
					CGeoPoint tempPos = oppPos + Utils::Polar2Vector(2 * Param::Vehicle::V2::PLAYER_SIZE, opp2ourGoalVector.dir());
					bool dist1OK = opp2ballVelLineProj.dist(oppPos) < DENY_DIST_LIMIT_1;
					bool dist2OK = tempPos.dist(pVision->OurPlayer(defenderNum).Pos()) < DENY_DIST_LIMIT_2;
					bool ballVelOK = ball.Vel().mod() < 500;
					if (debug)
					{
						//cout << opp2ballVelLineProj.dist(ball.Pos()) / (ball.Vel().mod() + 0.1)<<endl;
						cout << "deny condition  " << timeOK << "  " << angleOK << "  " << dist1OK << "  " << dist2OK << "  " << ballVelOK << endl;;
					}
					if (timeOK && angleOK && dist1OK && dist2OK && ballVelOK)
					{
						DENY_LOG[oppNum] = true;
					}
				}
				if (true == DENY_LOG[oppNum])//��ǰ������
				{
					//��ǰ����
					finalPoint = opp2ballVelLineProj + Utils::Polar2Vector(2 * Param::Vehicle::V2::PLAYER_SIZE + DENY_BUFFER, Utils::Normalize(ball.Vel().dir() + Param::Math::PI));
					if (debug)
					{
						GDebugEngine::Instance()->gui_debug_x(finalPoint, COLOR_YELLOW);
						QString denyBall;
						denyBall = QString("%d: deny Ball").arg(oppNum);
						GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(25 + oppNum * 10, -50), denyBall.toLatin1(), COLOR_YELLOW);
					}
					//�˳���ǰ�ж�
					bool defenderChange = DEFENDER_NUM_LOG[oppNum] != defenderNum;
					bool oppLeaves = oppPos.dist(opp2ballVelLineProj) > EXIT_DENY_DIST;
					if (defenderChange || oppLeaves)
					{
						DENY_LOG[oppNum] = false;
					}
				}
			}
		}
		DEFENDER_NUM_LOG[oppNum] = defenderNum;
		//markfield�ĵ�Ĳ���
		vector<MarkField> fieldList = DefenceInfo::Instance()->getMarkFieldList();
		vector<MarkField>::iterator ir;
		for (ir = fieldList.begin(); ir != fieldList.end(); ir++) {
			if (DefenceInfo::Instance()->checkInRecArea(oppNum, pVision, *ir)) {
				CGeoRectangle rect = CGeoRectangle((*ir)._upLeft, (*ir)._downRight);
				CGeoLine line = CGeoLine(pVision->TheirPlayer(oppNum).Pos(), ourGoal);
				CGeoLineRectangleIntersection intersect = CGeoLineRectangleIntersection(line, rect);
				if (intersect.intersectant()) {
					CGeoPoint tmpPoint;
					if (intersect.point1().dist(ourGoal) > intersect.point2().dist(ourGoal)) {
						tmpPoint = intersect.point2();
					} else {
						tmpPoint = intersect.point1();
					}
					tmpPoint = tmpPoint + Utils::Polar2Vector(10, (ourGoal - oppPos).dir());
					if (tmpPoint.dist(oppPos) > finalPoint.dist(oppPos)) {
						finalPoint = tmpPoint;
						areaList.push_back(oppNum);
					}
					//GDebugEngine::Instance()->gui_debug_msg(finalPoint,"F",COLOR_WHITE);
				}
			}
		}
		//GDebugEngine::Instance()->gui_debug_x(finalPoint,COLOR_WHITE);
		return finalPoint;
	}
	return CGeoPoint(0, 150);
	//return DefendUtils::reversePoint(DefendUtils::calcDefenderPointV2(DefendUtils::reversePoint(pVision->Ball().Pos()),CVector(pVision->Ball().Pos()-ourGoal).dir(),POS_SIDE_MIDDLE,0));
}



bool CMarkingPosV2::isInSpecialAreaBackLineMode(const CVisionModule* pVision, const int num)
{
	static bool theResult[Param::Field::MAX_PLAYER] = { false,false,false,false,false,false };
	static int theLogCycle[Param::Field::MAX_PLAYER] = { 0,0,0,0,0,0 };
	static int theBallLogCycle = 0;
	static CGeoPoint logBallPos = pVision->Ball().Pos();
	if (pVision->Cycle() > theLogCycle[num])
	{
		theResult[num] = false;
		if (true == SPECIAL_AREA_BACK_LINE_MODE)
		{
			const string refMsg = WorldModel::Instance()->CurrentRefereeMsg();
			if ("theirIndirectKick" == refMsg || "theirDirectKick" == refMsg || "gameStop" == refMsg)//����:���кз���
			{
				if (pVision->Cycle() > theBallLogCycle)
				{
					if (logBallPos.dist(pVision->Ball().Pos()) > 5)
					{
						logBallPos = pVision->Ball().Pos();
					}
					theBallLogCycle = pVision->Cycle();
				}
				if (logBallPos.x() < -Param::Field::PITCH_LENGTH / 2 + Param::Field::PENALTY_AREA_DEPTH + 10)//�������ҷ�����
				{
					const PlayerVisionT& opp = pVision->TheirPlayer(num);
					CGeoPoint oppPrePos = opp.Pos() + Utils::Polar2Vector(opp.Vel().mod() * SPECIAL_AREA_PRE_TIME, opp.Vel().dir());
					theResult[num] = checkInSpecialArea_A(oppPrePos, logBallPos);
				}
			}
		}
		theLogCycle[num] = pVision->Cycle();
	}
	return theResult[num];
}

bool CMarkingPosV2::checkInSpecialArea_A(const CGeoPoint p, const CGeoPoint ballPos)
{
	int theFlag = ballPos.y() > 0 ? -1 : 1;
	CGeoPoint p1 = CGeoPoint(-Param::Field::PITCH_LENGTH / 2.0 + Param::Field::PENALTY_AREA_DEPTH, theFlag * Param::Field::PENALTY_AREA_L / 2.0);
	double refDir = CVector(p1 - CGeoPoint(-Param::Field::PITCH_LENGTH / 2.0, 0)).dir();
	double goal2oppDir = CVector(p - CGeoPoint(-Param::Field::PITCH_LENGTH / 2.0, 0)).dir();
	if (p.x() < SPECIAL_AREA_X_BUFFER && (goal2oppDir * theFlag > refDir * theFlag))
	{
		if (debug) {
			QString backLine;
			backLine.append(QString("%1: special line").arg(oppNum));
			GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(125 + oppNum * 10, 50), backLine.toLatin1(), COLOR_YELLOW);
		}
		return true;
	}
	return false;
}

