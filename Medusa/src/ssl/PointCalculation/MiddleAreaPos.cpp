/************************************************************************/
/* Copyright (c) ShanghaiJiaoTongUniversity							    */
/* Team��		SRC											            */
/************************************************************************/
/* File:	  MiddleAreaPos.cpp											*/
/* Func:	  �����Ұ볡�в࣬����������ʱ�����ڷ��ضԷ���Ӧ��Աͷ��ĵ�*/
/* Author:	  �ͼ�ƽ 2020-07-18											*/
/* Refer:	  ###														*/
/* E-mail:	  hejiaping2016@sjtu.edu.cn									*/
/************************************************************************/

#include "CornerAreaPos.h"
#include "param.h"
#include "utils.h"
#include "WorldModel/WorldModel.h"
#include "GDebugEngine.h"
#include <math.h>
#include "defence/DefenceInfo.h"
#include <ssl/ParamManagerNew.h>

namespace{
	bool debug = false;	

	const double GoalBuffer = 2;
	CGeoPoint GOAL_LEFT_POS;
	CGeoPoint GOAL_RIGHT_POS;
	CGeoPoint GOAL_CENTRE_POS;

	const double CHANGE_BUFFER = 50;
	const double ABSOLUTELY_IN = 5; //�����Ѿ����׽�����������ٿ���

	const double extremeMiddleDefDist = 90;//һ�߷�����ʱ�����ͷ��λ��yֵ�����ñߣ����趢��
	const double OPP_PRE_TIME = 1.5;//�Ե��˵Ĵ��ٶ�λ��Ԥ��ʱ��
	const double OPP_REAL_PRE_TIME = 0.2;//�Ե�����ʵλ�õ�Ԥ��ʱ��
	const double DEAL_HIT_DIST = 35;//�������������Ͻ���ʱ���ٽ���״̬
	const double EXTREME_ANGLE = Param::Math::PI * 37 / 180;  //��defendMiddle���ռ�
};

CMiddleAreaPos::CMiddleAreaPos()
{
	//���㷨�ĳ�ʼ��
	_MAPos = CGeoPoint(0,0);
	_lastMAPos = CGeoPoint(0,0);
	GOAL_LEFT_POS = CGeoPoint(-Param::Field::PITCH_LENGTH / 2, -Param::Field::GOAL_WIDTH / 2 - GoalBuffer);
	GOAL_RIGHT_POS = CGeoPoint(-Param::Field::PITCH_LENGTH / 2, Param::Field::GOAL_WIDTH / 2 + GoalBuffer);
	GOAL_CENTRE_POS = CGeoPoint(-Param::Field::PITCH_LENGTH / 2,0);
	_MlastCycle = 0;
	//_forMarkingPos = CGeoPoint(0,0);
	//_headAttackEnemy = 0;
}

CMiddleAreaPos::~CMiddleAreaPos()
{}

CGeoPoint CMiddleAreaPos::getMiddleAreaPos(const CVisionModule* pVision)
{
	if (pVision->Cycle() == _MlastCycle) {
		return _lastMAPos;
	} else{
		_MlastCycle = pVision->Cycle();
	}
	//����������
	setPos(generateMiddlePos(pVision));	
	return _MAPos;
}

//bool CCornerAreaPos::isUsed(const CVisionModule* pVision)
//{
//	if (pVision->Cycle() - _MlastCycle > 3)
//	{
//		_headAttackEnemy = 0;
//		_dealWithHit = false;
//		return false;
//	} else return true;
//}

CGeoPoint CMiddleAreaPos::generateMiddlePos(const CVisionModule* pVision)
{
	_MAPos = CGeoPoint(-335,0); //���޸�ΪBrazil zhyaic
	//���²���û�з�����㣡��
	const BallVisionT& ball = pVision->Ball();
	const CGeoPoint ballPos = ball.Pos();
	static posSide ballSide = POS_SIDE_RIGHT;
	const string refMsg = WorldModel::Instance()->CurrentRefereeMsg();
	/*if ("theirIndirectKick" == refMsg || "theirDirectKick" == refMsg || "theirKickOff" == refMsg || "gameStop" == refMsg)
	{
		if (ballPos.y() > 0)
		{
			ballSide = POS_SIDE_RIGHT;
		} else ballSide = POS_SIDE_LEFT;
	}*/	
	if (ballPos.y() > 0)
	{
		ballSide = POS_SIDE_RIGHT;
	}
	else ballSide = POS_SIDE_LEFT;
	int attackNum = DefenceInfo::Instance()->getAttackNum();
	//��ȡ�Ʒ��ɹ��������Ա
	int enemyBreakMe = -1;
	double minDist = 1000;
	for (int i = 0;i < attackNum;++i)
	{
		int enemyNum = DefenceInfo::Instance()->getAttackOppNumByPri(i);
		if (!Utils::PlayerNumValid(enemyNum)) {
			continue;
		}
		const PlayerVisionT& opp = pVision->TheirPlayer(enemyNum);
		if (opp.Valid() && !Utils::InOurPenaltyArea(opp.Pos(),ABSOLUTELY_IN))
		{
			CGeoPoint oppPos = opp.Pos();
			CGeoPoint oppPrePos = oppPos + Utils::Polar2Vector(opp.Vel().mod()*OPP_PRE_TIME,opp.Vel().dir());
			if (POS_SIDE_LEFT == ballSide)
			{
				if (oppPrePos.y() > extremeMiddleDefDist)
				{
					continue;
				}
			} else if (POS_SIDE_RIGHT == ballSide)
			{
				if (oppPrePos.y() < -1 * extremeMiddleDefDist)
				{
					continue;
				}
			}
			//�ж��Ƿ񶢷�����
			bool markWell = true;
			double markerDistFactor = DefenceInfo::Instance()->getOppPlayerByNum(enemyNum)->getAttributeValue("AMarkerDistFactor");
			double markerDirFactor = DefenceInfo::Instance()->getOppPlayerByNum(enemyNum)->getAttributeValue("AMarkerDirFactor");
			if (markerDirFactor > Param::Math::PI * 100 / 180 ||
				markerDistFactor > 150						||
				sin(markerDirFactor) * markerDistFactor > 30)
			{
				markWell = false;
			}
			if (false == markWell)
			{
				double nowDist = oppPos.dist(GOAL_CENTRE_POS);
				if (nowDist < minDist)
				{
					minDist = nowDist;
					enemyBreakMe = enemyNum;
				}
			}
		}
	}
	//���û���Ʒ��ɹ��Ķ�Ա����ֻ�ҵ��������Ķ�Ա
	if (enemyBreakMe<0)
	{
		minDist = 1000;
		for (int i = 0;i < attackNum;++i)
		{
			int enemyNum = DefenceInfo::Instance()->getAttackOppNumByPri(i);
			if (!Utils::PlayerNumValid(enemyNum)) {
				continue;
			}
			const PlayerVisionT& opp = pVision->TheirPlayer(enemyNum);
			if (opp.Valid() && !Utils::InOurPenaltyArea(opp.Pos(),ABSOLUTELY_IN))
			{				
				CGeoPoint oppPos = opp.Pos();
				CGeoPoint oppPrePos = oppPos + Utils::Polar2Vector(opp.Vel().mod()*OPP_PRE_TIME,opp.Vel().dir());
				if (POS_SIDE_LEFT == ballSide)
				{
					if (oppPrePos.y() > extremeMiddleDefDist)
					{
						continue;
					}
				} else if (POS_SIDE_RIGHT == ballSide)
				{
					if (oppPrePos.y() <  -1 * extremeMiddleDefDist)
					{
						continue;
					}
				}
				double nowDist = oppPos.dist(GOAL_CENTRE_POS);
				if (nowDist < minDist)
				{
					minDist = nowDist;
					enemyBreakMe = enemyNum;
				}
			}
		}
	}
	//ȡ��
	CGeoPoint targetPos = CGeoPoint(0,0);
	if (POS_SIDE_LEFT == ballSide)
	{
		targetPos = CGeoPoint(-(Param::Field::PITCH_LENGTH/2-queryParamByName("data\\ssl\\params\\params.xml", "PENALTY_AREA_DEPTH")),Param::Field::PENALTY_AREA_WIDTH/2); // ���޸�ΪBrazil zhyaic
	} else if (POS_SIDE_RIGHT == ballSide)
	{
		targetPos = CGeoPoint(-(Param::Field::PITCH_LENGTH/2-queryParamByName("data\\ssl\\params\\params.xml", "PENALTY_AREA_DEPTH")),-Param::Field::PENALTY_AREA_WIDTH/2); // ���޸�ΪBrazil zhyaic
	}

	if (enemyBreakMe>=0)//���˴���
	{
		const PlayerVisionT& headOpp = pVision->TheirPlayer(enemyBreakMe);
		targetPos = headOpp.Pos() + Utils::Polar2Vector(headOpp.Vel().mod()*OPP_REAL_PRE_TIME,headOpp.Vel().dir());
		double angle_goal2target = CVector(targetPos - GOAL_CENTRE_POS).dir();
		if (POS_SIDE_LEFT == ballSide)
		{
			if (angle_goal2target >= EXTREME_ANGLE)
			{
				targetPos = GOAL_CENTRE_POS + Utils::Polar2Vector(150*Param::Field::RATIO,0.8* EXTREME_ANGLE);  //�ڳ����г����ط�Χʱ����������������ϣ��ص���Ȼ���г�
			}else if (angle_goal2target <= -1*EXTREME_ANGLE)
			{
				targetPos = GOAL_CENTRE_POS + Utils::Polar2Vector(150*Param::Field::RATIO,-0.8*EXTREME_ANGLE);
			}
		}else if (POS_SIDE_RIGHT == ballSide)
		{
			if (angle_goal2target <= -1*EXTREME_ANGLE)
			{
				targetPos = GOAL_CENTRE_POS + Utils::Polar2Vector(150*Param::Field::RATIO,-0.8*EXTREME_ANGLE);
			}else if (angle_goal2target >= EXTREME_ANGLE)
			{
				targetPos = GOAL_CENTRE_POS + Utils::Polar2Vector(150*Param::Field::RATIO,0.8*EXTREME_ANGLE);
			}
		}		
	}
	//��ʼ������㣬ע�⣡��
	CGeoPoint RTargetPos = DefendUtils::reversePoint(targetPos);
	CGeoPoint RblockPos = DefendUtils::calcDefenderPointV2(RTargetPos,CVector(DefendUtils::reversePoint(GOAL_CENTRE_POS) - RTargetPos).dir(),POS_SIDE_MIDDLE,0,1.0);
	_MAPos = DefendUtils::reversePoint(RblockPos);
	//��������붢�����ͻ������
	//if (WorldModel::Instance()->getEnemySituation().queryMarked(enemyBreakMe))//���˶���
	//{
	//	if (Utils::InOurPenaltyArea(targetPos,CHANGE_BUFFER))//���ҽ�����һȦ
	//	{
	//		int defenderNum = WorldModel::Instance()->getOurMarkDenfender(enemyBreakMe);
	//		if (pVision->OurPlayer(defenderNum).Valid() && _MAPos.dist(pVision->OurPlayer(defenderNum).Pos()) < DEAL_HIT_DIST)//���Ҷ��˵�ͷ�ͷ������ܽ�
	//		{
	//			_headAttackEnemy = enemyBreakMe;
	//			_dealWithHit = true;
	//			const PlayerVisionT& me = pVision->OurPlayer(defenderNum);
	//			if (/*me.Y() < _MAPos.y()*/CVector(me.Pos() - GOAL_CENTRE_POS).dir() - CVector(_MAPos - GOAL_CENTRE_POS).dir() < 0)//������ �� ��ͷ������� ����ͷ�����ߵ��ұ�
	//			{
	//				RblockPos = DefendUtils::calcDefenderPoint(RTargetPos,CVector(DefendUtils::reversePoint(GOAL_CENTRE_POS) - RTargetPos).dir(),POS_SIDE_LEFT);//ע�������POS_SIDE_LEFTû�д�
	//				_forMarkingPos = DefendUtils::reversePoint(DefendUtils::calcDefenderPoint(RTargetPos,CVector(DefendUtils::reversePoint(GOAL_CENTRE_POS) - RTargetPos).dir() + 0.05,POS_SIDE_RIGHT));
	//				_MAPos = DefendUtils::reversePoint(RblockPos);
	//			} else 
	//			{
	//				RblockPos = DefendUtils::calcDefenderPoint(RTargetPos,CVector(DefendUtils::reversePoint(GOAL_CENTRE_POS) - RTargetPos).dir(),POS_SIDE_RIGHT);
	//				_forMarkingPos = DefendUtils::reversePoint(DefendUtils::calcDefenderPoint(RTargetPos,CVector(DefendUtils::reversePoint(GOAL_CENTRE_POS) - RTargetPos).dir() - 0.05,POS_SIDE_LEFT));
	//				_MAPos = DefendUtils::reversePoint(RblockPos);
	//			}
	//		}
	//	}			
	//}

	_lastMAPos = _MAPos;
	return _MAPos;
}
