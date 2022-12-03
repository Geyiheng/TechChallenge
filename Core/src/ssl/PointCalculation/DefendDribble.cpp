/************************************************************************/
/* Copyright (c) Shanghai Jiao Tong University							*/
/* Team��		SRC-SJTU								                */
/************************************************************************/
/* File:	  DefendDribble.cpp											*/
/* Func:	  ʼ��վ�����򳵺ͽ��򳵼� �����䴫��                       */
/* Author:	  �ͼ�ƽ 2020-10-24	 V2 2021-05-03 		                    */
/* Refer:	  ###														*/
/* E-mail:	  hejiaping2016@sjtu.edu.cn									*/
/* Version:	  0.0.2														*/
/************************************************************************/

#include "DefendDribble.h"
#include "param.h"
#include "utils.h"
#include "WorldModel/WorldModel.h"
#include "GDebugEngine.h"
#include <math.h>
#include "defence/DefenceInfo.h"
#include "BestPlayer.h"
#include <ssl/ParamManagerNew.h>

namespace {
	bool debug = false;

	const double GoalBuffer = 2;
	CGeoPoint GOAL_LEFT_POS;
	CGeoPoint GOAL_RIGHT_POS;
	CGeoPoint GOAL_CENTRE_POS;

	const double CHANGE_MODE_BUFFER = Param::Vehicle::V2::PLAYER_SIZE * 2.5; //����ģʽ��վ������ģʽת����ֵ
	const double ABSOLUTELY_IN = 3; //�����Ѿ����׽�����������ٿ���

	const double OPP_PRE_TIME = 1.5;//�Ե��˵Ĵ��ٶ�λ��Ԥ��ʱ��
	const double OPP_REAL_PRE_TIME = 1;//�Ե�����ʵλ�õ�Ԥ��ʱ��
};

CDefendDribble::CDefendDribble()
{
	//���㷨�ĳ�ʼ��
    _DeDribblePos = CGeoPoint(-Param::Field::PITCH_LENGTH / 2 + ParamManager::Instance()->PENALTY_AREA_DEPTH + 20, 0);
    _lastDeDribblePos = CGeoPoint(-Param::Field::PITCH_LENGTH / 2 + ParamManager::Instance()->PENALTY_AREA_DEPTH + 20, 0);
	GOAL_LEFT_POS = CGeoPoint(-Param::Field::PITCH_LENGTH / 2, -Param::Field::GOAL_WIDTH / 2 - GoalBuffer);
	GOAL_RIGHT_POS = CGeoPoint(-Param::Field::PITCH_LENGTH / 2, Param::Field::GOAL_WIDTH / 2 + GoalBuffer);
	GOAL_CENTRE_POS = CGeoPoint(-Param::Field::PITCH_LENGTH / 2, 0);
	_lastCycle = 0;
	//_forMarkingPos = CGeoPoint(0,0);
	//_headAttackEnemy = 0;
}

CDefendDribble::~CDefendDribble()
{}

CGeoPoint CDefendDribble::getDeDribblePos(const CVisionModule* pVision)
{
	if (pVision->Cycle() == _lastCycle) {
		return _lastDeDribblePos;
	}
	else {
		_lastCycle = pVision->Cycle();
	}
	//���ɷ��������
	generateDeDribblePos(pVision);
	return _DeDribblePos;
}

CGeoPoint CDefendDribble::generateDeDribblePos(const CVisionModule* pVision)
{
    _DeDribblePos = CGeoPoint(-Param::Field::PITCH_LENGTH / 2 + ParamManager::Instance()->PENALTY_AREA_DEPTH + 20, 0);
	//���²��ֲ����з������
	const BallVisionT& ball = pVision->Ball();
	const CGeoPoint ballPos = ball.Pos();
	const CBestPlayer::PlayerList& oppList = BestPlayer::Instance()->theirFastestPlayerToBallList();
	int enemyNum = oppList[0].num;     //�°��ж�bestplayer��������advanceһ��,һ���Ǵ���
	int SecondEnemyNum = 0;
	_TheirPlayerXList.clear();         //�з�x��������,�ҳ������ҷ�����ĳ�
	int n = 0;
	while (n <= Param::Field::MAX_PLAYER) {
		if (!pVision->TheirPlayer(n).Valid()) { _TheirPlayerXList.push_back(TheirPlayerXPair(n, Param::Field::PITCH_LENGTH / 2)); n++; continue; }
		_TheirPlayerXList.push_back(TheirPlayerXPair(n, pVision->TheirPlayer(n).Pos().x()));
		n++;
	}
	std::sort(_TheirPlayerXList.begin(), _TheirPlayerXList.end());
	//int enemyNum = BestPlayer::Instance()->getTheirBestPlayer(); //�ҵ��Է����򳵣�Ŀǰ�ж�Ϊbestplayer,���廹��ʵ�ʲ���
	if (_TheirPlayerXList[0].num == oppList[1].num) {
		SecondEnemyNum = oppList[1].num;    //�����ŶԽ��򳵽����жϣ�����в���ŵڶ��ĳ������Ч�����ã�ѡ�����ҷ���������ĳ�
	}
	else if (_TheirPlayerXList[0].num == oppList[0].num) {
		SecondEnemyNum = _TheirPlayerXList[1].num;
	}
	else { SecondEnemyNum = _TheirPlayerXList[0].num; }
	const PlayerVisionT& enemyLeader = pVision->TheirPlayer(enemyNum);
	const PlayerVisionT& enemyReceiver = pVision->TheirPlayer(SecondEnemyNum);
	CGeoPoint enemyPos = enemyLeader.Pos();
	CGeoPoint enemyReceiverPos = enemyReceiver.Pos();
	GDebugEngine::Instance()->gui_debug_msg(enemyReceiverPos, "Receiver Opp!!!!!!!!!!", COLOR_WHITE);
	const CVector Receiver2Opp = enemyPos - enemyReceiverPos;
	const double Receiver2Oppdist = Receiver2Opp.mod();
	CGeoPoint targetPos = CGeoPoint(-315, 0);
	if (enemyLeader.Valid() && enemyReceiver.Valid() && enemyReceiverPos.x() > 0)                            //�����ڶԷ��볡��վ�ڰ볡�ߣ�ͬy����
	{
		targetPos = CGeoPoint(0, enemyReceiverPos.y());
		_DeDribblePos = targetPos;
	}
	else if (enemyLeader.Valid() && enemyReceiver.Valid())         //�����һ����λ����´�������
	{
		double angle_enemy2goal = CVector(GOAL_CENTRE_POS - enemyPos).dir();
		targetPos = enemyReceiverPos + Utils::Polar2Vector(Param::Vehicle::V2::PLAYER_SIZE * 4 + enemyReceiver.Vel().mod() * OPP_REAL_PRE_TIME, Receiver2Opp.dir());
		int moveStep = 0;
		if(Utils::InOurPenaltyArea(enemyReceiverPos, 30) || enemyReceiverPos.x() > -30 ){ targetPos = enemyReceiverPos + Utils::Polar2Vector(Param::Vehicle::V2::PLAYER_SIZE * 2 + enemyReceiver.Vel().mod() * OPP_REAL_PRE_TIME, Receiver2Opp.dir()); }
		if (Receiver2Oppdist < Param::Vehicle::V2::PLAYER_SIZE * 2)
		{
			targetPos = enemyReceiverPos + Utils::Polar2Vector(Receiver2Oppdist / 2.0, Receiver2Opp.dir());
		}
		while (Utils::InOurPenaltyArea(targetPos, Param::Vehicle::V2::PLAYER_SIZE * 0.5))    //��ֹ���ɵ��ڽ�����
		{
			moveStep += 1;
			targetPos = enemyReceiverPos + Utils::Polar2Vector(Param::Vehicle::V2::PLAYER_SIZE * (2 + moveStep), Receiver2Opp.dir());
		}
		_DeDribblePos = targetPos;
	}
	//����������Ͻ�����վ�����ߣ�����ͬdefendHead
	else if (enemyLeader.Valid() && enemyReceiver.Valid() && Utils::InOurPenaltyArea(enemyPos, CHANGE_MODE_BUFFER) && Utils::InOurPenaltyArea(enemyReceiverPos, CHANGE_MODE_BUFFER) && !Utils::InOurPenaltyArea(enemyLeader.Pos(), ABSOLUTELY_IN))
	{
		targetPos = enemyReceiverPos + Utils::Polar2Vector(enemyReceiver.Vel().mod()*OPP_REAL_PRE_TIME, enemyReceiver.Vel().dir());
		CGeoPoint RTargetPos = DefendUtils::reversePoint(targetPos);
		CGeoPoint RblockPos = DefendUtils::calcDefenderPointV2(RTargetPos, CVector(DefendUtils::reversePoint(GOAL_CENTRE_POS) - RTargetPos).dir(), POS_SIDE_MIDDLE, 0, 1.0);
		_DeDribblePos = DefendUtils::reversePoint(RblockPos);
	}

	_lastDeDribblePos = _DeDribblePos;
	return _DeDribblePos;

}
