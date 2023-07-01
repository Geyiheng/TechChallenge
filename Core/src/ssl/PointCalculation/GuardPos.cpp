#include "GuardPos.h"
#include "GDebugEngine.h"
#include "Global.h"
#include "param.h"
#include "TaskMediator.h"
#include "defenceNew/DefenceInfoNew.h"

namespace {
	const CGeoPoint GOAL_MIDDLE = CGeoPoint(-Param::Field::PITCH_LENGTH / 2, 0);
	const CGeoPoint GOAL_LEFT = CGeoPoint(-Param::Field::PITCH_LENGTH / 2, -Param::Field::GOAL_WIDTH / 2);
	const CGeoPoint GOAL_RIGHT = CGeoPoint(-Param::Field::PITCH_LENGTH / 2, Param::Field::GOAL_WIDTH / 2);
	const double MIN_DIST_TO_PENALTY = Param::Vehicle::V2::PLAYER_SIZE * 2;
	const double MAX_DIST_TO_PENALTY = Param::Vehicle::V2::PLAYER_SIZE * 5;
	const double MIN_DIST_TO_TEAMMATE = Param::Vehicle::V2::PLAYER_SIZE * 2 + Param::Field::BALL_SIZE * 2;
	// �����ƶ�λ��
	CGeoPoint PENALTY_RIGHT_UP(-Param::Field::PITCH_LENGTH / 2 + Param::Field::PENALTY_AREA_DEPTH + MIN_DIST_TO_PENALTY, -(Param::Field::PENALTY_AREA_WIDTH / 2 + MIN_DIST_TO_PENALTY));
	CGeoPoint PENALTY_RIGHT_DOWN(-Param::Field::PITCH_LENGTH / 2 + Param::Field::PENALTY_AREA_DEPTH + MIN_DIST_TO_PENALTY, Param::Field::PENALTY_AREA_WIDTH / 2 + MIN_DIST_TO_PENALTY);
	CGeoPoint PENALTY_LEFT_UP(-Param::Field::PITCH_LENGTH / 2, -(Param::Field::PENALTY_AREA_WIDTH / 2 + MIN_DIST_TO_PENALTY));
	CGeoPoint PENALTY_LEFT_DOWN(-Param::Field::PITCH_LENGTH / 2, Param::Field::PENALTY_AREA_WIDTH / 2 + MIN_DIST_TO_PENALTY);
	CGeoRectangle guardMoveRec(PENALTY_LEFT_UP, PENALTY_RIGHT_DOWN);
	// ��Ե�㣬����λ�ò�Ӧ��̫����
	CGeoPoint LEFTBACK_CRITICAL_POINT = CGeoPoint(-(Param::Field::PITCH_LENGTH / 2 - Param::Field::PENALTY_AREA_DEPTH - Param::Vehicle::V2::PLAYER_SIZE), -Param::Field::PITCH_WIDTH / 2);
	CGeoPoint RIGHTBACK_CRITICAL_POINT = CGeoPoint(-(Param::Field::PITCH_LENGTH / 2 - Param::Field::PENALTY_AREA_DEPTH - Param::Vehicle::V2::PLAYER_SIZE), Param::Field::PITCH_WIDTH / 2);

	bool DEBUG_GUARD_POS = true;
	const double VALID_THRESHOLD = 500;
	// ����������
	int dangerVel;
	int dangerDist;
}

CGuardPos::CGuardPos() : _cycle(0), _guardNum(0), ready_cnt(0), coming_cnt(0), missing_cnt(0)
{
	std::fill_n(_status, Param::Field::MAX_PLAYER, TbackStatus());

	dangerVel = ParamManager::Instance()->dangerVel;
	dangerDist = ParamManager::Instance()->dangerDist;
}
// lua�ӿڣ������ɵ�λ
CGeoPoint CGuardPos::backPos(int guardNum, int index)
{
	_guardNum = std::clamp(guardNum, 1, Param::Field::MAX_PLAYER);
	index = std::clamp(index, 1, _guardNum);
	if (vision->Cycle() != _cycle) {
		_cycle = vision->Cycle();
		checkBackPos();
		generatePos(_status, _status + ready_cnt, chooseTargetPos());
		adjustBackPos();
	}
	return _status[index - 1].pos;
}
// lua�ӿڣ���ȷ��˭��multiback
void CGuardPos::setBackNum(int realNum, int index)
{
	if (index < 1 || index > Param::Field::MAX_PLAYER) {
		std::cout << "ERROR in GuardPos: back num out of range: " << index << "\n";
		return;
	}
	_status[index - 1].cycle = vision->Cycle();
	_status[index - 1].num = realNum;
	TaskMediator::Instance()->setMultiBack(realNum);
}
// ���ʵ�ʺ���������ʵ�����õ���Ϣ������һ����
void CGuardPos::checkBackPos()
{
	ready_cnt = coming_cnt = missing_cnt = 0;
	for (int i = 0; i < Param::Field::MAX_PLAYER; i++) {
		if (i < _guardNum) {
			if (TaskMediator::Instance()->isMultiBack(_status[i].num)) {
				CGeoPoint playerPos = vision->OurPlayer(_status[i].num).Pos();
				if (playerPos.dist(_status[i].pos) > dangerDist && !Utils::InOurPenaltyArea(playerPos, MAX_DIST_TO_PENALTY)) {
					_status[i].state = COMING_BACK;
					coming_cnt++;
				} else {
					_status[i].state = READY_BACK;
					ready_cnt++;
				}
			} else { //�������㣬missing
				_status[i].state = MISSING_BACK;
				missing_cnt++;
			}
		} else {
			_status[i].state = NOT_BACK;

		}
	}
	// ��������ready-coming-missing-not
	std::sort(_status, _status + Param::Field::MAX_PLAYER, [](auto a, auto b) {return a.state >= b.state; });
}
//������Ҫ��λ
void CGuardPos::generatePos(TbackStatus* status_begin, TbackStatus* status_end, CGeoPoint target)
{
	int backNum = status_end - status_begin;
	if (backNum == 0)
		return;
	CGeoLine targetToMiddle(target, GOAL_MIDDLE);
	CGeoLineRectangleIntersection _intersecMiddle(targetToMiddle, guardMoveRec);
	if (_intersecMiddle.intersectant()) {
		bool leftValid = true, rightValid = true;
		if (backNum % 2 == 1) { // �������м�һ���������ſ�
			(*(status_begin + backNum / 2)).pos = _intersecMiddle.point2().dist(GOAL_MIDDLE) < 1e-8 ? _intersecMiddle.point1() : _intersecMiddle.point2(); // ����ε����������з��������ĵ�һ��
			for (int i = 0; i < backNum / 2; i++) { // ���м�����������
				leftValid = leftNextPos((*(status_begin + backNum / 2 - i)).pos, (*(status_begin + backNum / 2 - i - 1)).pos);
				rightValid = rightNextPos((*(status_begin + backNum / 2 + i)).pos, (*(status_begin + backNum / 2 + i + 1)).pos);
			}
		} else { // ż����ֱ�������ſ�
			CGeoPoint intersecPos = _intersecMiddle.point2().dist(GOAL_MIDDLE) < 1e-8 ? _intersecMiddle.point1() : _intersecMiddle.point2();
			leftValid = leftNextPos(intersecPos, (*(status_begin + backNum / 2 - 1)).pos, MIN_DIST_TO_TEAMMATE / 2);
			rightValid = rightNextPos(intersecPos, (*(status_begin + backNum / 2)).pos, MIN_DIST_TO_TEAMMATE / 2);
			for (int i = 0; i < backNum / 2 - 1; i++) {
				leftValid = leftNextPos((*(status_begin + backNum / 2 - 1 - i)).pos, (*(status_begin + backNum / 2 - 2 - i)).pos);
				rightValid = rightNextPos((*(status_begin + backNum / 2 + i)).pos, (*(status_begin + backNum / 2 + 1 + i)).pos);
			}
		}
		if (!leftValid) { // �ŵ��߽��ˣ�����������ұ���
			for (int i = 0; i < backNum - 1; i++) {
				rightNextPos((*(status_begin + i)).pos, (*(status_begin + i + 1)).pos);
			}
		}
		if (!rightValid) {
			for (int i = 1; i < backNum; i++) {
				rightNextPos((*(status_end - i)).pos, (*(status_end - i - 1)).pos);
			}
		}
	} else {
		std::cout << "GUARDPOS: NO INTERSECTION!!!\n";
	}
	if (DEBUG_GUARD_POS) {
		for (int i = 0; i < backNum; i++) {
			GDebugEngine::Instance()->gui_debug_line((*(status_begin + i)).pos, target, COLOR_CYAN);
		}
	}
}
//todo SYLG Ŀ������
CGeoPoint CGuardPos::chooseTargetPos()
{
	const BallVisionT& Ball = vision->Ball();
	const PlayerVisionT enemy = vision->TheirPlayer(DefenceInfoNew::Instance()->getBestBallChaser());
	CGeoPoint defendTarget;
	if (Ball.Valid()) {
		if (Ball.Pos().dist(enemy.Pos()) < dangerDist && GOAL_MIDDLE.dist(enemy.Pos()) < 300) { //�з������źܽ������з����׼
			defendTarget = enemy.Pos();
		} else {
			defendTarget = Ball.Pos();
		}
	} else {
		defendTarget = enemy.Pos();
	}
	//if ((defendTarget - GOAL_MIDDLE).dir() > (RIGHTBACK_CRITICAL_POINT - GOAL_MIDDLE).dir())
	//	defendTarget = RIGHTBACK_CRITICAL_POINT;
	//else if ((defendTarget - GOAL_MIDDLE).dir() < (LEFTBACK_CRITICAL_POINT - GOAL_MIDDLE).dir())
	//	defendTarget = LEFTBACK_CRITICAL_POINT;
	return defendTarget;
	/*
	if (vision->TheirPlayer(enemyNum).Pos().dist(defendTarget) < 20 && defendTarget.x() < -250 && fabs(diffAngle) < atan(0.5))
	double diffAngle = vision->TheirPlayer(enemyNum).Dir() - (GOAL_MIDDLE - defendTarget).dir();
	*/
}
//��COMING_BACK���й滮
void CGuardPos::adjustBackPos()
{
	if (ready_cnt == 0) {// ������� ���쵽λ
		generatePos(_status + ready_cnt, _status + ready_cnt + coming_cnt, chooseTargetPos());
	} else {
		generatePos(_status + ready_cnt, _status + ready_cnt + coming_cnt, chooseTargetPos());
		// SYLG 2023 ����һ�еĴ����ƺ������ԣ��Ȳ�д��
		//for (int i = 0; i < coming_cnt; i++) {
		//	// 1 �ж�������
		//	// 2 ����ֱ��next���м䵲ǰ��
		//}
	}
	// legacy
	//if (!CheckBackPos || (_guardNum != 3 && _guardNum != 2)) return;
	//const BallVisionT& Ball = vision->Ball();
	//int bestenemy = DefenceInfoNew::Instance()->getBestBallChaser();
	//double defendTargetPosX = Ball.Valid() ? Ball.X() : vision->TheirPlayer(bestenemy).X();
	//double defendTargetPosY = Ball.Valid() ? Ball.Y() : vision->TheirPlayer(bestenemy).Y();
	//double defendTargetVelX = Ball.Valid() ? Ball.VelX() : vision->TheirPlayer(bestenemy).VelX();
	//double defendTargetVelY = Ball.Valid() ? Ball.VelY() : vision->TheirPlayer(bestenemy).VelY();
	//bool leftDanger = (defendTargetPosX > PENALTY_RIGHT_UP.x() && defendTargetVelY < -dangerVel) ||
	//	(defendTargetPosX < PENALTY_RIGHT_UP.x() && defendTargetPosY > 0 && defendTargetVelX > dangerVel) ||
	//	(defendTargetPosX < PENALTY_RIGHT_UP.x() && defendTargetPosY < 0 && defendTargetVelX < -dangerVel);
	//bool rightDanger = (defendTargetPosX > PENALTY_RIGHT_UP.x() && defendTargetVelY > dangerVel) ||
	//	(defendTargetPosX < PENALTY_RIGHT_UP.x() && defendTargetPosY > 0 && defendTargetVelX < -dangerVel) ||
	//	(defendTargetPosX < PENALTY_RIGHT_UP.x() && defendTargetPosY < 0 && defendTargetVelX > dangerVel);
	//CGeoPoint midPoint = _backPos[1], rightPoint = _backPos[2], leftPoint = _backPos[0];
	//GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, 0), std::to_string(ready_cnt).c_str());
	//switch (ready_cnt) {
	//case 1:
	//	GDebugEngine::Instance()->gui_debug_msg(DebugPoint, "SingleBack");
	//	if (_guardNum == 2) {
	//		midPoint = _backPos[0].midPoint(_backPos[1]);
	//		rightPoint = _backPos[1];
	//	}
	//	if (leftDanger)
	//		_backPos[ready_cnt] = midPoint.midPoint(leftPoint);
	//	else if (rightDanger)
	//		_backPos[ready_cnt] = midPoint.midPoint(rightPoint);
	//	else
	//		_backPos[ready_cnt] = midPoint;
	//	break;
	//case 2:
	//	if (_guardNum == 2) break;
	//	if (!_readyBack[1]) { // ?��????????????��??????left/rightBack
	//		GDebugEngine::Instance()->gui_debug_msg(DebugPoint, "MiddleBack");
	//		_backPos[0] = midPoint.midPoint(leftPoint);
	//		_backPos[2] = midPoint.midPoint(rightPoint);
	//	} else if (!_readyBack[0] && leftDanger) { // ??????????????????????
	//		GDebugEngine::Instance()->gui_debug_msg(DebugPoint, "LeftBack");
	//		_backPos[2] = midPoint;
	//		_backPos[1] = leftPoint;
	//	} else if (!_readyBack[2] && rightDanger) { // ?????????????????????
	//		GDebugEngine::Instance()->gui_debug_msg(DebugPoint, "RightBack");
	//		_backPos[0] = midPoint;
	//		_backPos[1] = rightPoint;
	//	}
	//	break;
	//default:
	//	GDebugEngine::Instance()->gui_debug_msg(DebugPoint, "NoneBack");
	//	break;
	//}
}

bool CGuardPos::leftNextPos(CGeoPoint basePos, CGeoPoint& nextPos, double dist)
{
	if (dist < 0) dist = MIN_DIST_TO_TEAMMATE;
	if (basePos.y() >= PENALTY_RIGHT_UP.y() + dist && fabs(basePos.x() - PENALTY_RIGHT_DOWN.x()) < 1e-8) { // ��������
		nextPos = basePos + CVector(0, -dist);
	} else if (basePos.y() < PENALTY_RIGHT_UP.y() + dist) {
		if (fabs(basePos.x() - PENALTY_RIGHT_UP.x()) < 1e-8) { // �������ս�
			nextPos = PENALTY_RIGHT_UP + CVector(-sqrt(pow(dist, 2) - pow(basePos.y() - PENALTY_RIGHT_UP.y(), 2)), 0);
		} else if (fabs(basePos.x() - PENALTY_LEFT_UP.x()) >= dist) { // ����������
			nextPos = basePos + CVector(-dist, 0);
		} else { // �������ˣ��Ų���
			nextPos = PENALTY_LEFT_UP;
			return false;
		}
	} else {
		if (fabs(basePos.x() - PENALTY_RIGHT_DOWN.x()) <= dist) { // �Ҳ�����ս�
			nextPos = PENALTY_RIGHT_DOWN + CVector(0, -sqrt(pow(dist, 2) - pow(basePos.x() - PENALTY_RIGHT_DOWN.x(), 2)));
		} else { // �Ҳ��������
			nextPos = basePos + CVector(dist, 0);
		}
	}
	return true;
}

bool CGuardPos::rightNextPos(CGeoPoint basePos, CGeoPoint& nextPos, double dist)
{
	if (dist < 0) dist = MIN_DIST_TO_TEAMMATE;
	if (basePos.y() <= PENALTY_RIGHT_DOWN.y() - dist && fabs(basePos.x() - PENALTY_RIGHT_UP.x()) < 1e-8) {
		nextPos = basePos + CVector(0, dist);
	} else if (basePos.y() > PENALTY_RIGHT_DOWN.y() - dist) {
		if (fabs(basePos.x() - PENALTY_RIGHT_DOWN.x()) < 1e-8) {
			nextPos = PENALTY_RIGHT_DOWN + CVector(-sqrt(pow(dist, 2) - pow(basePos.y() - PENALTY_RIGHT_DOWN.y(), 2)), 0);
		} else if (fabs(basePos.x() - PENALTY_LEFT_DOWN.x()) >= dist) {
			nextPos = basePos + CVector(-dist, 0);
		} else {
			nextPos = PENALTY_LEFT_DOWN;
			return false;
		}
	} else {
		if (fabs(basePos.x() - PENALTY_RIGHT_UP.x()) <= dist) {
			nextPos = PENALTY_RIGHT_UP + CVector(0, sqrt(pow(dist, 2) - pow(basePos.x() - PENALTY_RIGHT_UP.x(), 2)));
		} else {
			nextPos = basePos + CVector(dist, 0);
		}
	}
	return true;
}
