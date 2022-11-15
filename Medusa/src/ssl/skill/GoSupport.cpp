#include "GoSupport.h"
#include <Vision/VisionModule.h>
#include <WorldModel/KickStatus.h>
#include "RobotSensor.h"
#include "skill/Factory.h"
#include <WorldModel/WorldModel.h>
#include "WorldModel/DribbleStatus.h"
#include <utils.h>
#include <GDebugEngine.h>
#include <iostream>
#include <TaskMediator.h>
#include "PassRangeList.h"


namespace {
	CGeoPoint goalPoint(Param::Field::PITCH_LENGTH, 0);
	CGeoLine endLine(CGeoPoint(Param::Field::PITCH_LENGTH / 2, -Param::Field::PITCH_WIDTH / 2), -1.57);
	CGeoLine myEndLine(CGeoPoint(-Param::Field::PITCH_LENGTH / 2, -Param::Field::PITCH_WIDTH / 2), -1.57);
	CGeoLine leftSideLine(CGeoPoint(0, -Param::Field::PITCH_WIDTH / 2 + 20), 0);
	CGeoLine rightSideLine(CGeoPoint(0, Param::Field::PITCH_WIDTH / 2 - 20), 0);
}

CGoSupport::CGoSupport() {

}

CGoSupport::~CGoSupport() {

}

void CGoSupport::plan(const CVisionModule* pVision)
{
	/********************************************************************************************/
	/*  ���ֽ�������ָ����ǰ��״̬			  Modified by JLC                                   */
	/********************************************************************************************/

	int robotNum = task().executor;
	bool leftOrRight = task().player.left_or_right; // 0:�Ҳ����� 1:�������
	const PlayerVisionT& me = pVision->OurPlayer(robotNum);
	const BallVisionT& ball = pVision->Ball();
	const CGeoPoint& ballPos = ball.Pos();
	const CVector& ballVel = ball.Vel();
	int intersectArea = 0;   // 0:δ�ཻ 1:����������ཻ 2;���Ҳ������ཻ
	bool waitingBall = 0;   // �Ƿ������ٵ��ӳ����ϵ���
	bool roleChange = 0;
	bool hasSelfClosePlayer = 0;	//��ǰ�����п��Ժ��Լ���ϵĳ����������


	CGeoPoint taskPoint(0,0);
	float taskDir = 0;

	if (leftOrRight) {
		roleChange = ((_lastLeftPlayerPos - me.Pos()).mod() > 20);
		_lastLeftPlayerPos = me.Pos();
	}
	else {
		roleChange = ((_lastRightPlayerPos - me.Pos()).mod() > 20);
		_lastRightPlayerPos = me.Pos();
	}
	
	//����hasSelfClosePlayer
	for (int i = 0; i < Param::Field::MAX_PLAYER; ++i) {
		if (pVision->OurPlayer(i).Valid()) {
			CGeoPoint selfPlayerPos = pVision->OurPlayer(i).Pos();
			CGeoLine ballRuningLine(ballPos, ballVel.dir());	// ���ٵ��ӳ���
			CGeoPoint proPoint = ballRuningLine.projection(selfPlayerPos);	// ��ǰ��Ա���������ߵĶԳƵ�
			double dist = proPoint.dist(selfPlayerPos);
			if (dist < 30 && abs(pVision->OurPlayer(i).Dir() - ballVel.dir()) > Param::Math::PI / 6 ){	
				hasSelfClosePlayer = 1;
				break;
			}
		}
	}

	TaskT myTask(task());
	myTask.player.angle = 0.0;
	myTask.player.pos = CGeoPoint(0, 0);

	if (ballVel.mod() > 50) {
		intersectArea = isectSeg(pVision);
	}

	bool ballRunningMe = (intersectArea == 1 && leftOrRight == 1) || (intersectArea == 2 && leftOrRight == 0);
	//����ָ����Ա����ʱ��ballRunningMe = 1
	waitingBall = (ballVel.mod() > 50) && ballRunningMe;
	
	/********************************************************************************************/
	/*  �ƶ�task��ִ�н������񣬴˴�����SupportPos2022		Modified by JLC                         */
	/********************************************************************************************/

	//�䵱֧Ԯ��
	if (!waitingBall) {
		if (leftOrRight) {
			taskPoint = SupportPos2022::Instance()->getSupportPos(pVision).getLeftSupportPos();
			double dir1 = (goalPoint - taskPoint).dir();
			double dir2 = (ballPos - taskPoint).dir();
			taskDir = (dir1 + dir2) / 2;
			if (taskDir * dir1 < 0) {
				taskDir += 3.14;
			}
		}
		else {
			taskPoint = SupportPos2022::Instance()->getSupportPos(pVision).getRightSupportPos();
			double dir1 = (goalPoint - taskPoint).dir();
			double dir2 = (ballPos - taskPoint).dir();
			taskDir = (dir1 + dir2) / 2;
			if (taskDir * dir1 < 0) {
				taskDir += 3.14;
			}
		}
	}

	else {
		if (hasSelfClosePlayer) {	
			// ������֧Ԯ��
			CVector actualVector(-100, 0);
			taskPoint = ballPos + actualVector;
			taskDir = (goalPoint - me.Pos()).dir();
			if (!leftOrRight) {
				taskDir += Param::Math::PI;
			}
		}
		else if (leftOrRight) {	//�������
			if (intersectArea == 1) {
				double middleDir = ballVel.dir();
				CGeoLine ballRuningLine(ballPos, middleDir);
				double maxDist = calcMaxDist(pVision, middleDir, ballRuningLine);
				CVector actualVector(0.7 * maxDist * cos(middleDir), 0.7 * maxDist * sin(middleDir));
				taskPoint = ballPos + actualVector;
				taskDir = middleDir + Param::Math::PI;
			}
			else {
				taskPoint = SupportPos2022::Instance()->getSupportPos(pVision).getLeftSupportPos();
				double dir1 = (goalPoint - taskPoint).dir();
				double dir2 = (ballPos - taskPoint).dir();
				taskDir = (dir1 + dir2) / 2;
				if (taskDir * dir1 < 0) {
					taskDir += 3.14;
				}
			}
		}
		else {	//�Ҳ�����
			if (intersectArea == 2) {
				double middleDir = ballVel.dir();
				CGeoLine ballRuningLine(ballPos, middleDir);
				double maxDist = calcMaxDist(pVision, middleDir, ballRuningLine);
				CVector actualVector(0.7 * maxDist * cos(middleDir), 0.7 * maxDist * sin(middleDir));
				taskPoint = ballPos + actualVector;
				taskDir = middleDir + Param::Math::PI;
			}
			else {
				taskPoint = SupportPos2022::Instance()->getSupportPos(pVision).getRightSupportPos();
				double dir1 = (goalPoint - taskPoint).dir();
				double dir2 = (ballPos - taskPoint).dir();
				taskDir = (dir1 + dir2) / 2;

				if (taskDir * dir1 < 0) {
					taskDir += 3.14;
				}
			}
		}
	}

	if (waitingBall) {
		GDebugEngine::Instance()->gui_debug_msg(taskPoint, "WAITING BALL");
	}
	else {
		GDebugEngine::Instance()->gui_debug_msg(taskPoint, "NOT WAITING BALL");
	}

	// ע�⣬���ɵ�λ�������﷢���˸ı�
	// ����������ﴦ����ô����������λ�÷����ľ��ұ仯����Ŀ��λ�ù��ڲ�����
	// ������һЩ�����Ŀ��λ����advanceλ�ù��ڽӽ�����Ҫ�������⴦��
	// ���ѡ��20CM�Ĳ�������ʹ�䵽��Ŀ���
	float stepDist = 50;
	CVector targetVector = taskPoint - me.Pos();
	if (targetVector.mod() > stepDist) {
		taskPoint = me.Pos() + targetVector.unit() * stepDist;
	}

	myTask.player.pos = taskPoint;
	myTask.player.angle = taskDir;

	setSubTask(TaskFactoryV2::Instance()->SmartGotoPosition(myTask));
	CPlayerTask::plan(pVision);
}


CPlayerCommand* CGoSupport::execute(const CVisionModule* pVision)
{
	if (subTask()) {
		return subTask()->execute(pVision);
	}
	return NULL;
}

/********************************************************************************************/
/*  ����������򣬷���ֵ����intersectArea  Modified by JLC                                    */ 
/********************************************************************************************/
int CGoSupport::isectSeg(const CVisionModule* pVision) {
	// 0:δ�ཻ 1:����������ཻ 2;���Ҳ������ཻ
	const BallVisionT& ball = pVision->Ball();
	const CGeoPoint& ballPos = ball.Pos();
	float ballPosX = ballPos.x();
	float ballPosY = ballPos.y();
	const CVector& ballVel = ball.Vel();
	float ballVelX = ballVel.x();
	float ballVelY = ballVel.y();

	// �ж��Ƿ��п���ֱ�ӽ������������ŷ�ȥ���򷵻� 0
	// �������������ļн�
	float leftBoundaryDir = (CGeoPoint(Param::Field::PITCH_LENGTH / 2, -Param::Field::PENALTY_AREA_WIDTH / 2) - ballPos).dir();
	float rightBoundaryDir = (CGeoPoint(Param::Field::PITCH_LENGTH / 2, Param::Field::PENALTY_AREA_WIDTH / 2) - ballPos).dir();
	if (ballVel.dir() < rightBoundaryDir && ballVel.dir() > leftBoundaryDir) {
		return 0;
	}

	// �ж�����������Ƿ��ཻ
	if (ballPosX > 0 && ballPosY < -Param::Field::PENALTY_AREA_WIDTH / 2) {	
		// ���ڸ����������
		return 1;
	}
	else { // �������������
		if (abs(ballVelX) < 1e-6 && ballVelY < 0 && ballPosX > 0) { 
			// ���������
			return 1;
		}
		else if (abs(ballVelY) < 1e-6 && ballVelX > 0 && ballPosY < -Param::Field::PENALTY_AREA_WIDTH / 2) {	
			// ��·��ǰ��
			return 1;
		}
		else {
			float t1 = (-Param::Field::PENALTY_AREA_WIDTH / 2 - ballPosY) / ballVelY;
			float t2 = (-Param::Field::PITCH_WIDTH / 2 - ballPosY) / ballVelY;

			float t3 = -ballPosX / ballVelX;
			float t4 = (Param::Field::PITCH_LENGTH / 2 - ballPosX) / ballVelX;

			if (t1 > t2) { float tmp = t2; t2 = t1; t1 = tmp; }
			if (t3 > t4) { float tmp = t4; t4 = t3; t3 = tmp; }
			
			float tmp = max(t1, t3);
			float tmin = max(tmp, 0.0f);
			float tmax = min(t2, t4);

			if (tmin < tmax) {	
				//�����ȱ߽��ʱ�����ȸ���
				return 1;
			}
		}
	}
	// �ж����Ҳ������Ƿ��ཻ
	if (ballPosX > 0 && ballPosY > Param::Field::PENALTY_AREA_WIDTH / 2) { 
		// �����Ҳ�������
		return 2;
	}
	else { 
		// �����Ҳ�������
		if (abs(ballVelX) < 1e-6 && ballVelY > 0 && ballPosX > 0) {
			return 2;
		}
		else if (abs(ballVelY) < 1e-6 && ballVelX > 0 && ballPosY > Param::Field::PENALTY_AREA_WIDTH / 2) {
			return 2;
		}
		else {
			float t1 = (Param::Field::PENALTY_AREA_WIDTH / 2 - ballPosY) / ballVelY;
			float t2 = (Param::Field::PITCH_WIDTH / 2 - ballPosY) / ballVelY;

			float t3 = -ballPosX / ballVelX;
			float t4 = (Param::Field::PITCH_LENGTH / 2 - ballPosX) / ballVelX;

			if (t1 > t2) { float tmp = t2; t2 = t1; t1 = tmp; }
			if (t3 > t4) { float tmp = t4; t4 = t3; t3 = tmp; }

			float tmp = max(t1, t3);
			float tmin = max(tmp, 0.0f);
			float tmax = min(t2, t4);

			if (tmin < tmax) {	
				//���ҿ�ȱ߽��ʱ�����ȸ���
				return 2;
			}
		}
	}
	return 0;
}

/********************************************************************************************/
/*  ����������򣬷���ֵ����intersectArea  Modified by JLC                                    */
/********************************************************************************************/
double CGoSupport::calcMaxDist(const CVisionModule* pVision, double middleDir, CGeoLine angleBisector) {
	CGeoPoint ballPos = (pVision->Ball()).Pos();
	double maxDist = 0;
	if (middleDir > 0 && middleDir < 1.57) {
		CGeoLineLineIntersection cross1(angleBisector, rightSideLine);
		CGeoPoint point1 = cross1.IntersectPoint();
		CGeoLineLineIntersection cross2(angleBisector, endLine);
		CGeoPoint point2 = cross2.IntersectPoint();
		maxDist = min(ballPos.dist(point1), ballPos.dist(point2));
	}
	else if (middleDir >= 1.57) {
		CGeoLineLineIntersection cross1(angleBisector, rightSideLine);
		CGeoPoint point1 = cross1.IntersectPoint();
		CGeoLineLineIntersection cross2(angleBisector, myEndLine);
		CGeoPoint point2 = cross2.IntersectPoint();
		maxDist = min(ballPos.dist(point1), ballPos.dist(point2));
	}
	else if (middleDir <= 0 && middleDir > -1.57) {
		CGeoLineLineIntersection cross1(angleBisector, leftSideLine);
		CGeoPoint point1 = cross1.IntersectPoint();
		CGeoLineLineIntersection cross2(angleBisector, endLine);
		CGeoPoint point2 = cross2.IntersectPoint();
		maxDist = min(ballPos.dist(point1), ballPos.dist(point2));
	}
	else {
		CGeoLineLineIntersection cross1(angleBisector, leftSideLine);
		CGeoPoint point1 = cross1.IntersectPoint();
		CGeoLineLineIntersection cross2(angleBisector, myEndLine);
		CGeoPoint point2 = cross2.IntersectPoint();
		maxDist = min(ballPos.dist(point1), ballPos.dist(point2));
	}
	return maxDist;
}

