#include "SupportPos2022.h"
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
	//�˴�ѡ��6��֧Ԯ�㣬getAreaScore�����ã��˰汾δ�õ�
	CGeoPoint leftPointFirst(225, -200);
	CGeoPoint rightPointFirst(225, 200);
	CGeoPoint leftPointSecond(75, -200);
	CGeoPoint rightPointSecond(75, 200);
	CGeoPoint leftPointThird(375, -200);
	CGeoPoint rightPointThird(375, 200);
	int areaScoreArray[6] = { 3, 3, 2, 2, 1, 1 };
	CGeoPoint chosenPointArray[6] = { leftPointFirst, rightPointFirst, leftPointSecond, rightPointSecond, leftPointThird, rightPointThird };
}


CSupportPos2022::CSupportPos2022() {
	_lastCycle = 0;
}

CSupportPos2022::~CSupportPos2022() {

}

/********************************************************************************************/
/*  ����֧Ԯ�㣬ballPos���Դ��������Χ/�޷�����ʱ���� 	Modified by JLC                     */
/********************************************************************************************/
void CSupportPos2022::generateSupportPoint(const CVisionModule* pVision)
{ 
	const BallVisionT& ball = pVision->Ball();
	const CGeoPoint& ballPos = ball.Pos();
	const CVector& ballVel = ball.Vel();

	CPassRangeList passRangeList(pVision);
	const CValueRange* leftBestRange = NULL;   // ѡ������õĴ���Χ
	const CValueRange* rightBestRange = NULL;   // ѡ������õĴ���Χ
	const CValueRangeList& leftPassRange = passRangeList.getLeftPassRange();   // ���п��ܵĴ���Χ
	const CValueRangeList& rightPassRange = passRangeList.getRightPassRange();

	const double goalDir = (goalPoint - ballPos).dir();
	int leftRangeNum = leftPassRange.size();
	int rightRangeNum = rightPassRange.size();

	//int* scoreArray = new int[rangeNum];
	//double* distArray = new double[rangeNum];
	//int cnt = 0;
	CGeoPoint leftTaskPoint(0, 0); // �������ɵ������
	CGeoPoint rightTaskPoint(0, 0); // �������ɵ������

	//std::cout << "==================================================" << std::endl;
	//std::cout << "leftRangeNum: " << leftRangeNum << std::endl;
	//std::cout << "rightRangeNum: " << rightRangeNum << std::endl;
	if (true) {
		if (ballPos.y() > Param::Field::PENALTY_AREA_WIDTH / 2) { // �򿿽��ұ���
			// ����Ŀ���
			leftBestRange = leftPassRange.getMaxRange();
			if (leftBestRange) {
				double middleDir = leftBestRange->getMiddle();
				CGeoLine angleBisector(ballPos, middleDir);
				double leftMaxDist = 0;
				leftMaxDist = calcMaxDist(pVision, middleDir, angleBisector);
				//std::cout << "right side -- left point" << std::endl;

				leftTaskPoint = generateBestPoint(pVision, leftBestRange->getMax(), leftBestRange->getMin(), leftMaxDist);
			}
			else {
				leftTaskPoint.fill(150, -200);
			}
			// ���������������͵��ߵľ�����Ϊ�ж�����
			if (ballPos.x() > 250) { // ��ǳ��������ߣ��޷�����ȥ��ĺ�
				rightTaskPoint.fill(ballPos.x() - 150, ballPos.y());
			}
			else {
				rightBestRange = rightPassRange.getMaxRange();
				if (rightBestRange) {
					double middleDir = rightBestRange->getMiddle();
					CGeoLine angleBisector(ballPos, middleDir);
					double rightMaxDist = calcMaxDist(pVision, middleDir, angleBisector);
					//std::cout << "right side -- left point" << std::endl;
					rightTaskPoint = generateBestPoint(pVision, rightBestRange->getMax(), rightBestRange->getMin(), rightMaxDist);
				}
				else {
					rightTaskPoint.fill(150, 200);
				}
			}
		}
		else if (ballPos.y() < -Param::Field::PENALTY_AREA_WIDTH / 2) { // �򿿽������
			// ����Ŀ���
			rightBestRange = rightPassRange.getMaxRange();
			if (rightBestRange) {
				double middleDir = rightBestRange->getMiddle();
				CGeoLine angleBisector(ballPos, middleDir);
				double rightMaxDist = calcMaxDist(pVision, middleDir, angleBisector);
				rightTaskPoint = generateBestPoint(pVision, rightBestRange->getMax(), rightBestRange->getMin(), rightMaxDist);
			}
			else {
				rightTaskPoint.fill(150, 200);
			}
			// ����Ŀ��㣬���������������͵��ߵľ�����Ϊ�ж�����
			if (ballPos.x() > 250) { // ��ǳ��������ߣ��޷�����ȥ��ĺ�
				leftTaskPoint.fill(ballPos.x() - 150, ballPos.y());
			}
			else {
				//std::cout << "left side -- left point" << std::endl;
				leftBestRange = leftPassRange.getMaxRange();
				if (leftBestRange) {
					double middleDir = leftBestRange->getMiddle();
					CGeoLine angleBisector(ballPos, middleDir);
					double leftMaxDist = calcMaxDist(pVision, middleDir, angleBisector);;
					leftTaskPoint = generateBestPoint(pVision, leftBestRange->getMax(), leftBestRange->getMin(), leftMaxDist);
				}
				else {
					leftTaskPoint.fill(150, -200);
				}
			}
		}
		else { // �����м�λ��
			// �����Ҳ�Ŀ���
			rightBestRange = rightPassRange.getMaxRange();
			if (rightBestRange) {
				double rightMiddleDir = rightBestRange->getMiddle();
				CGeoLine rightAngleBisector(ballPos, rightMiddleDir);
				double rightMaxDist = calcMaxDist(pVision, rightMiddleDir, rightAngleBisector);
				rightTaskPoint = generateBestPoint(pVision, rightBestRange->getMax(), rightBestRange->getMin(), rightMaxDist);
			}
			else {
				rightTaskPoint.fill(150, 200);
			}
			// �������Ŀ���
			leftBestRange = leftPassRange.getMaxRange();
			if (leftBestRange) {
				double leftMiddleDir = leftBestRange->getMiddle();
				CGeoLine leftAngleBisector(ballPos, leftMiddleDir);
				double leftMaxDist = calcMaxDist(pVision, leftMiddleDir, leftAngleBisector);
				leftTaskPoint = generateBestPoint(pVision, leftBestRange->getMax(), leftBestRange->getMin(), leftMaxDist);
			}
			else {
				leftTaskPoint.fill(150, -200);
			}

		}
	}
	_supportPosStruct2022.leftSupportPos = leftTaskPoint;
	_supportPosStruct2022.rightSupportPos = rightTaskPoint;
	//std::cout << "_supportPosStruct2022.leftSupportPos.x: " << _supportPosStruct2022.leftSupportPos.x() << std::endl;
}

SupportPosStruct2022 CSupportPos2022::getSupportPos(const CVisionModule* pVision) {
	if (pVision->Cycle() == _lastCycle) {
		return _supportPosStruct2022;
	}
	else {
		_lastCycle = pVision->Cycle();
	}
	generateSupportPoint(pVision);
	return _supportPosStruct2022;
}

/********************************************************************************************/
/*  ��ǰ�汾�в�δ�õ�   getAreaScore                                                         */
/*  �Ľ�ѡ��˼·��ս��ѡ�񣬿�ǰ���߿���֧Ԯ������Ӧ�Բ�ͬ���� 	Modified by JLC                 */
/********************************************************************************************/
int  CSupportPos2022::getAreaScore(const CVisionModule* pVision, double v_max, double v_min) {
	CGeoPoint ballPos = (pVision->Ball()).Pos();
	for (int i = 0; i < 6; i++) {
		CGeoPoint chosenPoint = chosenPointArray[i];
		double dir = (chosenPoint - ballPos).dir();
		//���ս���areaScoreֵ���������Ȩ�أ��Ӵ�С�������Ƕ�
		if (dir > v_min && dir < v_max) {
			return areaScoreArray[i];
		}
	}
	return 0;
}

/********************************************************************************************/
/*  ��ǰ�汾��ѡ��˼·��ѡ������λ���򣬼���Distʱ���ݵг�ѹ�ȳ̶�ѡ�����                      */
/*  �Ľ�ѡDist˼·����ѡ��ǰ���߿���֧Ԯ������Ӧ�Բ�ͬ���� 	Modified by JLC                     */
/********************************************************************************************/
CGeoPoint CSupportPos2022::generateBestPoint(const CVisionModule* pVision, double v_max, double v_min, double maxDist) {
	CGeoPoint ballPos = (pVision->Ball()).Pos();
	double middleDir = (v_max + v_min) / 2;
	// maxDist = calcMaxDist(pVision, left/rightMiddleDir, left/rightAngleBisector) �����߾���;
	double targetDist = 0.8 * maxDist;
	double dist = targetDist;
	CVector targetVector(targetDist * cos(middleDir), targetDist * sin(middleDir));
	CGeoPoint targetPoint = ballPos + targetVector;
	for (int i = 0; i < Param::Field::MAX_PLAYER; ++i) {
		if (pVision->TheirPlayer(i).Valid()) {
			const CGeoPoint& enemyPos = pVision->TheirPlayer(i).Pos();
			const CVector target2enemy = enemyPos - targetPoint;
			const double target2enemyfDist = target2enemy.mod();
			if (target2enemyfDist < 40) {
				if (targetDist > 150) {
					dist = max(80.0, targetDist - 40);
				}
				else {
					dist = min(dist + 40, maxDist);
				}
			}
		}
	}
	CVector actualVector(dist * cos(middleDir), dist * sin(middleDir));
	CGeoPoint actualPos = ballPos + actualVector;
	actualPos.fill(min(Param::Field::PITCH_LENGTH / 2 - 30, actualPos.x()), min(max(-Param::Field::PITCH_WIDTH / 2 + 20, actualPos.y()), Param::Field::PITCH_WIDTH / 2 - 20));
	return actualPos;
}

/********************************************************************************************/
/*  �����������ǰ���˶�״̬�¾�����ߵľ���MaxDist 	Modified by JLC                         */
/********************************************************************************************/
double CSupportPos2022::calcMaxDist(const CVisionModule* pVision, double middleDir, CGeoLine angleBisector) {
	//middleDir��angleBisector����ballPos-MiddleDir
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


