#include <Vision/VisionModule.h>
#include <WorldModel/RobotCapability.h>
#include <TaskMediator.h>
#include "PassRangeList.h"
#include "param.h"

namespace {
	
}

/********************************************************************************************/
/*  ���ص�ǰballPos�����Է����赲��RangeList����ɢ�б� 	Modified by JLC                     */
/********************************************************************************************/
CPassRangeList::CPassRangeList(const CVisionModule* pVision)
{
	if (lastCycle != pVision->Cycle()) { // ���¼���
		lastCycle = pVision->Cycle();

		lastLeftPassList.clear(); // ���
		lastRightPassList.clear(); // ���
		
		CGeoPoint startPos = pVision->Ball().Pos();
		CGeoPoint leftLeftPost, leftRightPost, rightLeftPost, rightRightPost;
		if (startPos.x() < -200) {
			leftLeftPost.fill(0, -Param::Field::PITCH_WIDTH / 2);
			leftRightPost.fill(Param::Field::PITCH_LENGTH / 2, -Param::Field::PENALTY_AREA_WIDTH / 2);

			rightLeftPost.fill(Param::Field::PITCH_LENGTH / 2, Param::Field::PENALTY_AREA_WIDTH / 2);
			rightRightPost.fill(0, Param::Field::PITCH_WIDTH / 2);
		}
		else {
			if (startPos.y() > Param::Field::PENALTY_AREA_WIDTH / 2) {
				leftLeftPost.fill(0, -Param::Field::PITCH_WIDTH / 2);
				leftRightPost.fill(Param::Field::PITCH_LENGTH / 2, -Param::Field::PENALTY_AREA_WIDTH / 2);

				rightLeftPost.fill(Param::Field::PITCH_LENGTH / 2, Param::Field::PENALTY_AREA_WIDTH / 2);
				rightRightPost.fill(Param::Field::PITCH_LENGTH / 2, Param::Field::PITCH_WIDTH / 2);
			}
			else if (startPos.y() < -Param::Field::PENALTY_AREA_WIDTH / 2) {
				leftLeftPost.fill(Param::Field::PITCH_LENGTH / 2, -Param::Field::PITCH_WIDTH / 2);
				leftRightPost.fill(Param::Field::PITCH_LENGTH / 2, -Param::Field::PENALTY_AREA_WIDTH / 2);

				rightLeftPost.fill(Param::Field::PITCH_LENGTH / 2, Param::Field::PENALTY_AREA_WIDTH / 2);
				rightRightPost.fill(0, Param::Field::PITCH_WIDTH / 2);
			}
			else {
				leftLeftPost.fill(Param::Field::PITCH_LENGTH / 2, -Param::Field::PITCH_WIDTH / 2);
				leftRightPost.fill(Param::Field::PITCH_LENGTH / 2, -Param::Field::PENALTY_AREA_WIDTH / 2);

				rightLeftPost.fill(Param::Field::PITCH_LENGTH / 2, Param::Field::PENALTY_AREA_WIDTH / 2);
				rightRightPost.fill(Param::Field::PITCH_LENGTH / 2, Param::Field::PITCH_WIDTH / 2);
			}
		}

		const double leftLeftPostAngle = (leftLeftPost - startPos).dir();
		const double leftRightPostAngle = (leftRightPost - startPos).dir();
		lastLeftPassList.add(CValueRange(leftLeftPostAngle, leftRightPostAngle)); // ��ʼ��
		const double rightLeftPostAngle = (rightLeftPost - startPos).dir();
		const double rightRightPostAngle = (rightRightPost - startPos).dir();
		lastRightPassList.add(CValueRange(rightLeftPostAngle, rightRightPostAngle)); // ��ʼ��

		//��������ѡ����
		for (int i = 0; i < Param::Field::MAX_PLAYER; ++i) {
			if (pVision->TheirPlayer(i).Valid() && pVision->TheirPlayer(i).X() > startPos.x()) {
				const CGeoPoint& playerPos = pVision->TheirPlayer(i).Pos();
				const CVector startPos2player = playerPos - startPos;
				const double playerDist = startPos2player.mod() - Param::Field::MAX_PLAYER_SIZE / 2;
				const double playerDir = startPos2player.dir();
				const double angleRange = (playerDist <= Param::Field::MAX_PLAYER_SIZE / 2 ? Param::Math::PI / 2 : std::asin(Param::Field::MAX_PLAYER_SIZE / 2 / playerDist)); // ��ס�ĽǶ�
				if (playerDist < 15) {
					continue; // �з�Χס��ʱ�������ǵз�������赲
				}
				lastLeftPassList.removeAngleRange(playerDir, angleRange, playerDist);
				lastRightPassList.removeAngleRange(playerDir, angleRange, playerDist);
			}
		}

		//if (startPos.x() < 250 && abs(startPos.y()) < 180) {
		//	// ֻ�������Լ�ǰ���Ĵ���λ��
		//	const CGeoPoint leftPost(startPos.x(), -Param::Field::PITCH_WIDTH / 2);
		//	const CGeoPoint rightPost(startPos.x(), Param::Field::PITCH_WIDTH / 2);
		//	const double leftPostAngle = (leftPost - startPos).dir();
		//	const double rightPostAngle = (rightPost - startPos).dir();
		//	const double leftPostDist = (leftPost - startPos).mod();
		//	const double rightPostDist = (rightPost - startPos).mod();

		//	// ������ͨ�������ķ�Χ
		//	const CGeoPoint leftRemovePost(Param::Field::PITCH_LENGTH / 2 - Param::Field::PENALTY_AREA_DEPTH, -Param::Field::PENALTY_AREA_WIDTH / 2);
		//	const CGeoPoint rightRemovePost(Param::Field::PITCH_LENGTH / 2 - Param::Field::PENALTY_AREA_DEPTH, Param::Field::PENALTY_AREA_WIDTH / 2);
		//	const double leftRemovePostAngle = (leftRemovePost - startPos).dir();
		//	const double rightRemovePostAngle = (rightRemovePost - startPos).dir();
		//	const double leftRemovePostDist = (leftRemovePost - startPos).mod();
		//	const double rightRemovePostDist = (rightRemovePost - startPos).mod();


		//	lastPassList.add(CValueRange(leftPostAngle, rightPostAngle, leftPostDist, rightPostDist)); // ��ʼ��
		//	lastPassList.remove(CValueRange(leftRemovePostAngle, rightRemovePostAngle)); // ȥ�����������Ĳ���
		//	lastPassList.remove(CValueRange(1.04, 1.6));
		//	lastPassList.remove(CValueRange(-1.6, -1.04));
		//	for (int i = 0; i < Param::Field::MAX_PLAYER; ++i) {
		//		if (pVision->TheirPlayer(i).Valid() && pVision->TheirPlayer(i).X() > startPos.x()) {
		//			const CGeoPoint& playerPos = pVision->TheirPlayer(i).Pos();
		//			const CVector startPos2player = playerPos - startPos;
		//			const double playerDist = startPos2player.mod() - Param::Field::MAX_PLAYER_SIZE / 2;
		//			const double playerDir = startPos2player.dir();
		//			const double angleRange = (playerDist <= Param::Field::MAX_PLAYER_SIZE / 2 ? Param::Math::PI / 2 : std::asin(Param::Field::MAX_PLAYER_SIZE / 2 / playerDist)); // ��ס�ĽǶ�
		//			if (playerDist < 15) {
		//				continue; // �з�Χס��ʱ�������ǵз�������赲
		//			}
		//			lastPassList.removeAngleRange(playerDir, angleRange, playerDist);
		//		}
		//	}
		//}
		//else {
		//	if (startPos.y() > 0){
		//		// ��ʼ����Χ
		//		const CGeoPoint leftPost(0, -Param::Field::PITCH_WIDTH / 2);
		//		const CGeoPoint rightPost(Param::Field::PITCH_LENGTH / 2, Param::Field::PENALTY_AREA_WIDTH / 2);
		//		const double leftPostAngle = (leftPost - startPos).dir();
		//		const double rightPostAngle = (rightPost - startPos).dir();
		//		const double leftPostDist = (leftPost - startPos).mod();
		//		const double rightPostDist = (rightPost - startPos).mod();

		//		lastPassList.add(CValueRange(leftPostAngle, rightPostAngle, leftPostDist, rightPostDist)); // ��ʼ��
		//		for (int i = 0; i < Param::Field::MAX_PLAYER; ++i) {
		//			if (pVision->TheirPlayer(i).Valid() && pVision->TheirPlayer(i).X() > 0) {
		//				const CGeoPoint& playerPos = pVision->TheirPlayer(i).Pos();
		//				const CVector startPos2player = playerPos - startPos;
		//				const double playerDist = startPos2player.mod() - Param::Field::MAX_PLAYER_SIZE / 2;
		//				const double playerDir = startPos2player.dir();
		//				const double angleRange = (playerDist <= Param::Field::MAX_PLAYER_SIZE / 2 ? Param::Math::PI / 2 : std::asin(Param::Field::MAX_PLAYER_SIZE / 2 / playerDist)); // ��ס�ĽǶ�
		//				if (playerDist < 20) {
		//					continue; // �з�Χס��ʱ�������ǵз�������赲
		//				}
		//				lastPassList.removeAngleRange(playerDir, angleRange, playerDist);
		//			}
		//		}
		//	}
		//	else {
		//		// ��ʼ����Χ
		//		const CGeoPoint leftPost(0, Param::Field::PITCH_WIDTH / 2);
		//		const CGeoPoint rightPost(Param::Field::PITCH_LENGTH / 2, -Param::Field::PENALTY_AREA_WIDTH / 2);
		//		const double leftPostAngle = (leftPost - startPos).dir();
		//		const double rightPostAngle = (rightPost - startPos).dir();
		//		const double leftPostDist = (leftPost - startPos).mod();
		//		const double rightPostDist = (rightPost - startPos).mod();

		//		lastPassList.add(CValueRange(leftPostAngle, rightPostAngle, leftPostDist, rightPostDist)); // ��ʼ��
		//		for (int i = 0; i < Param::Field::MAX_PLAYER; ++i) {
		//			if (pVision->TheirPlayer(i).Valid() && pVision->TheirPlayer(i).X() > 0) {
		//				const CGeoPoint& playerPos = pVision->TheirPlayer(i).Pos();
		//				const CVector startPos2player = playerPos - startPos;
		//				const double playerDist = startPos2player.mod() - Param::Field::MAX_PLAYER_SIZE / 2;
		//				const double playerDir = startPos2player.dir();
		//				const double angleRange = (playerDist <= Param::Field::MAX_PLAYER_SIZE / 2 ? Param::Math::PI / 2 : std::asin(Param::Field::MAX_PLAYER_SIZE / 2 / playerDist)); // ��ס�ĽǶ�
		//				if (playerDist < 20) {
		//					continue; // �з�Χס��ʱ�������ǵз�������赲
		//				}
		//				lastPassList.removeAngleRange(playerDir, angleRange, playerDist);
		//			}
		//		}
		//	}
		//}
	}
}

const CValueRangeList& CPassRangeList::getLeftPassRange()
{
	return lastLeftPassList;
}

const CValueRangeList& CPassRangeList::getRightPassRange()
{
	return lastRightPassList;
}