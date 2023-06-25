#include "ballReceiverAttributeSet.h"
#include "defence/DefenceInfo.h"

namespace {
	CGeoPoint goalCentre(-Param::Field::PITCH_LENGTH / 2, 0);
}

//attribute define
//�㷨������ֲ��GPU֧Ԯ���
DEFINE_ATTRIBUTE_NEW(TooFar4Pass);//̫Զ����Ƚ���
DEFINE_ATTRIBUTE_NEW(TooClose4Pass);//̫������û����
DEFINE_ATTRIBUTE_NEW(TooFar4Shoot);//̫Զ���űȽ���
DEFINE_ATTRIBUTE_NEW(EasyBlock);//ֻ����ͶӰ���ҷ��Ƿ�����Ա�ܹ�����
//DEFINE_ATTRIBUTE_NEW(AffectShoot);//ǰ��Ӱ��ԭ�������ţ��о��������ڷ�����ϵ
DEFINE_ATTRIBUTE_NEW(NearSideLine);//�������ߣ��������ѣ����׳���
DEFINE_ATTRIBUTE_NEW(EasyShoot);//ֻ����ͶӰ���ҷ��Ƿ�����Ա�ܹ��赲����

//attribute evaluate
EVALUATE_ATTRIBUTE_NEW(TooFar4Pass)
{
	double dist = pVision->TheirPlayer(num).Pos().dist(pVision->Ball().Pos());
	setValue(max(0.0, dist - 0.5 * Param::Field::PITCH_LENGTH));
}

EVALUATE_ATTRIBUTE_NEW(TooClose4Pass)
{
	double dist = pVision->TheirPlayer(num).Pos().dist(pVision->Ball().Pos());
	if (dist < 0.1 * Param::Field::PITCH_LENGTH)
		setValue(0.1 * Param::Field::PITCH_LENGTH - dist + 0.08 * Param::Field::PITCH_LENGTH);
	else
		setValue(0);
}

EVALUATE_ATTRIBUTE_NEW(TooFar4Shoot)
{
	double dist = pVision->TheirPlayer(num).Pos().dist(goalCentre);
	setValue(max(0.0, dist - 500));//������в��Ҫȡ���ھ��Ծ���
}

EVALUATE_ATTRIBUTE_NEW(EasyBlock)
{
	double tempValue = 0;
	const BallVisionT& ball = pVision->Ball();
	const PlayerVisionT& enemy = pVision->TheirPlayer(num);
	CGeoLine passLine(ball.Pos(), enemy.Pos());
	for (int i = 0; i < Param::Field::MAX_PLAYER; i++)
	{
		const PlayerVisionT& blocker = pVision->OurPlayer(i);
		if (!blocker.Valid())
			continue;
		if (DefenceInfo::Instance()->queryMarked(num) && i == DefenceInfo::Instance()->getOurMarkDenfender(num))
			continue;
		CGeoPoint blockerProj = passLine.projection(blocker.Pos());
		double blocker2projDist = blocker.Pos().dist(blockerProj);
		double ball2projDist = ball.Pos().dist(blockerProj);
		// ���봫���߽�Զ�������Ͻ��������ǶԴ����ߵ�Ӱ��
		if (blocker2projDist > 300 || ball2projDist < 30)
			continue;
		double ratio_dist = blocker2projDist / ball2projDist;
		//�����ҷ���Ա�ٶȽ������ٵ�0.6����ñ�ֵ����0.6ʱ���ҷ���Ա�Դ�����Ӱ��
		if (Utils::InBetween(blockerProj, ball.Pos(), enemy.Pos()))
			tempValue += max(0.0, 0.6 - ratio_dist);
	}
	setValue(tempValue);
}

EVALUATE_ATTRIBUTE_NEW(NearSideLine)
{
	const PlayerVisionT& enemy = pVision->TheirPlayer(num);
	setValue(std::max(0.0, fabs(enemy.Y()) - Param::Field::PITCH_WIDTH / 2 + 50));
}

EVALUATE_ATTRIBUTE_NEW(EasyShoot)
{
	double tempValue = 0;
	const BallVisionT& ball = pVision->Ball();
	const PlayerVisionT& enemy = pVision->TheirPlayer(num);
	CGeoLine shootLine(goalCentre, enemy.Pos());
	for (int i = 0; i < Param::Field::MAX_PLAYER; i++)
	{
		const PlayerVisionT& blocker = pVision->OurPlayer(i);
		if (!blocker.Valid() || i == 0)//0:Ŀǰ�ҷ�Goalie
			continue;
		if (DefenceInfo::Instance()->queryMarked(num) && i == DefenceInfo::Instance()->getOurMarkDenfender(num))
			continue;
		CGeoPoint blockerProj = shootLine.projection(blocker.Pos());
		double blocker2projDist = blocker.Pos().dist(blockerProj);
		double enemy2projDist = enemy.Pos().dist(blockerProj);
		// �ҷ����������߽�Զ�����Բ����Ƕ������ߵ�Ӱ��
		if (blocker2projDist > 300)//|| enemy2projDist < 10)
			continue;
		// ����Ͻ�ʱ������������в����ô��
		if (blocker2projDist < 20)
			tempValue += 1;
		else
		{
			double ratio_dist = blocker2projDist / enemy2projDist;
			//����ʱ���ٽϴ��������㷨ʹ�������е���м��㣬ʵ�����Ż��и���ѡ����˶����ŵ��赲���С
			if (Utils::InBetween(blockerProj, goalCentre, enemy.Pos()))
				tempValue += max(0.0, 0.3 - ratio_dist);
		}
	}
	setValue(tempValue);
}

void CballReceiverAttributeSet::setAttributes()
{
	ADD_ATTRIBUTE_NEW(TooFar4Pass);
	ADD_ATTRIBUTE_NEW(TooClose4Pass);
	ADD_ATTRIBUTE_NEW(TooFar4Shoot);
	ADD_ATTRIBUTE_NEW(EasyBlock);
	ADD_ATTRIBUTE_NEW(NearSideLine);
	ADD_ATTRIBUTE_NEW(EasyShoot);
}
