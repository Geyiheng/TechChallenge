#include <string>
#include "DefenceInfoNew.h"
#include "TaskMediator.h"
#include "GDebugEngine.h"
#include "defence/DefenceInfo.h"

namespace {
	bool display_debug_info;
}

#define debug_arr(arr,p) if(display_debug_info){\
	QString str_##arr;\
	str_##arr.append(QString(#arr));\
	for (auto i : arr)\
	str_##arr.append(QString(" %1").arg(i));\
	GDebugEngine::Instance()->gui_debug_msg(p, str_##arr.toLatin1(), COLOR_YELLOW);\
}

CDefenceInfoNew::CDefenceInfoNew() :isInTheirPass(false), _ballChaserChanged(false), _ballReceiverChanged(false), _markerAmount(0), _lastMarkerAmount(0)
{
	display_debug_info = paramManager->display_debug_info;
	_chaserPotientialList.resize(Param::Field::MAX_PLAYER, 0);
	_receiverPotientialList.resize(Param::Field::MAX_PLAYER, 0);
	_ballChaserList.reserve(Param::Field::MAX_PLAYER);
	_ballChaserList.push_back(0);
	_ballChaserSteadyList.reserve(Param::Field::MAX_PLAYER);
	_ballChaserSteadyList.push_back(0);
	_ballReceiverList.reserve(Param::Field::MAX_PLAYER);
	_ballReceiverList.push_back(0);
	_ballReceiverSteadyList.reserve(Param::Field::MAX_PLAYER);
	_ballReceiverSteadyList.push_back(0);
}

CDefenceInfoNew::~CDefenceInfoNew()
{
}

void CDefenceInfoNew::updateDefenceInfoNew(const CVisionModule* pVision)
{
	_lastMarkerAmount = _markerAmount;
	_markerAmount = 0;
	updateBallChaserList(pVision);
	updateBallReceiverList(pVision);
	checkPass(pVision);
	updateSteady();
	//debug���
	debug_arr(_ballChaserList, CGeoPoint(-Param::Field::PITCH_LENGTH / 4, Param::Field::PITCH_WIDTH * 0.6));
	debug_arr(_ballChaserSteadyList, CGeoPoint(Param::Field::PITCH_LENGTH / 4, Param::Field::PITCH_WIDTH * 0.6));
	debug_arr(_ballReceiverList, CGeoPoint(-Param::Field::PITCH_LENGTH / 4, Param::Field::PITCH_WIDTH * 0.7));
	debug_arr(_ballReceiverSteadyList, CGeoPoint(Param::Field::PITCH_LENGTH / 4, Param::Field::PITCH_WIDTH * 0.7));

}

void CDefenceInfoNew::updateBallChaserList(const CVisionModule* pVision)
{
	_lastChaserPotientialList = _chaserPotientialList;
	_chaserPotientialList.clear();
	//��Ȩ����׷��Ǳ����ԽСԽ��������
	for (int num = 0; num < Param::Field::MAX_PLAYER; num++)
		_chaserPotientialList.push_back(ballChaserAttributeSet::Instance()->evaluate(pVision, num));
	//������ֱ�Ӹ��Ǽ�����Ϊ����
	for (int i = 0; i < Param::Field::MAX_PLAYER; i++)
		if (BallStatus::Instance()->getBallPossession(false, i) > 0.5)
			_chaserPotientialList[i] = 0;
	//�Ž���Ӧ̫���׳�Ϊ����
	//_chaserPotientialList[pVision->TheirGoalie()] *= 1.5;
	//_chaserPotientialList[pVision->TheirGoalie()] += 0.2;
	//����һ֡��Ȩ
	for (int i = 0; i < Param::Field::MAX_PLAYER; i++)
		_chaserPotientialList[i] = _chaserPotientialList[i] * 0.75 + _lastChaserPotientialList[i] * 0.25;
	//���ɼ���Ա�ų�������ֵ�ڵĳ�ΪBallChaser����Ǳ������
	_ballChaserList.clear();
	for (int i = 0; i < Param::Field::MAX_PLAYER; i++)
		_ballChaserList.push_back(i);
	_ballChaserList.erase(remove_if(_ballChaserList.begin(), _ballChaserList.end(),
		[&](int num) {return (!pVision->TheirPlayer(num).Valid()) || _chaserPotientialList[num] >= 100; }),
		_ballChaserList.end());
	sort(_ballChaserList.begin(), _ballChaserList.end(),
		[&](int num1, int num2) {return _chaserPotientialList[num1] < _chaserPotientialList[num2]; });
}

void CDefenceInfoNew::updateBallReceiverList(const CVisionModule* pVision)
{
	_lastReceiverPotientialList = _receiverPotientialList;
	_receiverPotientialList.clear();
	//��Ȩ�������Ǳ����ԽСԽ������Ϊ�����߱�����
	for (int num = 0; num < Param::Field::MAX_PLAYER; num++)
		_receiverPotientialList.push_back(ballReceiverAttributeSet::Instance()->evaluate(pVision, num, false));
	//ballChaser������Receiver
	_receiverPotientialList[_ballChaserList[0]] = 10000;
	//�Ž���Ӧ̫���׳�Ϊ����
	//_receiverPotientialList[pVision->TheirGoalie()] *= 1.5;
	//����һ֡��Ȩ
	for (int i = 0; i < Param::Field::MAX_PLAYER; i++)
		_receiverPotientialList[i] = _receiverPotientialList[i] * 0.75 + _lastReceiverPotientialList[i] * 0.25;
	//���ɼ���Ա�ų�������ֵ�ڵĳ�ΪBallReceiver����Ǳ������
	_ballReceiverList.clear();
	for (int i = 0; i < Param::Field::MAX_PLAYER; i++)
		_ballReceiverList.push_back(i);
	_ballReceiverList.erase(remove_if(_ballReceiverList.begin(), _ballReceiverList.end(),
		[&](int num) {return (!pVision->TheirPlayer(num).Valid()) || _receiverPotientialList[num] >= 10000; }),
		_ballReceiverList.end());
	sort(_ballReceiverList.begin(), _ballReceiverList.end(),
		[&](int num1, int num2) {return _receiverPotientialList[num1] < _receiverPotientialList[num2]; });
}

//����ʱ�����⴦��
void CDefenceInfoNew::checkPass(const CVisionModule* pVision)
{
	const BallVisionT& ball = pVision->Ball();
	if (isInTheirPass)
	{
		//�ж��Ƿ����ڴ���״̬
		const PlayerVisionT& receiver = pVision->TheirPlayer(_receiver);
		double ball2ReceiverDir = (receiver.Pos() - ball.Pos()).dir();
		double angleDiff = fabs(Utils::Normalize(ball2ReceiverDir - ball.Vel().dir()));
		double ball2ReceiverDist = ball.Pos().dist(receiver.Pos());
		if (angleDiff > Param::Math::PI / 2.0 || ball2ReceiverDist > 450 || ball2ReceiverDist < 100 || ball.Vel().mod() < 70)
		{
			isInTheirPass = false;
			TaskMediator::Instance()->resetAdvancerPassTo();
		}
	} else
	{
		if (BallStatus::Instance()->IsBallKickedOut())
		{
			if (display_debug_info)
				GDebugEngine::Instance()->gui_debug_msg(ball.Pos(), "ball kick!");
			for (_kicker = 0; _kicker < 2 * Param::Field::MAX_PLAYER; _kicker++)
				if (BallStatus::Instance()->IsBallKickedOut(_kicker))
					break;
			if (Param::Field::MAX_PLAYER <= _kicker && _kicker < 2 * Param::Field::MAX_PLAYER)//TheirKick
			{
				_kicker -= Param::Field::MAX_PLAYER;
				matchReceiver(pVision);
				if (_receiver != -1)
				{
					isInTheirPass = true;
					//����ж�Ӧmarking����ʹ���Ϊ���ǵ�advance
					if (DefenceInfo::Instance()->queryMarked(_receiver))
						TaskMediator::Instance()->setAdvancerPassTo(pVision->OurPlayer(DefenceInfo::Instance()->getOurMarkDenfender(_receiver)).Pos());
				}
			}
		}
	}
	//��������Ϊ�µ�BallChaser��ͬʱ��BallReceiver�Ƴ�
	if (isInTheirPass)
	{
		int bestBallChaserNoPass = getBestBallChaser();
		if (display_debug_info)
			GDebugEngine::Instance()->gui_debug_msg(ball.Pos(), "new: in pass");
		auto temp = find(_ballChaserList.begin(), _ballChaserList.end(), _receiver);
		if (temp != _ballChaserList.end())
			rotate(_ballChaserList.begin(), temp, temp + 1);
		else
			_ballChaserList.insert(_ballChaserList.begin(), _receiver);
		auto tmp = find(_ballReceiverList.begin(), _ballReceiverList.end(), _receiver);
		if (tmp != _ballReceiverList.end())
			_ballReceiverList.erase(tmp);
		_ballReceiverList.push_back(bestBallChaserNoPass);
	}
}

void CDefenceInfoNew::matchReceiver(const CVisionModule* pVision)
{
	const BallVisionT& ball = pVision->Ball();
	for (int maybeReceiverNum : _ballReceiverList)
	{
		const PlayerVisionT& maybeReceiver = pVision->TheirPlayer(maybeReceiverNum);
		double ball2ReceiverDir = (maybeReceiver.Pos() - ball.Pos()).dir();
		double angleDiff = fabs(Utils::Normalize(ball2ReceiverDir - ball.Vel().dir()));
		double ball2ReceiverDist = ball.Pos().dist(maybeReceiver.Pos());
		if (angleDiff > Param::Math::PI / 2.0 || ball2ReceiverDist > 450 || ball2ReceiverDist < 100)
			continue;
		_receiver = maybeReceiverNum;
		return;
	}
	_receiver = -1;
}

void CDefenceInfoNew::updateSteady()
{
	_ballChaserSteadyList = _ballChaserList;
	_ballReceiverSteadyList = _ballReceiverList;

	//Ŀǰʹ�ýӿڵĵط����ж��Ƿ���ڣ�����Ҫ���ף����ѱ�֤steady��Ϊ�գ����������steady
	//�ܲ���ֱ�ӵ���steady���ɵģ�������
	if (_ballChaserList.empty())
		_ballChaserList = _ballChaserSteadyList;
	if (_ballReceiverList.empty())
		_ballReceiverList = _ballReceiverSteadyList;

	//chaser: ��Ҫ�����׷����
	_ballChaserChanged = false;
	if (_ballChaserList[0] != _ballChaserSteadyList[0])
		_ballChaserChanged = true;
	if (_ballChaserChanged)
		_ballChaserSteadyList = _ballChaserList;

	//receiver: ���marking�����������⴦��
	_ballReceiverChanged = false;
	if (_ballReceiverList[0] != _ballReceiverSteadyList[0])
		_ballReceiverChanged = true;
	if (_ballReceiverList.size() < _lastMarkerAmount || _ballReceiverSteadyList.size() < _lastMarkerAmount)
		_ballReceiverChanged = true;
	else
	{
		vector<double> toMarkPotientialList(_lastMarkerAmount);
		for (int i = 0; i < _lastMarkerAmount; i++)
			toMarkPotientialList[i] = _receiverPotientialList[_ballReceiverSteadyList[i]];
		sort(toMarkPotientialList.begin(), toMarkPotientialList.end());
		for (int i = 0; i < _lastMarkerAmount; i++)
			if (toMarkPotientialList[i] > 1.3 * _receiverPotientialList[_ballReceiverList[i]])
			{
				_ballReceiverChanged = true; //qDebug() << "big:" << toMarkPotientialList[i] << "small:" << _receiverPotientialList[_ballReceiverList[i]];
			}
	}
	if (_ballReceiverChanged)
		_ballReceiverSteadyList = _ballReceiverList;
}
