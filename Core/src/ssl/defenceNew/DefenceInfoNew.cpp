#include <string>
#include "DefenceInfoNew.h"
#include "TaskMediator.h"
#include "GDebugEngine.h"

namespace{
}

CDefenceInfoNew::CDefenceInfoNew()
{
	for (int i = 0; i < Param::Field::MAX_PLAYER; i++)
	{
		_potientialList.emplace_back(0, 0);
	}
	_ballChaserList.push_back(0);
	_ballChaserSteadyList.push_back(0);
}

CDefenceInfoNew::~CDefenceInfoNew()
{
}

void CDefenceInfoNew::updateDefenceInfoNew(const CVisionModule* pVision)
{
	updateBallChaserList(pVision);
	updateBallReceiverList(pVision);
}

void CDefenceInfoNew::updateBallChaserList(const CVisionModule* pVision)
{
	_lastPotientialList = _potientialList;
	_potientialList.clear();
	//��Ȩ����׷��Ǳ����ԽСԽ��������
	for(int num=0;num<Param::Field::MAX_PLAYER;num++)
		_potientialList.emplace_back(num,ballChaserAttributeSet::Instance()->evaluate(pVision, num));

	//for test��
	/*int cnt = 0;
	for (auto potientialPair : _potientialList)
		if (fabs(potientialPair.second-1)<1e-8)
			cnt += 1;
	GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0,-100), std::to_string(cnt).c_str());*/
	//������ֱ�Ӹ��Ǽ�����Ϊ����
	double OurLooseBallCtrlDist = Param::Vehicle::V2::PLAYER_SIZE + Param::Field::BALL_SIZE + 3.5;
	double OurLooseBallCtrlAngle = Param::Math::PI * 15 / 180;
	double OurStrictBallCtrlDist = Param::Vehicle::V2::PLAYER_SIZE + Param::Field::BALL_SIZE + 1.5;
	double OurStrictBallCtrlAngle = Param::Math::PI * 10 / 180;
	for (int i = 0; i < Param::Field::MAX_PLAYER; i++)
	{
		if (!pVision->TheirPlayer(i).Valid())
			continue;
		CVector player2Ball = pVision->Ball().Pos() - pVision->TheirPlayer(i).Pos();
		double distPlayer2Ball = player2Ball.mod();
		double dAnglePlayer2Ball = fabs(Utils::Normalize(player2Ball.dir() - pVision->TheirPlayer(i).Dir()));
		bool ballLooseCtrl = (distPlayer2Ball < OurLooseBallCtrlDist && dAnglePlayer2Ball < OurLooseBallCtrlAngle);
		bool ballStrictCtrl = (distPlayer2Ball < OurStrictBallCtrlDist && dAnglePlayer2Ball < OurStrictBallCtrlAngle);
		extern bool IS_SIMULATION;
		if (!IS_SIMULATION) {
			ballLooseCtrl = ballLooseCtrl || RobotSensor::Instance()->IsInfraredOn(i);
			ballStrictCtrl = ballStrictCtrl || RobotSensor::Instance()->IsInfraredOn(i);
		}
		if (ballLooseCtrl)
			_potientialList[i].second = 0.1;
		if (ballStrictCtrl)
			_potientialList[i].second = 0;
	}
	//�Ž���Ӧ̫���׳�Ϊ����
	_potientialList[pVision->TheirGoalie()].second *= 1.5;
	_potientialList[pVision->TheirGoalie()].second += 0.2;
	//����һ֡��Ȩ
	for (int i = 0; i < Param::Field::MAX_PLAYER; i++)
		_potientialList[i].second = _potientialList[i].second * 0.75 + _lastPotientialList[i].second * 0.25;
	//���ɼ���Ա�ƺ�
	for (int i = 0; i < Param::Field::MAX_PLAYER; i++)
		if (!pVision->TheirPlayer(i).Valid())
			_potientialList[i].second = 1;
	//����ֵ�ڵĳ�ΪBallChaser
	std::sort(_potientialList.begin(), _potientialList.end(),
		[](auto a, auto b) {return a.second < b.second; });
	_ballChaserList.clear();
	for (auto potientialPair : _potientialList)
		if (potientialPair.second < 1)
			_ballChaserList.push_back(potientialPair.first);
	if (_ballChaserList.empty())
		_ballChaserList.push_back(0);//û�������Ա�ı�ʾ����
	//�ض�����¸���Steady
	if (_ballChaserList[0]!=_ballChaserSteadyList[0] ||
		_ballChaserList.size()!=_ballChaserSteadyList.size())//todo ԭdefenceinfo����ƿ��ǵ��Ǿ�����Ա�Ƿ���ͬ���Լ������˽���nomark�����������
		_ballChaserSteadyList = _ballChaserList;
	//Debug���
	QString attackInformNew;
	attackInformNew.append(QString("attackInform:"));
	for (auto i:_ballChaserList)
		attackInformNew.append(QString(" %1").arg(i));
	QString attackSteadyInformNew;
	attackSteadyInformNew.append(QString("steady attack:"));
	for (auto i:_ballChaserSteadyList)
		attackSteadyInformNew.append(QString(" %1").arg(i));
	GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(100, -350), attackInformNew.toLatin1(), COLOR_YELLOW);
	GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(-400, -350), attackSteadyInformNew.toLatin1(), COLOR_YELLOW);
}

void CDefenceInfoNew::updateBallReceiverList(const CVisionModule* pVision)
{
}
