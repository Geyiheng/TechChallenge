#ifndef _VISION_MODULE_H_
#define _VISION_MODULE_H_
#include <param.h>
#include <WorldModel/WorldDefine.h>
#include <PlayerCommand.h>
#include "BallPredictor.h"
#include "RobotPredictor.h"
#include <game_state.h>
#include <OptionModule.h>
#include "utils.h"
#include <vector>
using namespace std;

/// @file   VisionModule.h
/// @author Yonghai Wu <liunian@zju.edu.cn>
/// @date   Tue Oct 13 14:26:36 2009
/// 
/// @brief  �Ӿ�Ԥ���������˲���Ԥ��ȵȡ�
///         
///     ���ļ���Ҫ��ά����������ģ�ͣ�����Ҫ�����Ӿ���Ϣ���������в���
///  ����Ϣ������λ��ͨ��˫��ͨѶ�õ��Ļ����˵��ת�ټ�������״̬��Ϣ��
///  ���У�
/// 
///   -  �Ӿ���ԭʼ��Ϣ���� 10 �������˺����λ����Ϣ���Ӿ���Ϣ�Ĵ�����
/// Ҫ����:
///       -# �ݴ���˲���������ԭʼ���ݰ���������������ʱ������Ϣ��ȫ
///    �ʹ�������������������Ҫ�Ƚ����˲����ݴ���������õ�
///    kalman �˲��㷨��
///       -# Ԥ�⣺���������Ӳ����ͼ�����ʱ��ԭ���Ӿ���Ϣ���� 100
///    ms ���ҵ���ʱ�����Ա������Ԥ����ܵõ������˺������ʵλ�á���
///    ���ܹ����Թ������������Ԥ��ģ�ͺ�����Ԥ��ģ�͡�
///       -# ��ײģ�ͣ����򱻻����˵�סʱ�����û�����ײģ�͵��㷨�����
///    λ�ý��й��ơ�
/// 
///  - �Բ��к���Ϣ�Ĵ���������Ҫ������һ��Ƚ��Ѻõİ�װ�����¶�����
/// ��ȡ�Ľӿڡ�
/// 
///  - ����λ����ȡ����Ϣ�����У�
///      - ����Թ��Ƿ��⵽��
///      - ����ָ���Ƿ�ִ��\n
///   �����״̬��������У�����򱻻����˵�סʱ����ײģ�ͣ����ŵ�ִ����
///   �������ж����Ŷ����Ƿ�����ִ�У���Ի�����״̬����ת�ǳ���Ҫ��
/************************************************************************/
/*                       VisionModule                                   */
/************************************************************************/
class CVisionModule{
public:
	CVisionModule();
	~CVisionModule(void);
	void registerOption(const COptionModule* pOption);
	const int& getValidNum() const { return _validNum; };
	// �Է����Ͽ�����Ա�� [6/28/2011 zhanfei]
	const int& getTheirValidNum() const { return _TheirValidNum; };
	////////////////////////////////////////////////////////////////////////////////////////////////////
	/// @fn	void SetNewVision(const VisualInfoT& vInfo)
	///
	/// @brief �Ӿ�������ѭ���������˲���Ԥ���. 
	///
	/// @author	Yonghai Wu
	/// @date	2009-10-13
	///
	/// @param	vInfo	Information describing the v. 
	////////////////////////////////////////////////////////////////////////////////////////////////////
	void SetRefRecvMsg(const GameInfoT);
	void SetNewVision(const GameInfoT& vInfo);
	////////////////////////////////////////////////////////////////////////////////////////////////////
	/// @fn	const PlayerVisionT& AllPlayer(int num) const
	///
	/// @brief �õ����л����˵��Ӿ���Ϣ. 
	///
	/// @author	Yonghai Wu
	/// @date	2009-10-13
	///
	/// @param	num - �����˺��룬1-5 Ϊ�ҷ��� 6-10 Ϊ����. 
	///
	/// @return	. 
	////////////////////////////////////////////////////////////////////////////////////////////////////
	const PlayerVisionT& AllPlayer(int num) const { return (num < Param::Field::MAX_PLAYER) ? OurPlayer(num) : TheirPlayer(num - Param::Field::MAX_PLAYER); }
	////////////////////////////////////////////////////////////////////////////////////////////////////
	/// @fn	const PlayerVisionT& OurPlayer(int num) const
	///
	/// @brief	ȡ�ҷ��������Ӿ���Ϣ. 
	///
	/// @author	Yonghai Wu
	/// @date	2009-10-13
	///
	/// @maintainer Logan Shi
	/// @date   2020-7-24
	/// @param	num	- �����˺��룬 0-10��ֱ�Ӷ�Ӧ�����˲���
	///
	/// @return	. 
	////////////////////////////////////////////////////////////////////////////////////////////////////
	const PlayerVisionT& OurPlayer(int num) const {
		if (Utils::PlayerNumValid(num)) {
			return _ourPlayerPredictor[num].getResult(_timeCycle);
		} else {
		//	std::cout<<"Player num:" << num << " [ ####### ] Get our player info Invalid !!!"<<std::endl;
			return _ourPlayerPredictor[0].getResult(_timeCycle);
		}
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////
	/// @fn	const PlayerVisionT& TheirPlayer(int num) const
	///
	/// @brief	ȡ���ֻ�������Ϣ. 
	///
	/// @author	Yonghai Wu
	/// @date	2009-10-13
	///
	/// @maintainer Logan Shi
	/// @date   2020-7-24
	/// @param	num	- �����˺��룬 0-10��ֱ�Ӷ�Ӧ�����˲���
	///
	/// @return	. 
	////////////////////////////////////////////////////////////////////////////////////////////////////
	const PlayerVisionT& TheirPlayer(int num) const {
		if (Utils::PlayerNumValid(num)) {
			return _theirPlayerPredictor[num].getResult(_timeCycle);
		} else {
		//	std::cout<<"Player num:" << num << " [ ####### ] Get their player info Invalid !!!"<<std::endl;
			return _theirPlayerPredictor[0].getResult(_timeCycle);
		}
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////
	/// @fn	const BallVisionT& Ball() const
	///
	/// @brief	�õ�����Ӿ���Ϣ. 
	///
	/// @author	Yonghai Wu
	/// @date	2009-10-13
	///
	/// @return	. 
	////////////////////////////////////////////////////////////////////////////////////////////////////
	const BallVisionT& Ball() const { return _ballPredictor.getResult(_timeCycle); }
	// �����������Ķ�Ա�����λ����Ϣ

	////////////////////////////////////////////////////////////////////////////////////////////////////
	/// @fn	const PlayerVisionT& OurPlayer(int cycle, int num) const
	///
	/// @brief	ȡ�ض����ڵĻ������Ӿ���Ϣ. 
	///
	/// @author	Yonghai Wu
	/// @date	2009-10-13
	///
	/// @param	cycle	������. 
	/// @param	num		�����˺��� 1-5. 
	///
	/// @return	. 
	////////////////////////////////////////////////////////////////////////////////////////////////////
	const PlayerVisionT& OurPlayer(int cycle, int num) const { return _ourPlayerPredictor[num].getResult(cycle); }

	////////////////////////////////////////////////////////////////////////////////////////////////////
	/// @fn	const PlayerVisionT& TheirPlayer(int cycle, int num) const
	///
	/// @brief	ȡ�ض����ڶԷ��������Ӿ���Ϣ. 
	///
	/// @author	Yonghai Wu
	/// @date	2009-10-13
	///
	/// @param	cycle	The cycle. 
	/// @param	num		�����˺��� 1-5. 
	///
	/// @return	. 
	////////////////////////////////////////////////////////////////////////////////////////////////////
	const PlayerVisionT& TheirPlayer(int cycle, int num) const { return _theirPlayerPredictor[num].getResult(cycle); }

	////////////////////////////////////////////////////////////////////////////////////////////////////
	/// @fn	const BallVisionT& Ball(int cycle) const
	///
	/// @brief	ȡ�ض������������Ϣ. 
	///
	/// @author	Yonghai Wu
	/// @date	2009-10-13
	///
	/// @param	cycle	The cycle. 
	///
	/// @return	. 
	////////////////////////////////////////////////////////////////////////////////////////////////////
	const BallVisionT& Ball(int cycle) const { return _ballPredictor.getResult(cycle);	}
	////////////////////////////////////////////////////////////////////////////////////////////////////
	/// @fn	void SetPlayerCommand(int num, const CPlayerCommand* pCmd)
	///
	/// @brief	��¼�������ҷ�������ִ�е�ָ����浽��ʷ��ջ� 
	///
	/// @author	Yonghai Wu
	/// @date	2009-10-13
	///
	/// @param	num		Number of. 
	/// @param	pCmd	If non-null, the command. 
	////////////////////////////////////////////////////////////////////////////////////////////////////
	void SetPlayerCommand(int num, const CPlayerCommand* pCmd);
	////////////////////////////////////////////////////////////////////////////////////////////////////
	/// @fn	const GameState& gameState() const
	///
	/// @brief	�õ����к���Ϣ. 
	///
	/// @author	Yonghai Wu
	/// @date	2009-10-13
	///
	/// @return	. 
	////////////////////////////////////////////////////////////////////////////////////////////////////
	const GameState& gameState() const { return _gameState; } ///< ����״̬
	////////////////////////////////////////////////////////////////////////////////////////////////////
	/// @fn	int Cycle() const
	///
	/// @brief	�õ���ǰ��������. 
	///
	/// @author	Yonghai Wu
	/// @date	2009-10-13
	///
	/// @return	. 
	////////////////////////////////////////////////////////////////////////////////////////////////////
	int Cycle() const { return _timeCycle; }
	////////////////////////////////////////////////////////////////////////////////////////////////////
	/// @fn	int LastCycle() const
	///
	/// @brief	�õ���һ�����ڵ�������. 
	///
	/// @author	Yonghai Wu
	/// @date	2009-10-13
	///
	/// @return	. 
	////////////////////////////////////////////////////////////////////////////////////////////////////
	int LastCycle() const { return _lastTimeCycle; }

	////////////////////////////////////////////////////////////////////////////////////////////////////
	/// @fn	int Side() const
	///
	/// @brief	�õ��ҷ�ѡ�еı�. 
	///
	/// @author	Yonghai Wu
	/// @date	2009-10-13
	///
	/// @return	. 
	////////////////////////////////////////////////////////////////////////////////////////////////////
	int Side() const { return _pOption->MySide(); }

	const string GetCurrentRefereeMsg()const;
	const string GetLastRefereeMsg()const;
	int  OurGoal() const { return _ourGoal; }
	int  TheirGoal() const { return _theirGoal; }
	int TheirGoalie() const { return _theirGoalie; }

	const CVector OurRawPlayerSpeed(int num) const {return _ourPlayerPredictor[num].getResult(_timeCycle).Vel(); }
	const CVector TheirRawPlayerSpeed(int num) const {return _theirPlayerPredictor[num].getResult(_timeCycle).Vel();}

	const int GetTheirPenaltyNum() const {return _theirPenaltyNum;}
	void ResetTheirPenaltyNum() { _theirPenaltyNum = 0;}
	int GetTheirGoalieStrategyNum() const {return _theirGoalieStrategyNum; }

	bool getBallVelStable() const {return !_ballVelDirChanged;}
	const CGeoPoint& getBallPlacementPosition() const {
		return _ballPlacementPosition;
	}

	bool ballVelValid();
protected:
	////////////////////////////////////////////////////////////////////////////////////////////////////
	/// @fn	void SetCycle(int cycle)
	///
	/// @brief	���õ�ǰ������. 
	///
	/// @author	Yonghai Wu
	/// @date	2009-10-13
	///
	/// @param	cycle	The cycle. 
	////////////////////////////////////////////////////////////////////////////////////////////////////
	void SetCycle(int cycle){ _lastTimeCycle = _timeCycle; _timeCycle = cycle; } 
	////////////////////////////////////////////////////////////////////////////////////////////////////
	/// @fn	void CheckBothSidePlayerNum()
	///
	/// @brief	���˫����Ա�ڳ�����. 
	///
	/// @author	Yonghai Wu
	/// @date	2009-10-13
	////////////////////////////////////////////////////////////////////////////////////////////////////
	void CheckBothSidePlayerNum();
	////////////////////////////////////////////////////////////////////////////////////////////////////
	/// @fn	void CheckKickoffStatus(const VisualInfoT& info)
	///
	/// @brief	������״̬. 
	///
	/// @author	Yonghai Wu
	/// @date	2009-10-13
	///
	/// @param	info	The information. 
	////////////////////////////////////////////////////////////////////////////////////////////////////
	void CheckKickoffStatus(const GameInfoT& info); // ��鿪��״̬

	void UpdateRefereeMsg();

	void judgeBallVelStable();
private:
	/// ����ģʽ�����Լ�������ǰ״̬
	const COptionModule* _pOption;   			///< ������ز���������볡�����Ұ볡. 
	static const int MAX_SAVE_CYCLES = 64;		///< ��������������. 
	GameState _gameState;						///< ����״̬�����к���Ϣ.
    GameState _next_gameState;					///< ��������һ״̬�����к���Ϣ.

	/// ������ص����ڣ�ʱ�䣩
	int _timeCycle;								///< ��ǰ������.
	int _lastTimeCycle;							///< ��һ������.
	
	///	�����˲���
	CBallPredictor _ballPredictor;										///< ��Ԥ��. 
	CRobotPredictor _ourPlayerPredictor[Param::Field::MAX_PLAYER];		///< �ҷ�������Ԥ��. 
	CRobotPredictor _theirPlayerPredictor[Param::Field::MAX_PLAYER];	///< �Է�������Ԥ��. 

	/// ˫�����϶�Աͳ��
	int _validNum;							///< �õ�������Ա���ҷ�����Ա��
	int _TheirValidNum;						///< �õ��Է�����Ա��

	bool _ballKicked;						///< �ж����Ƿ���,������������״̬.
	CGeoPoint _ballPosSinceNotKicked;		///< ����û�б���֮ǰ��λ��.
	TranslationT _rawBallPos;				///< ����������ԭʼ���ݣ�����draw�Ƚ�.
	TranslationT _lastRawBallPos;			///< ��֡��ĵ�ԭʼ����
	// ��������
	bool _hasCollision;						///< ��������ײ,��ʱ�Ĵ�������Щ(����һ��������).
	string _refereeMsg;
	string _lastRefereeMsg;
	int _lastContactNum;
	int _ourGoal;
	int _theirGoal;
	int _ourGoalie;
	int _theirGoalie;
	int _theirGoalieStrategyNum;
	int _theirPenaltyNum; // �Է��ڼ�������

	int _ballVelChangeCouter;
	bool _ballVelDirChanged;
	CGeoPoint _ballPlacementPosition;

};

typedef Falcon::NormalSingleton<CVisionModule> VisionModule;
#define VISION_MODULE VisionModule::Instance()

#endif // _VISION_MODULE_H_
