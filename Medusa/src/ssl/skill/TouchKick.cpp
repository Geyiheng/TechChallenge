#include "TouchKick.h"
#include "skill/Factory.h"
#include "PointCalculation/TouchKickPos.h"
#include "GDebugEngine.h"
#include "BallSpeedModel.h"
#include "BallStatus.h"
#include "Global.h"

namespace{
	// ����Ԥ����ֵ
	const int TouchDist = 30;
	const double TouchAngle = Param::Math::PI / 36;
	const int MaxPreFrame = 75;
	double TOUCH_SHIFT_DIST = 0;
	bool TOUCH_Debug = false;
}

CTouchKick::CTouchKick() 
{
	_reach_pre_flag = false;
	_lastCycle = 0;
	//DECLARE_PARAM_READER_BEGIN(TOUCHKICK)
	//READ_PARAM(TOUCH_SHIFT_DIST)
	//READ_PARAM(TOUCH_Debug)
	//DECLARE_PARAM_READER_END
	TOUCH_SHIFT_DIST = paramManager->TOUCH_SHIFT_DIST;
	TOUCH_Debug = paramManager->TOUCH_Debug;
}

void CTouchKick::plan(const CVisionModule* pVision) 
{
	// ���������ͼ����Ϣ
	int runner = task().executor;
	bool runner_visible = pVision->OurPlayer(runner).Valid();
	const PlayerVisionT& me = pVision->OurPlayer(runner);
	const BallVisionT& ball = pVision->Ball();
	CGeoPoint myHead = me.Pos() + Utils::Polar2Vector(11, me.Dir());
	const CVector me2ball = ball.Pos() - me.Pos();
	const CVector ball2myHead = myHead - ball.Pos();
	const double antiBallVelDir = Utils::Normalize(ball.Vel().dir() + Param::Math::PI);
	double kick_dir;

	// ��ʼ��
	if ( pVision->Cycle() - _lastCycle > Param::Vision::FRAME_RATE * 0.1 
		|| BallStatus::Instance()->IsBallKickedOut(runner)){
		_reach_pre_flag = false;
		PreFrame = MaxPreFrame;
	}

	// ����ķ���		
	kick_dir = task().player.angle;
	// ����վλ�㣺��������������������
	// ��������ʱ��Ԥ��
	// ���ٷ����������������ֵ
	double diff_antiBallVelDir_kickDir = fabs(Utils::Normalize(antiBallVelDir - kick_dir));
	// ʱ��Ԥ��
	PreFrame = 30 + diff_antiBallVelDir_kickDir * 90 / Param::Math::PI;
    // ��λ��Ԥ��
	CGeoPoint prePos = BallSpeedModel::Instance()->posForTime(PreFrame, pVision);
	// Ԥ��λ�ó���+���ٽϴ����
	if(Utils::OutOfField(prePos,0) && ball.Vel().mod()>10){
		CGeoLine ballMovingLine = CGeoLine(ball.Pos(),ball.Pos()+ball.Vel());
		// Ԥ��λ��Ϊ��Ա�����˶�������ͶӰ��
		prePos = ballMovingLine.projection(me.Pos());
		if (TOUCH_Debug) {
			GDebugEngine::Instance()->gui_debug_x(prePos, COLOR_CYAN);
		}
	}

	// ��Touchʱ����һ������
	prePos = prePos + Utils::Polar2Vector( Param::Vehicle::V2::TOUCH_SHIFT_DIST,Utils::Normalize(kick_dir + Param::Math::PI));
	GDebugEngine::Instance()->gui_debug_x(prePos, COLOR_CYAN);
	double me2prePosDist = me.Pos().dist(prePos);
	double diffAngle = fabs(Utils::Normalize((ball2myHead.dir() - pVision->Ball().Vel().dir())));


	TaskT wait_kick_task(task());
	wait_kick_task.player.flag = wait_kick_task.player.flag | PlayerStatus::DODGE_OUR_DEFENSE_BOX;
	// �ܵ㵽λ
	if(!_reach_pre_flag){
		CGeoLine ballMovingLine = CGeoLine(ball.Pos(),ball.Pos()+ball.Vel());
		CGeoPoint tmpprojPos = ballMovingLine.projection(me.Pos());
		bool isPreBack = ball.Pos().dist(prePos) > ball.Pos().dist(tmpprojPos)+9;

		if(me2prePosDist < TouchDist || (diffAngle < TouchAngle && me.Pos().dist(ball.Pos())<TouchDist*2)
			|| (isPreBack && tmpprojPos.dist(me.Pos())<50 && ball.Vel().mod()>10)){
				_reach_pre_flag = true;
		}
		if(diffAngle < TouchAngle && ball2myHead.mod()<30){
			PreFrame = PreFrame - 0.1;
		} 
		wait_kick_task.player.pos = prePos;
		if (TOUCH_Debug) {
			GDebugEngine::Instance()->gui_debug_x(wait_kick_task.player.pos, COLOR_CYAN);
		}
	} 

	if(_reach_pre_flag || !ball.Valid()){
		TouchKickPos::Instance()->GenerateTouchKickPos(pVision,runner,kick_dir);
		wait_kick_task.player.pos = TouchKickPos::Instance()->getKickPos();
		if (TOUCH_Debug) {
			GDebugEngine::Instance()->gui_debug_x(wait_kick_task.player.pos, COLOR_CYAN);
		}
	}
	wait_kick_task.player.angle = kick_dir;
	wait_kick_task.player.vel = CVector(0.0,0.0);
	wait_kick_task.player.is_specify_ctrl_method = true;
	wait_kick_task.player.specified_ctrl_method = CMU_TRAJ;


	setSubTask(TaskFactoryV2::Instance()->GotoPosition(wait_kick_task));

    _lastCycle = pVision->Cycle();
	CStatedTask::plan(pVision);
}

CPlayerCommand* CTouchKick::execute(const CVisionModule* pVision)
{
	if (subTask()) {
		return subTask()->execute(pVision);
	}

	return NULL;
}