/************************************************************************/
/* Copyright (c) CSC-RL, Zhejiang University							*/
/* Team:			SSL-ZJUNlict										*/
/* HomePage: http://www.nlict.zju.edu.cn/ssl/WelcomePage.html			*/
/************************************************************************/
/* File:	ActionModule.h												*/
/* Brief:	C++ Implementation: Action	execution						*/
/* Func:	Provide an action command send interface					*/
/* Author:	cliffyin, 2012, 08											*/
/* Refer:	NONE														*/
/* E-mail:	cliffyin007@gmail.com										*/
/************************************************************************/	

#include "ActionModule.h"
#include <WorldModel/KickStatus.h>
#include <WorldModel/DribbleStatus.h>
#include <TaskMediator.h>
#include <PlayerCommandV2.h>
#include <CommandFactory.h>
//#include <PathPlan/PathPlanner.h>
#include <BallStatus.h>
#include "Semaphore.h"
extern Semaphore decision_to_action;

CActionModule::CActionModule(const COptionModule* pOption,CVisionModule* pVision,const CDecisionModule* pDecision)
: _pOption(pOption),_pVision(pVision),_pDecision(pDecision)
{
    isYellow = (pOption->MyColor() == TEAM_YELLOW);
    cmds_socket = new QUdpSocket();
}

CActionModule::~CActionModule(void)
{
    delete cmds_socket;
    cmds_socket = nullptr;
}

// 用于当场上有的车号大于5的情况
bool CActionModule::sendAction(const GameInfoT& vInfo)
{
    decision_to_action.Wait();
	rbk::protocol::SRC_Cmd cmds;

	/************************************************************************/
	/* 第一步：遍历小车，执行赋予的任务，生成动作指令                       */
	/************************************************************************/
	for (int vecNum = 0; vecNum < Param::Field::MAX_PLAYER; ++ vecNum) {
		rbk::protocol::Message_SSL_Command* ssl_cmd = nullptr;
		// 获取当前小车任务
		CPlayerTask* pTask = TaskMediator::Instance()->getPlayerTask(vecNum);
		// 没有任务，跳过
		if (NULL == pTask) {
			continue;
		}

		// 执行skill，任务层层调用执行，得到最终的指令：<vx vy w> + <kick dribble>
		// 执行的结果：命令接口（仿真-DCom，实物-CommandSender） + 指令记录（运动-Vision，踢控-PlayInterface)
		bool dribble = false;
		CPlayerCommand* pCmd = NULL;
		pCmd = pTask->execute(_pVision); 

		if (!pCmd) {
			std::cout<<"PlayerCommand execute is Null "<<vecNum<<std::endl;
		}
		// 跑：有效的运动指令
		if (pCmd) {
			dribble = pCmd->dribble() > 0;
			// 下发运动 <vx vy w>
			if(ssl_cmd == nullptr){
				ssl_cmd = cmds.add_command();
				//LogInfo("action module number in lua: " << pCmd->number());
				//LogInfo("action module number in real: " << vInfo.player[pCmd->number()].ID);
				ssl_cmd->set_robot_id(vInfo.player[pCmd->number()].ID);
			}
			// for rbk
			//pCmd->execute(IS_SIMULATION, robotIndex[vecNum-1]);
			((CPlayerSpeedV2*)pCmd)->setSpeedtoSSLCmd(ssl_cmd);
			// 记录指令
			_pVision->SetPlayerCommand(pCmd->number(), pCmd);
		}

		// 踢：有效的踢控指令
		double kickPower = 0.0;
		double chipkickDist = 0.0;
		double passdist = 0.0;
		if (KickStatus::Instance()->needKick(vecNum) && pCmd) {
			// 更新踢相关参数
			kickPower = KickStatus::Instance()->getKickPower(vecNum);
			chipkickDist = KickStatus::Instance()->getChipKickDist(vecNum);
			passdist = KickStatus::Instance()->getPassDist(vecNum);
			// 涉及到平/挑射分档，这里只关系相关参数，实际分档请关注 CommandSender
			CPlayerKickV2 kickCmd(vecNum, kickPower, chipkickDist, passdist, dribble);
			// 机构动作 <kick dribble>
			//kickCmd.execute(IS_SIMULATION);
			if(ssl_cmd == nullptr){
				ssl_cmd = cmds.add_command();
				ssl_cmd->set_robot_id(vInfo.player[pCmd->number()].ID);
			}
			kickCmd.setKicktoSSLCmd(ssl_cmd);
		}

		// 记录命令
		//std::cout << "kickPower: " << kickPower << std::endl;
		BallStatus::Instance()->setCommand(vecNum, kickPower, chipkickDist, dribble, _pVision->Cycle());
	}

	sendToOwl(cmds);

	/************************************************************************/
	/* 第二步：指令清空处理                                                 */
	/************************************************************************/
	// 清除上一周期的射门指令
	KickStatus::Instance()->clearAll();
	// 清除上一周期的控球指令
	DribbleStatus::Instance()->clearDribbleCommand();
	//// 清除上一周期的障碍物标记
	//CPathPlanner::resetObstacleMask();

	return true;
}

bool CActionModule::sendNoAction(const GameInfoT& vInfo)
{
	rbk::protocol::SRC_Cmd cmds;
	//TODO: make num compatible!!!!!!!!2020-11-05!!!!!!!!!!!!!!!!!!!!
	for (int vecNum = 0; vecNum < Param::Field::MAX_PLAYER - 10; ++ vecNum) {
		// 生成停止命令
		CPlayerCommand *pCmd = CmdFactory::Instance()->newCommand(CPlayerSpeedV2(vecNum,0,0,0,0));
		// 执行且下发
		//pCmd->execute(IS_SIMULATION);
		rbk::protocol::Message_SSL_Command* ssl_cmd = cmds.add_command();
		ssl_cmd->set_robot_id(vInfo.player[pCmd->number()].ID);
		((CPlayerSpeedV2*)pCmd)->setSpeedtoSSLCmd(ssl_cmd);
		// 记录指令
		_pVision->SetPlayerCommand(pCmd->number(), pCmd);
	}
    sendToOwl(cmds);
	return true;
}

void CActionModule::sendToOwl(const rbk::protocol::SRC_Cmd& cmds)
{
	int totalnum = cmds.command_size();
	if (totalnum >= 24) {
        qDebug()<<"too many commands!!!";
        return;
	}
    else if (totalnum <= 0) {
        qDebug()<<"nocmd_";
    }
	ZSS_CMDS.set_login_name(OParamManager::Instance()->LoginName);
    //qDebug()<<"ready to send!!!";
    for (int i = 0; i < cmds.command_size(); i++) {
        auto zss_cmd = ZSS_CMDS.add_command();
        auto src_cmd = cmds.command(i);
		zss_cmd->set_robot_id(src_cmd.robot_id());
		zss_cmd->set_velocity_x(src_cmd.velocity_x()); // m/s
		zss_cmd->set_velocity_y(src_cmd.velocity_y());
		zss_cmd->set_velocity_r(src_cmd.velocity_r());
        zss_cmd->set_dribbler_spin(src_cmd.dribbler_spin());
        if (src_cmd.flat_kick() > 0.01) {
            zss_cmd->set_kick(false);
            zss_cmd->set_power(src_cmd.flat_kick());
        }
        else if (src_cmd.chip_kick() > 0.01) {
            zss_cmd->set_kick(true);
            zss_cmd->set_power(src_cmd.chip_kick());
        }
        else {
            zss_cmd->set_kick(false);
            zss_cmd->set_power(0);
        }
	}
    //发送cmd//
	int port = isYellow ? CParamManager::Instance()->yellow_control : CParamManager::Instance()->blue_control;
    int size = ZSS_CMDS.ByteSize();
    QByteArray data(size, 0);
    ZSS_CMDS.SerializeToArray(data.data(), size);
    cmds_socket->writeDatagram(data.data(), size, QHostAddress(CParamManager::Instance()->local_address), port);
    ZSS_CMDS.Clear();
}
