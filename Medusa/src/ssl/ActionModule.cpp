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
#include "staticparams.h"
#include <CommandFactory.h>
//#include <PathPlan/PathPlanner.h>
#include <BallStatus.h>

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

// ���ڵ������еĳ��Ŵ���5�����
bool CActionModule::sendAction(const GameInfoT& vInfo)
{
	rbk::protocol::SRC_Cmd cmds;

	/************************************************************************/
	/* ��һ��������С����ִ�и�����������ɶ���ָ��                       */
	/************************************************************************/
	for (int vecNum = 0; vecNum < Param::Field::MAX_PLAYER; ++ vecNum) {
		rbk::protocol::Message_SSL_Command* ssl_cmd = nullptr;
		// ��ȡ��ǰС������
		CPlayerTask* pTask = TaskMediator::Instance()->getPlayerTask(vecNum);
		// û����������
		if (NULL == pTask) {
			continue;
		}

		// ִ��skill�����������ִ�У��õ����յ�ָ�<vx vy w> + <kick dribble>
		// ִ�еĽ��������ӿڣ�����-DCom��ʵ��-CommandSender�� + ָ���¼���˶�-Vision���߿�-PlayInterface)
		bool dribble = false;
		CPlayerCommand* pCmd = NULL;
		pCmd = pTask->execute(_pVision); 

		if (!pCmd) {
			std::cout<<"PlayerCommand execute is Null "<<vecNum<<std::endl;
		}
		// �ܣ���Ч���˶�ָ��
		if (pCmd) {
			dribble = pCmd->dribble() > 0;
			// �·��˶� <vx vy w>
			if(ssl_cmd == nullptr){
				ssl_cmd = cmds.add_command();
				//LogInfo("action module number in lua: " << pCmd->number());
				//LogInfo("action module number in real: " << vInfo.player[pCmd->number()].ID);
				ssl_cmd->set_robot_id(vInfo.player[pCmd->number()].ID);
			}
			// for rbk
			//pCmd->execute(IS_SIMULATION, robotIndex[vecNum-1]);
			((CPlayerSpeedV2*)pCmd)->setSpeedtoSSLCmd(ssl_cmd);
			// ��¼ָ��
			_pVision->SetPlayerCommand(pCmd->number(), pCmd);
		}

		// �ߣ���Ч���߿�ָ��
		double kickPower = 0.0;
		double chipkickDist = 0.0;
		double passdist = 0.0;
		if (KickStatus::Instance()->needKick(vecNum) && pCmd) {
			// ��������ز���
			kickPower = KickStatus::Instance()->getKickPower(vecNum);
			chipkickDist = KickStatus::Instance()->getChipKickDist(vecNum);
			passdist = KickStatus::Instance()->getPassDist(vecNum);
			// �漰��ƽ/����ֵ�������ֻ��ϵ��ز�����ʵ�ʷֵ����ע CommandSender
			CPlayerKickV2 kickCmd(vecNum, kickPower, chipkickDist, passdist, dribble);
			// �������� <kick dribble>
			//kickCmd.execute(IS_SIMULATION);
			if(ssl_cmd == nullptr){
				ssl_cmd = cmds.add_command();
				ssl_cmd->set_robot_id(vInfo.player[pCmd->number()].ID);
			}
			kickCmd.setKicktoSSLCmd(ssl_cmd);
		}

		// ��¼����
		//std::cout << "kickPower: " << kickPower << std::endl;
		BallStatus::Instance()->setCommand(vecNum, kickPower, chipkickDist, dribble, _pVision->Cycle());
	}

    sendToOwl(cmds);

	/************************************************************************/
	/* �ڶ�����ָ����մ���                                                 */
	/************************************************************************/
	// �����һ���ڵ�����ָ��
	KickStatus::Instance()->clearAll();
	// �����һ���ڵĿ���ָ��
	DribbleStatus::Instance()->clearDribbleCommand();
	//// �����һ���ڵ��ϰ�����
	//CPathPlanner::resetObstacleMask();

	return true;
}

bool CActionModule::sendNoAction(const GameInfoT& vInfo)
{
	rbk::protocol::SRC_Cmd cmds;
	//TODO: make num compatible!!!!!!!!2020-11-05!!!!!!!!!!!!!!!!!!!!
	for (int vecNum = 0; vecNum < Param::Field::MAX_PLAYER - 10; ++ vecNum) {
		// ����ֹͣ����
		CPlayerCommand *pCmd = CmdFactory::Instance()->newCommand(CPlayerSpeedV2(vecNum,0,0,0,0));
		// ִ�����·�
		//pCmd->execute(IS_SIMULATION);
		rbk::protocol::Message_SSL_Command* ssl_cmd = cmds.add_command();
		ssl_cmd->set_robot_id(vInfo.player[pCmd->number()].ID);
		((CPlayerSpeedV2*)pCmd)->setSpeedtoSSLCmd(ssl_cmd);
		// ��¼ָ��
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
        qDebug()<<"no command!!!";
    }
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
    //����cmd//
    int port = ZSS::Athena::CONTROL_SEND[isYellow];
    int size = ZSS_CMDS.ByteSize();
    QByteArray data(size, 0);
    ZSS_CMDS.SerializeToArray(data.data(), size);
    cmds_socket->writeDatagram(data.data(), size, QHostAddress(ZSS::LOCAL_ADDRESS), port);
    ZSS_CMDS.Clear();
}
