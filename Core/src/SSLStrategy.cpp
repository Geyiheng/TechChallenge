#include <iostream>
#include <WorldModel/WorldModel.h>
#include "DecisionModule.h"
#include "ActionModule.h"
#include <CtrlBreakHandler.h>
#include <GDebugEngine.h>
#include "bayes/MatchState.h"
#include "gpuBestAlgThread.h"
#include "Global.h"
#include "DefendUtils.h"
#include "Compensate.h"
#include "zss_debug.pb.h"
#include "referee.pb.h"
#include "src_cmd.pb.h"
#include "src_rawvision.pb.h"
#include "Vision/DataReceiver4rbk.h"
#include <thread>
#include <QCoreApplication>
#include <parammanager.h>

ZSS::Protocol::Debug_Msgs guiDebugMsgs;

//extern CEvent *visionEvent;
QMutex* _best_visiondata_copy_mutex = 0;
QMutex* _value_getter_mutex = 0;
QMutex* _debug_mutex = 0;
bool IS_SIMULATION = false;
bool VERBOSE_MODE = false;

// handle _vision_event;

void run(){
    ZSS::ZParamManager::instance()->loadParam(IS_SIMULATION, "Alert/IsSimulation", 1);
    //_vision_event = CreateEvent(NULL, true, false, NULL);
    initializeSingleton(); // init parammanager first
    CCtrlBreakHandler breakHandler;
    // SLEEP(1000);
    COptionModule option; // right or left, yellow or blue
	vision->registerOption(&option);
	WORLD_MODEL->registerVision(vision);
	WORLD_MODEL->registerOption(&option);
	CDecisionModule decision(&option, vision);
	CActionModule action(&option, vision, &decision);
	MATCH_STATE->initialize(&option,vision);

    _best_visiondata_copy_mutex = new QMutex;
    _value_getter_mutex = new QMutex;
    _debug_mutex = new QMutex;
	GPUBestAlgThread::Instance()->initialize(VISION_MODULE);
	GameInfoT gameInfo;
    //SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);

	while (true) {
        //WaitForSingleObject(_vision_event, INFINITY);

		if (! DataReceiver4rbk::Instance()->getGameInfo(&option,gameInfo)) {
			continue;
		}

        vision->SetRefRecvMsg(gameInfo);
        vision->SetNewVision(gameInfo);
		
        if (breakHandler.halted()){
            decision.DoDecision(true);
            action.sendNoAction(gameInfo);
        } else {
			decision.DoDecision(false);
			action.sendAction(gameInfo);
        }

        GDebugEngine::Instance()->send(option.MyColor() == TEAM_BLUE);
		guiDebugMsgs.Clear();
	}
}

int main(int argc, char* argv[]) {
    QCoreApplication a(argc, argv);
    std::thread t(run);
    t.detach();
    return a.exec();
}