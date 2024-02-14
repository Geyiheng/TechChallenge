#include "Tech3PassRobot.h"

#include "VisionModule.h"
#include "Global.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846


#include <iostream>
#include "GDebugEngine.h"
#include "Vision/VisionModule.h"
#include "skill/Factory.h"
#include "utils.h"
#include "WorldModel/DribbleStatus.h"
#include "RobotSensor.h"
#include "param.h"
#include "WorldModel/WorldModel.h"
#include <BallStatus.h>
#include "geometry.h"
#include <vector>
#include <variant>
#include <string>
#include <cmath>
using namespace std;

// namespace 
// {
// enum TDstate {getball ,wait};//setState(getball);if (state()==getball);
// }



CGeoPoint midpoint(const CGeoPoint& A, const CGeoPoint& B) {
    return CGeoPoint((A.x() + B.x()) / 2.0, (A.y() + B.y()) / 2.0);
}

// Function to calculate the cross product of two vectors A and B
double crossProduct(const CGeoPoint& A, const CGeoPoint& B, const CGeoPoint& C) {
    CGeoPoint AB(B.x() - A.x(), B.y() - A.y());
    CGeoPoint AC(C.x() - A.x(), C.y() - A.y());
    return AB.x() * AC.y() - AB.y() * AC.x();
}

// Main function to determine on which side of the line through A1 and AM the point B1 lies
int B1whichSide(const CGeoPoint& A1, const CGeoPoint& A2, const CGeoPoint& A3, const CGeoPoint& B1) {
    CGeoPoint AM = midpoint(A2, A3);
    double cp = crossProduct(A1, AM, B1);

    // Assuming positive cross product means B1 is on the same side as A2 relative to the line A1-AM
    if (cp > 0) {
        return 3; // B1 is on the side of A3
    } else {
        return 2; // B1 is on the side of A2
    }
}
CGeoPoint R01Head(const CGeoPoint& R, double DIR, double distance) {
    double deltaX = distance * cos(DIR); // Calculate the x offset
    double deltaY = distance * sin(DIR); // Calculate the y offset
    return CGeoPoint(R.x() + deltaX, R.y() + deltaY); // Return the new point
}
CTech3Pass::CTech3Pass() 
{
	
}

void CTech3Pass::plan(const CVisionModule* pVision)
{
	TaskT taskR1(task());
	int rolenum=task().executor;
	taskR1.executor=rolenum;
//----------------------------------------------INITIALIZE ALL PLAYER INFOS AND BALL INFOS
	const BallVisionT& ball = pVision->Ball();
	CGeoPoint O=ball.Pos();
	std::vector<const PlayerVisionT*> OPptrs;
    std::vector<int> rolenums;
    int possballrole=-1;
    for (int irole = 0; irole <= Param::Field::MAX_PLAYER; ++irole)
	{
		const PlayerVisionT& OPtmp=pVision->OurPlayer(irole);
		if (OPtmp.Valid())
		{
			OPptrs.push_back(&OPtmp);
            rolenums.push_back(irole);
            //------------------------------------------------FIND OUT WHO GETS THE BALL
            CGeoPoint Rtmp=OPtmp.Pos();
            CGeoPoint R0tmp=R01Head(Rtmp,OPtmp.Dir(),16);
            GDebugEngine::Instance()->gui_debug_msg(R0tmp,"R", COLOR_RED);
            if (R0tmp.dist(O)<10)
			{
				possballrole=irole;
			}
		}	
	}
// -----------------------------------------------PRINT PLAYER ROLE INFOS
	std::cout<<"possballrole<"<<possballrole<<">";

	// std::cout<<"task().executor"<<rolenum<<std::endl;
    // for (int i : rolenums) {
    //     std::cout << i << " ";
    // }
    // std::cout << std::endl;

// -----------------------------------------------CALCULATE THREE POINTS
	enum states {PrepareAndPass,Receive,Test};
	
	CGeoPoint B1;
	CGeoPoint A1(-75,-130);
	CGeoPoint A2(-75,130);
	CGeoPoint A3(150,0);
	CGeoPoint A4;
	setState(Test);
	if (state()==Test)
	{
// ------------------------------------------------IF ROBOT IS TO GO TO A POINT ,THEN DEFINE A1 A2 A3 ABOVE
	    if (rolenum==rolenums[1])
	    {
	    	A4=A1;
	    }
	    else if (rolenum==rolenums[2])
	    {
	    	A4=A2;
	    }
	    else if (rolenum==rolenums[3])
	    {
	    	A4=A3;
	    }
	    GDebugEngine::Instance()->gui_debug_arc(A1,300,0,360, COLOR_RED);
		GDebugEngine::Instance()->gui_debug_arc(A2,300,0,360, COLOR_RED);
		GDebugEngine::Instance()->gui_debug_arc(A3,300,0,360, COLOR_RED);
	    taskR1.player.pos=A4;
	    setSubTask(TaskFactoryV2::Instance()->GotoPosition(taskR1));
	    KickStatus::Instance()->setKick(1, 5000);
		CStatedTask::plan(pVision);
	}
	else if (state()==PrepareAndPass)
	{

	}// setSubTask(PlayerRole::makeItShootBallV2(1,0, task().player.flag));
	// CGeoPoint O0(0,0);
    // CGeoPoint O1(0,20);
    // CGeoPoint O2(0,40);
    // CGeoPoint O3(0,60);
    // CGeoPoint O4(0,80);
    // CGeoPoint O5(0,100);
    // CGeoPoint O6(0,120);

	// GDebugEngine::Instance()->gui_debug_msg(O1, ("SAH: " + std::to_string(SAH)).c_str(), COLOR_RED);
	// GDebugEngine::Instance()->gui_debug_msg(O2, ("SF: " + std::to_string(SF)).c_str(), COLOR_RED);
	// GDebugEngine::Instance()->gui_debug_msg(O3, ("VOP: " + std::to_string(VOP)).c_str(), COLOR_RED);
	// GDebugEngine::Instance()->gui_debug_msg(O4, ("VB: " + std::to_string(VB)).c_str(), COLOR_RED);
	// GDebugEngine::Instance()->gui_debug_msg(O5, ("SOH: " + std::to_string(SOH)).c_str(), COLOR_RED);
    // GDebugEngine::Instance()->gui_debug_msg(O6, ("AHP: " + std::to_string(AHP)).c_str(), COLOR_RED);
	
}
CTech3Pass::~CTech3Pass() {
}


CPlayerCommand* CTech3Pass::execute(const CVisionModule* pVision)
{
	if (subTask()){
		return subTask()->execute(pVision);
	}
	return NULL;
}
#endif