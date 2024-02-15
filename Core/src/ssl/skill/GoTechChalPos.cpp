#include "GoTechChalPos.h"
#include "GDebugEngine.h"
#include <Vision/VisionModule.h>
#include "RobotSensor.h"
#include "skill/Factory.h"
#include "geometry.h""
#include <MotionControl/ControlModel.h>
#include <utils.h>
#include <WorldModel/DribbleStatus.h>
#include <MotionControl/ControlModel.h>
#include "GDebugEngine.h"
#include "TaskMediator.h"
#include "BestPlayer.h"
#include <string>
#include <ctime>
#include <algorithm>

CGoTechChalPos::CGoTechChalPos() {
}

void CGoTechChalPos::plan(const CVisionModule* pVision) {

    const int _executor = task().executor;
    int flag = task().player.flag;
    const PlayerVisionT& me = pVision->OurPlayer(_executor);
    const BallVisionT& ball = pVision->Ball();
    CGeoPoint BallPos = ball.Pos();
/*
    printf("%.3lf\n", TheMinDistBetweenTheOppAndTheLine(pVision, BallPos, CGeoPoint(470,210)));
    TaskT SubTask(task());
    SubTask.player.pos = CGeoPoint(470,210);
    SubTask.player.angle = (ball.Pos() - CGeoPoint(470,210)).dir();
    GDebugEngine::Instance()->gui_debug_x(CGeoPoint(470,210), COLOR_RED);
    setSubTask(TaskFactoryV2::Instance()->SmartGotoPosition(SubTask));
    CStatedTask::plan(pVision);
*/
    if(BallPos.x() < -50){
        MiddlePoint = GenerateMiddlePoint(pVision);
        FrontPoint = GenerateFrontPoint(pVision, 1);
    }
    else {
        FrontPoint = GenerateFrontPoint(pVision, 0);
        MiddlePoint = GenerateMiddlePoint(pVision);
    }
    if(flag == 0){
        TaskT SubTask(task());
        SubTask.player.pos = FrontPoint;
        SubTask.player.angle = (ball.Pos() - FrontPoint).dir();
        GDebugEngine::Instance()->gui_debug_x(FrontPoint, COLOR_RED);
        setSubTask(TaskFactoryV2::Instance()->SmartGotoPosition(SubTask));
    }
    else {
        TaskT SubTask(task());
        SubTask.player.pos = MiddlePoint;
        SubTask.player.angle = (ball.Pos() - MiddlePoint).dir();
        GDebugEngine::Instance()->gui_debug_x(MiddlePoint, COLOR_RED);
        setSubTask(TaskFactoryV2::Instance()->SmartGotoPosition(SubTask));
    }

    CStatedTask::plan(pVision);
}

CGeoPoint CGoTechChalPos::GenerateFrontPoint(const CVisionModule* pVision, const int IsBack) {
    const BallVisionT& ball = pVision->Ball();
    const CGeoPoint BallPos = ball.Pos();
    const double r = 250;
    CGeoCircle Circle = CGeoCircle(GoalMiddlePoint, r);
    CGeoPoint ChoosePoint[50];
    int cnt = 0;
    k_m = 5;
    for (int i = 0;i <= 17; ++i){
        double NowDir = Param::Math::PI/50.0 * (double)i;

        CGeoLine NowLine = CGeoLine(GoalMiddlePoint, NowDir);
        CGeoLineCircleIntersection Intersection = CGeoLineCircleIntersection(NowLine, Circle);
        if(Intersection.point1().x() > 600.0)ChoosePoint[++cnt] = Intersection.point2();
        else ChoosePoint[++cnt] = Intersection.point1();

        NowDir = -1.0 * Param::Math::PI/50.0 * (double)i;
        NowLine = CGeoLine(GoalMiddlePoint, NowDir);
        Intersection = CGeoLineCircleIntersection(NowLine, Circle);
        if(Intersection.point1().x() > 600.0)ChoosePoint[++cnt] = Intersection.point2();
        else ChoosePoint[++cnt] = Intersection.point1();
    }
    while(k_m >= 0.5){
        double MinDist = 10000000;
        int Minidx = -1;
        for(int i = 1;i <= cnt;++i){
            if(!CanShoot(pVision, ChoosePoint[i]))continue;
            double NowDist;
            if(IsBack)NowDist = (BallPos - MiddlePoint).mod() + (MiddlePoint - ChoosePoint[i]).mod();
            else NowDist = (BallPos - ChoosePoint[i]).mod();
            if(NowDist < MinDist)MinDist = NowDist, Minidx = i;
        }
        if(Minidx != -1)return ChoosePoint[Minidx];
        else k_m -= 0.5;
    }
    return CGeoPoint(400, 0);
}

CGeoPoint CGoTechChalPos::GenerateMiddlePoint(const CVisionModule* pVision) {
    const BallVisionT& ball = pVision->Ball();
    const CGeoPoint BallPos = ball.Pos();
    const double r = 150;
    CGeoCircle Circle = CGeoCircle(CGeoPoint(0,0), r);
    CGeoPoint ChoosePoint[70];
    int cnt = 0;
    k_m = 5;
    for (int i = 0;i <= 29; ++i){
        double NowDir = Param::Math::PI/30.0 * (double)i;
        CGeoLine NowLine = CGeoLine(GoalMiddlePoint, NowDir);
        CGeoLineCircleIntersection Intersection = CGeoLineCircleIntersection(NowLine, Circle);
        ChoosePoint[++cnt] = Intersection.point1();
        ChoosePoint[++cnt] = Intersection.point2();
    }
    while(k_m >= 0.5){
        double MinDist = 10000000;
        int Minidx = -1;
        for(int i = 1;i <= cnt;++i){
            //if(!CanPass(pVision, ChoosePoint[i]))continue;
            double NowDist = (BallPos - ChoosePoint[i]).mod();
            if(NowDist < MinDist)MinDist = NowDist, Minidx = i;
        }
        if(Minidx != -1)return ChoosePoint[Minidx];
        else k_m -= 0.5;
    }
    return CGeoPoint(0, 0);
}

bool CGoTechChalPos::CanShoot(const CVisionModule* pVision, const CGeoPoint StartPoint){
    const BallVisionT& ball = pVision->Ball();
    const CGeoPoint BallPos = ball.Pos();
    CGeoPoint KickPoint = KickDirection::Instance()->GetTheShootPoint(pVision, StartPoint);
    if(KickPoint.y() > 50)KickPoint.setY(50);
    if(KickPoint.y() < -50)KickPoint.setY(-50);
    double opp2KickDist = TheMinDistBetweenTheOppAndTheLine(pVision, StartPoint, KickPoint);
//    printf("%.3lf  %.3lf  %.3lf\n", BallPos.x(), StartPoint.x(), opp2PassDist);
    if (opp2KickDist > k_m * Param::Vehicle::V2::PLAYER_SIZE)return true;// && opp2PassDist > k_m * Param::Vehicle::V2::PLAYER_SIZE)return true;
    return false;
}

bool CGoTechChalPos::CanPass(const CVisionModule* pVision, const CGeoPoint StartPoint){
    const BallVisionT& ball = pVision->Ball();
    const CGeoPoint BallPos = ball.Pos();
    double opp2KickDist = TheMinDistBetweenTheOppAndTheLine(pVision, StartPoint, FrontPoint);
    double opp2PassDist = TheMinDistBetweenTheOppAndTheLine(pVision, BallPos, StartPoint);
    if (opp2KickDist > k_m * Param::Vehicle::V2::PLAYER_SIZE && opp2PassDist > k_m * Param::Vehicle::V2::PLAYER_SIZE)return true;
    return false;
}


double CGoTechChalPos::TheMinDistBetweenTheOppAndTheLine(const CVisionModule* pVision, const CGeoPoint startPoint, const CGeoPoint targetPoint){
    int n = 0;
    double passDir = (targetPoint - startPoint).dir();
    CGeoLine start2Target = CGeoLine(startPoint, passDir);
    CGeoPoint projectionPoint;
    double opp2LineDist = 1000000;
    while (n <= Param::Field::MAX_PLAYER) {
        if (!pVision->TheirPlayer(n).Valid()) { n++; continue; }
        projectionPoint = start2Target.projection(pVision->TheirPlayer(n).Pos());
        if (opp2LineDist > (projectionPoint - pVision->TheirPlayer(n).Pos()).mod() && projectionPoint.x() < Param::Field::PITCH_LENGTH / 2.0
                && projectionPoint.x() > min(targetPoint.x(), startPoint.x()) && projectionPoint.x() < max(targetPoint.x(), startPoint.x())) {
            opp2LineDist = (projectionPoint - pVision->TheirPlayer(n).Pos()).mod();
        }
        n++;
    }

    return opp2LineDist;
}

CPlayerCommand* CGoTechChalPos::execute(const CVisionModule* pVision) {
    if (subTask()) {
        return subTask()->execute(pVision);
    }
    return NULL;
}
