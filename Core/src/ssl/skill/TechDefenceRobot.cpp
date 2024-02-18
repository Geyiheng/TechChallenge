#include "TechDefenceRobot.h"

#include "VisionModule.h"
#include "Global.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <iostream>
#include "GDebugEngine.h"
#include "Vision/VisionModule.h"
#include "skill/Factory.h"
#include "utils.h"
#include "WorldModel/DribbleStatus.h"
#include "RobotSensor.h"
#include "param.h"
#include "WorldModel/WorldModel.h"
#include "geometry.h"


#include <vector>
#include <variant>
#include <string>
#include <cmath>
using namespace std;

namespace {enum TDstate {getball ,wait};}
int findIndex(const std::vector<int>& vec, int target) {
    auto it = std::find(vec.begin(), vec.end(), target);
    if (it != vec.end()) {return std::distance(vec.begin(), it);}
    std::cout<<"INDEX NOT FOUND";return -1;}
bool eraseElement(std::vector<int>& vec, int target) {
    auto it = std::find(vec.begin(), vec.end(), target);
    if (it != vec.end()) {vec.erase(it);return true;}
    return false;}
void drawDir(CGeoPoint A,double DIR,string msg){
    CGeoPoint D(A.x() + 100 * std::cos(DIR), A.y() + 100 * std::sin(DIR));
    CGeoSegment AD(A,D);
    GDebugEngine::Instance()->gui_debug_line(A,D,COLOR_YELLOW);
    GDebugEngine::Instance()->gui_debug_msg(D,(msg).c_str(), COLOR_RED);}
double qiexiandir[3];
void getcircledir (const CGeoPoint& playerpos, const CGeoPoint& centre, const double r){
    // CGeoLine player2centre(playerpos, centre);
    const CVector player2centre = playerpos - centre;
    const double vertical = player2centre.dir() + 3.1415926/2;
    const CGeoPoint p1 (centre.x() + r * cos(vertical), centre.y() + r * sin(vertical));
    const CGeoPoint p2 (centre.x() - r * cos(vertical), centre.y() - r * sin(vertical));
    const CVector player2p1 = playerpos - p1;
    const CVector player2p2 = playerpos - p2;
    qiexiandir[0] = player2p1.dir() < player2p2.dir() ? player2p1.dir() : player2p2.dir();
    qiexiandir[1] = player2p1.dir() > player2p2.dir() ? player2p1.dir() : player2p2.dir();}
double normalizeAngle1(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle <= -M_PI) angle += 2 * M_PI;
    return angle;}
double sortdir(double A, double B, double dir, double vel,double threshold) {
    if ((A >= dir && dir >= B) || (B >= dir && dir >= A)) {
        if (vel > threshold) {return normalizeAngle1(std::max(A, B));}
        else if (vel<=-threshold) {return normalizeAngle1(std::min(A, B));}}
    return 0;}
double amidDir(double A, double B, double dir,double vel,double Vthreshold) {
    const double thres = 0.2;
    double A1 = A;
    double B1 = B;
    double A2 = A;
    double B2 = B;
    if (std::abs(A - B) > M_PI) {
        if (A < 0) {A1 = 2 * M_PI + A;}
        if (B < 0) {B1 = 2 * M_PI + B;}
        if (A > 0) {A2 = A - 2 * M_PI;}
        if (B > 0) {B2 = B - 2 * M_PI;}}
    double tortn1 = sortdir(A1, B1, dir, vel,Vthreshold);
    double tortn2 = sortdir(A2, B2, dir, vel,Vthreshold);
    if      (tortn1!=0) {return tortn1;}
    else if (tortn2!=0) {return tortn2;}
    else                {return 0;}}
double gotoAngle(double a,double b,double c,double d,double DIR,double VR,CGeoPoint A){
    double a_c=amidDir(a,c,DIR,VR,0.1);
    return DIR;
    // std::cout<<"amidDir("<<a<<c<<DIR<<VR<<0.1<<"->"<<a_c<<std::endl;
    // if      (amidDir(a,d,DIR,1,0)){drawDir(A,DIR,"a d");return a;}
    // else if (amidDir(d,c,DIR,1,0)){drawDir(A,DIR,"d c");return c;}
    // else if (a_c)                 {drawDir(A,DIR,to_string(a_c).c_str());return a_c;}
    // else if (amidDir(a,b,DIR,1,0)){drawDir(A,DIR,"a b");return a;}
    // else if (amidDir(b,c,DIR,1,0)){drawDir(A,DIR,"b c");return c;}
}
CGeoPoint midpoint1(const CGeoPoint& A, const CGeoPoint& B) {return CGeoPoint((A.x() + B.x()) / 2.0, (A.y() + B.y()) / 2.0);}
CTechDefence::CTechDefence(){}

void CTechDefence::plan(const CVisionModule* pVision){
//----------------------------------------------INITIALIZE ALL PLAYER INFOS AND BALL INFOS
    std::vector<const PlayerVisionT*> OPptrs;
    std::vector<const PlayerVisionT*> TPptrs;
    std::vector<int> OProlenums;
    std::vector<int>TProlenums;
    for (int irole = 0; irole <= Param::Field::MAX_PLAYER; ++irole){
        const PlayerVisionT& OPtmp = pVision->OurPlayer(irole);
        const PlayerVisionT& TPtmp = pVision->TheirPlayer(irole);
        if (OPtmp.Valid()){OPptrs.push_back(&OPtmp);OProlenums.push_back(irole);}
        if (TPtmp.Valid()){TPptrs.push_back(&TPtmp);TProlenums.push_back(irole);}}
    const BallVisionT& ball = pVision->Ball();
//-------------------------------------------------CALCULATE WHICH CIRCLE TO FOCUS(RETURN CAR NUM)
    int focusNum=2;//car num
//-------------------------------------------------CALCULATE a,b,c,d,DIR,VR of the certain car num
    CGeoPoint A=TPptrs[findIndex(TProlenums,focusNum)]->Pos();
    double DIR=TPptrs[findIndex(TProlenums,focusNum)]->Dir();
    double VR=OPptrs[0]->RotVel();//should be TPptrs[findIndex(TProlenums,focusNum)]->RotVel();
    CGeoPoint ORA(0,0);
    // GDebugEngine::Instance()->gui_debug_msg(ORA, ("rotate velo" + std::to_string(VR)).c_str(), COLOR_RED);
    std::vector<int>TProlenumsR=TProlenums;
    eraseElement(TProlenumsR,focusNum);
    CGeoPoint A2=TPptrs[findIndex(TProlenums,TProlenumsR[0])]->Pos();
    CGeoPoint A3=TPptrs[findIndex(TProlenums,TProlenumsR[1])]->Pos();
    CGeoPoint O=ball.Pos();
    CGeoPoint M=midpoint1(A2,A3);
    CVector AM(M.x()-A.x(),M.y()-A.y());
    double b=AM.dir();
    double d=normalizeAngle1(b-M_PI);
    // auto ac=ball2circle(A,DIR,M,45*2,1);
    // double a=ac.first;
    // double c=ac.second;
    getcircledir(A,M,150);
    double a=normalizeAngle1(qiexiandir[0]-M_PI);
    double c=normalizeAngle1(qiexiandir[1]-M_PI);
// //-------------------------------------------------DEBUG INFOS
    std::cout<<"OProlenums:[";for (int i : OProlenums) {std::cout << i << " ";}std::cout <<"]"<< std::endl;
    std::cout<<"TProlenums:[";for (int i : TProlenums) {std::cout << i << " ";}std::cout <<"]"<< std::endl;
    std::cout<<"TProlenumsR:[";for (int i : TProlenumsR) {std::cout << i << " ";}std::cout <<"]"<< std::endl;
    // std::cout<<"findIndex(TProlenums,TProlenumsR[1])="<<findIndex(TProlenums,TProlenumsR[1])<<std::endl;
    // std::cout<<"TPptrs[1] pos x="<<TPptrs[TProlenumsR[0]]->Pos().x()<<std::endl;
    std::cout<<"b:{"<<b<<"};d{"<<d<<"};a{"<<a<<"};c{"<<c<<"}"<<std::endl;
    drawDir(A,a,"a");drawDir(A,b,"b");drawDir(A,c,"c");drawDir(A,d,"d");
//-------------------------------------------------GO TO WHICH ANGLE
    double JamAngle=gotoAngle(a,b,c,d,DIR,VR,A);
    drawDir(A,JamAngle,"JamAngle");
    CGeoPoint Jam(A.x() + 40 * std::cos(JamAngle), A.y() + 40 * std::sin(JamAngle));
//-------------------------------------------------SET TASK 
    TaskT taskR1(task());
    int rolenum=task().executor;
    taskR1.executor=OProlenums[0];
    taskR1.player.pos=Jam;
    taskR1.player.angle=normalizeAngle1(DIR-M_PI);
	setSubTask(TaskFactoryV2::Instance()->GotoPosition(taskR1));//将taskR1给走位subtask执行
	CStatedTask::plan(pVision);}

CTechDefence::~CTechDefence() {}

CPlayerCommand* CTechDefence::execute(const CVisionModule* pVision)
{if (subTask()){return subTask()->execute(pVision);} return NULL;}
