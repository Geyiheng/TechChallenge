#include "TechDefenceRobot.h"

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
#include "geometry.h"


#include <vector>
#include <variant>
#include <string>
#include <cmath>
using namespace std;

namespace 
{
enum TDstate {getball ,wait};//setState(getball);if (state()==getball);
}
double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle <= -M_PI) angle += 2 * M_PI;
    return angle;
}
bool DIRsame(double angle1, double angle2) {
    const double tolerance = 30 * M_PI / 180; // 5度转换为弧度
    angle1 = normalizeAngle(angle1);
    angle2 = normalizeAngle(angle2);
    double diff = std::abs(angle1 - angle2);
    double sum = std::abs(angle1 + angle2);
    bool same= diff <= tolerance || (std::abs(angle1) > M_PI - tolerance && std::abs(angle2) > M_PI - tolerance && sum <= tolerance);
    CGeoPoint T(0,-30);
    if (same){GDebugEngine::Instance()->gui_debug_msg(T,"same", COLOR_RED);}
    return same;
}
//2024.2.14
CGeoPoint initialpos(const CGeoPoint& A, const CGeoPoint& B, const CGeoPoint& C){
    CGeoLine AB = CGeoLine(A,B);
    CGeoLine AC = CGeoLine(A,C);
    CGeoCircle CircleB = CGeoCircle(B, 30);
    CGeoCircle CircleC = CGeoCircle(C, 30);

    CGeoLineCircleIntersection Intersection1 = CGeoLineCircleIntersection(AB, CircleB);
    CGeoPoint D = Intersection1.point2();
    CGeoLineCircleIntersection Intersection2 = CGeoLineCircleIntersection(AC, CircleC);
    CGeoPoint E = Intersection2.point2();
    CGeoLine VerL_AB = verticalLine1(AB,D);
    CGeoLine VerL_AC = verticalLine2(AC,E);
    
    CGeoLineLineIntersection Intersection3 = CGeoLineLineIntersection(VerL_AB, VerL_AC);
    return Intersection3.IntersectPoint();
}
//2024.2.15
CGeoPoint tacklepos(const CVisionModule* pVision){
    const BallVisionT& ball = pVision->Ball();
    CGeoPoint ballpos = ball.Pos();
    
    vector<CGeoPoint> a = {CGeoPoint(75,-130), CGeoPoint(75,130), CGeoPoint(-150,0)};
    vector<CGeoCircle> Circles = {CGeoCircle(a[0], 35), CGeoCircle(a[1], 35), CGeoCircle(a[2], 35)};

    int circlenum = 3, irole;
    for(int i=0;i<3;i++)if(ballpos.dist(a[i])<50)circlenum = i;
    if(circlenum ==3)return CGeoPoint(0,0);
    for(int i = 0; i <= Param::Field::MAX_PLAYER; ++i){
        const PlayerVisionT& target = pVision->TheirPlayer(i);
        if(target.Valid()){
            CGeoPoint targetpos = target.Pos();
            if(targetpos.dist(a[circlenum])<30){
                irole = i;break;
            }
        }    
    }
    const PlayerVisionT& target = pVision->TheirPlayer(irole);
    CGeoPoint targetpos = target.Pos();
    CGeoPoint b(targetpos.x()+60*std::cos(target.Dir()),targetpos.y()+60*std::sin(target.Dir()));
    CGeoSegment Veldir(targetpos, b);
    //CGeoLine Veldir(targetpos,target.Dir());
    CGeoSegmentCircleIntersection Intersection = CGeoSegmentCircleIntersection(Veldir, Circles[circlenum]);
    return Intersection.point1();


}
CGeoPoint backpos(const CGeoPoint& A, const CGeoPoint& H, double distback) {
    // 计算向量AH
    double dx = H.x() - A.x();
    double dy = H.y() - A.y();
    
    // 计算AH向量的长度
    double lengthAH = std::sqrt(dx * dx + dy * dy);
    
    // 计算单位向量的方向
    double unitX = dx / lengthAH;
    double unitY = dy / lengthAH;
    
    // 根据distback正负确定B点在AH的同方向还是反方向
    // 并计算B点的坐标
    double Bx = A.x() + unitX * distback;
    double By = A.y() + unitY * distback;
    
    return CGeoPoint(Bx, By);
}

// 假设 CGeoPoint 和 CGeoLine 的定义如前所述
class Intercept {
public:
    Intercept(const CGeoPoint& O, const double& DIR, const CGeoPoint& A)
        : O(O), DIR(DIR), A(A), M(O, CGeoPoint(O.x() + std::cos(DIR), O.y() + std::sin(DIR))) {
        H = M.projection(A); // 计算垂足H并存储
        SOH = O.dist(H);
    }
    
    CGeoPoint FootH() const {
        return H; // 直接返回计算得到的垂足
    }

    CGeoPoint AHPFootH(double AHP) const {
        // 计算O到H的方向
        double SHP=A.dist(H)*tan(AHP);
        double ohDir = (H - O).dir();
        if(!DIRsame(DIR,ohDir)) {
            GDebugEngine::Instance()->gui_debug_msg(O,"O", COLOR_PURPLE);
            GDebugEngine::Instance()->gui_debug_msg(H,"H", COLOR_PURPLE);
            return backpos(H, O, SHP); 
        } else {
            GDebugEngine::Instance()->gui_debug_msg(O,"O", COLOR_YELLOW);
            GDebugEngine::Instance()->gui_debug_msg(H,"H", COLOR_YELLOW);
            return backpos(O, H, SOH + SHP);// 如果方向相反，延伸距离需要为负
        }
    }

    CGeoLine LineM() const {
        return M; // 直接返回计算得到的射线
    }

    double DistanceAH() const {
        return A.dist(H); // 直接返回A到H的距离
    }

    double DistanceOH() const {
        return SOH;
    }
    double OHDIR() const {
        return (H - O).dir();
    }

private:
    CGeoPoint O;
    double DIR;
    CGeoPoint A;
    CGeoLine M;
    CGeoPoint H;
    double SOH; // H到O的距离
};


double timeOH(double v0, double s) {
    double a = -1.93;
    double discriminant = v0 * v0 - 2 * a * (-s);
    if (discriminant < 0) {
        if (v0>0.5)
        {
        	return -v0/a;
        }

    }
    double t = (-v0 + std::sqrt(discriminant)) / a;
    return t;
}

double getdistback(const double& SAH, const double& TAH) {
	double maxa=4;
    double SF = 0.5 * maxa * std::pow(TAH, 2); // 使用std::pow进行幂运算
    if (SAH >= SF) {
        return 2 * SAH; // 假设这里是期望的返回逻辑
    } else if (0.5 * SF < SAH && SAH < SF) { // 分开进行比较
        double T2 = TAH * std::pow((SF - SAH) / (2 * SF), 0.5); // 再次使用std::pow
        return maxa * std::pow( (TAH - T2), 2); // 修改括号和幂运算
    }
    return SAH;
}
#include <cmath> // For std::sqrt, std::cos, and std::sin
#include <iostream>

// Assuming CGeoPoint and CGeoLine classes are defined as provided above



CTechDefence::CTechDefence() 
{
	
}

void CTechDefence::plan(const CVisionModule* pVision)
{
	TaskT taskR1(task());
	int rolenum=task().executor;
//----------------------------------------------INITIALIZE ALL PLAYER INFOS AND BALL INFOS
	std::vector<const PlayerVisionT*> OPptrs;
    std::vector<const PlayerVisionT*> OPptrs1;
    std::vector<int> rolenums;
	for (int irole = 0; irole <= Param::Field::MAX_PLAYER; ++irole)
	{
		const PlayerVisionT& OPtmp=pVision->OurPlayer(irole);
		if (OPtmp.Valid())
		{
			OPptrs.push_back(&OPtmp);
            rolenums.push_back(irole);
		}
        const PlayerVisionT& OPtmp1=pVision->TheirPlayer(irole);
        if(OPtmp1.Valid())OPptrs1.push_back(&OPtmp1);
	}
	const BallVisionT& ball = pVision->Ball();//ball.Pos().x() ball().Pos().y()  ball.Vel().mod() ball.Vel().dir()
//----------------------------------------------PRINT ROLE NUM INFOS
    for (int i : rolenums) {
        std::cout << i << " ";
    }
    std::cout << std::endl;
    std::cout<<"task().executor"<<rolenum<<std::endl;
    taskR1.executor=rolenums[1];
// ---------------------------------------------PREPARE INPUTS FOR CLASS INTERCEPT 
	CGeoPoint O=ball.Pos(); // 射线的起点
    double DIR =ball.Vel().dir(); 
    CGeoPoint A=OPptrs[1]->Pos(); // 射线外的点A,应该先有一个谁快要接到球的判断车号，或踢球者也可以改这个谁快要接到球的车号
// ----------------------------------------------GET INFOS FROM CLASS INTERCEPT
    
    Intercept calculator(O, DIR, A);
    CGeoLine M = calculator.LineM(); 
    double VB=ball.Vel().mod()*0.01;
    double VOP=OPptrs[1]->Vel().mod()*0.01;
    double maxa=3;
    double SAH;
    double SOH;
    double TAH;
    double distback;
    double SF;
    double AHP=0;
    std::vector<double> SAHs;
    std::vector<double> SFs;
    CGeoPoint H;
// -----------------------------------------------WHETHER THERE IS ENOUGH TIME TO INTERCEPT THE BALL
    H=calculator.AHPFootH(0);
    SAH = A.dist(H)*0.01; // 获取A到H的距离
    SOH=O.dist(H)*0.01;
    TAH=timeOH(VB,SOH); 
    // double distback=getdistback(SAH,TAH);
    SF = 0.5 * maxa * std::pow(TAH, 2)+TAH*VOP;
    if (SAH>=SF-0.1) //添加车运动方向-->H 的限制
    {
// --------------------------------------------THERE IS NO AMPLE TIME TO INTERCEPT THE BALL.DECIDE ON THE INTERCEPT ANGLE
        distback=1000;
        for (AHP=0;AHP<M_PI/12;AHP+=0.05)
        {

            H=calculator.AHPFootH(AHP);
            SAH = A.dist(H)*0.01; // 获取A到H的距离
            SOH=O.dist(H)*0.01;
            TAH=timeOH(VB,SOH); 
            SF = 0.5 * maxa * std::pow(TAH, 2)+TAH*VOP;
            if (SAH+0.16<SF) //添加车运动方向-->H 的限制
            {
                break;   
            }
            SAHs.push_back(SAH);
            SFs.push_back(SF);
        }
    }else 
    {
        distback=0;
    }
//----------------------------------------------CALCULATE POINTBACK FROM DISTBACK.THE distback AND H ARE RENEWED
    CGeoPoint B= backpos(A,H,distback);
    taskR1.player.pos=B;
//----------------------------------------------PRINT ALL INFOS
    GDebugEngine::Instance()->gui_debug_msg(O,"O", COLOR_RED);
    GDebugEngine::Instance()->gui_debug_msg(H,"HH", COLOR_RED);
    GDebugEngine::Instance()->gui_debug_msg(A,"AAA", COLOR_RED);
    GDebugEngine::Instance()->gui_debug_msg(B,"BBBB", COLOR_RED);
    CGeoPoint O0(0,0);
    CGeoPoint O1(0,20);
    CGeoPoint O2(0,40);
    CGeoPoint O3(0,60);
    CGeoPoint O4(0,80);
    CGeoPoint O5(0,100);
    CGeoPoint O6(0,120);
    CGeoPoint O7(0,140);
    CGeoPoint O8(0,160);
	GDebugEngine::Instance()->gui_debug_msg(O0, ("TAH: " + std::to_string(TAH)).c_str(), COLOR_RED);
	GDebugEngine::Instance()->gui_debug_msg(O1, ("SAH: " + std::to_string(SAH)).c_str(), COLOR_RED);
	GDebugEngine::Instance()->gui_debug_msg(O2, ("SF: " + std::to_string(SF)).c_str(), COLOR_RED);
	GDebugEngine::Instance()->gui_debug_msg(O3, ("VOP: " + std::to_string(VOP)).c_str(), COLOR_RED);
	GDebugEngine::Instance()->gui_debug_msg(O4, ("VB: " + std::to_string(VB)).c_str(), COLOR_RED);
	GDebugEngine::Instance()->gui_debug_msg(O5, ("SOH: " + std::to_string(SOH)).c_str(), COLOR_RED);
    GDebugEngine::Instance()->gui_debug_msg(O6, ("AHP: " + std::to_string(AHP)).c_str(), COLOR_RED);
    GDebugEngine::Instance()->gui_debug_msg(O7, ("DIR: " + std::to_string(DIR)).c_str(), COLOR_RED);
    GDebugEngine::Instance()->gui_debug_msg(O8, ("OHDIR: " + std::to_string(calculator.OHDIR())).c_str(), COLOR_RED);
   	GDebugEngine::Instance()->gui_debug_line(O,H,COLOR_WHITE);
   	GDebugEngine::Instance()->gui_debug_line(A,B,COLOR_WHITE);
    CGeoPoint A1(75,-130);
    CGeoPoint A2(75,130);
    CGeoPoint A3(-150,0);
    GDebugEngine::Instance()->gui_debug_arc(A1,30,0,360, COLOR_YELLOW);
    GDebugEngine::Instance()->gui_debug_arc(A2,30,0,360, COLOR_YELLOW);
    GDebugEngine::Instance()->gui_debug_arc(A3,30,0,360, COLOR_YELLOW);
    CGeoPoint I = initialpos(OPptrs1[1]->Pos(),OPptrs1[2]->Pos(),OPptrs1[3]->Pos());
    CGeoPoint J = tacklepos(pVision);
    GDebugEngine::Instance()->gui_debug_msg(I,"I", COLOR_YELLOW);
    GDebugEngine::Instance()->gui_debug_msg(J,"J", COLOR_YELLOW);

    for (double iSAH:SAHs)
    {
        
        GDebugEngine::Instance()->gui_debug_msg(B,"BBBB", COLOR_RED);
    }

//----------------------------------------------SET SUB TASKS
	setSubTask(TaskFactoryV2::Instance()->GotoPosition(taskR1));//将taskR1给走位subtask执行

	CStatedTask::plan(pVision);
}


CTechDefence::~CTechDefence() {

}


CPlayerCommand* CTechDefence::execute(const CVisionModule* pVision)
{
	if (subTask()){
		return subTask()->execute(pVision);
	}
	return NULL;
}
#endif