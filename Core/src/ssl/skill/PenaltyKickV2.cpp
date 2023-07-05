/************************************************************************/
/*		modified by ������
        2023.5
        long dist penalty, rebuilt based on Advance
*/
/************************************************************************/
#include "PenaltyKickV2.h"
#include <Vision/VisionModule.h>
#include <WorldModel/KickStatus.h>
#include "RobotSensor.h"
#include "skill/Factory.h"
#include <WorldModel/WorldModel.h>
#include "WorldModel/DribbleStatus.h"
#include "PointCalculation/IndirectDefender.h"
#include <utils.h>
#include "defenceNew/DefenceInfoNew.h"
#include "KickDirection.h"
#include <GDebugEngine.h>
#include <iostream>
#include <BestPlayer.h>
#include <TaskMediator.h>
#include "Global.h"


CPenaltyKickV2::CPenaltyKickV2()
{
    KICK_DIST = paramManager->KICK_DIST;  /*��������Χ Խ��Խ��������*/
    WantToLessShoot = paramManager->WantToLessShoot; /*��������Խ��Խ�������� ���Ϊ0 ���Ϊ5*/
    RELIEF_DIST = paramManager->RELIEF_DIST;  /*GET�н���״���µ�RELIEF�жϾ���*/
    OPP_HAS_BALL_DIST = paramManager->OPP_HAS_BALL_DIST; /*�жϵз��Ƿ�����ľ��� ��Ҫ����*/
    CanPassToWingDist = paramManager->CanPassToWingDist; /*Advance�ܹ������߷���ٽ����*/
    CanWingShootDist = paramManager->CanWingShootDist; /*�߷��ܹ����ŵ��ٽ����*/
    SHOOT_PRECISION = paramManager->SHOOT_PRECISION;	/*����������С���ȽǷ�ĸ��Խ��Խ��Խ��ȷ ���Ϊ7���17*/
    GetBallBias = paramManager->AdGetBallBias;	/*AdvanceGetball��ƫ��*/
    BalltoMeVel = paramManager->BalltoMeVelTime; /*Advance�����������ȥ�ӵ��ٽ��ٶ�*/
    /*�������Ȳ���*/
    KICKPOWER = paramManager->KICKPOWER;
    CHIPPOWER = paramManager->CHIPPOWER; // ��ʱ������
    ADV_FPASSPOWER = paramManager->ADV_FPASSPOWER;
    ADV_CPASSPOWER = paramManager->ADV_CPASSPOWER;
    RELIEF_POWER = paramManager->RELIEF_POWER;
    BACK_POWER = paramManager->BACK_POWER;
    Advance_DEBUG_ENGINE = paramManager->Advance_DEBUG_ENGINE;
}

double getbetterPointShootDir(const CVisionModule* pVision, CGeoPoint startPoint);
double getPointPUshDir(const CVisionModule* pVision, CGeoPoint startPoint);
CGeoPoint getbetterPointShoot(const CVisionModule* pVision, CGeoPoint startPoint);

CPenaltyKickV2::~CPenaltyKickV2() {

}

void CPenaltyKickV2::plan(const CVisionModule* pVision)
{
    if (pVision->Cycle() - _cycle > Param::Vision::FRAME_RATE * 0.1) {
        _state = BEGIN;
    }
    /**********************************************************
    * Description: ��ʼ����Ҫ���������б�
    * Author: ̷���
    * Created Date: 2022/10/10
    ***********************************************************/
    const int maxFrared = 100 * 1.25;
    const int maxMeHasBall = int(50 * 1.25);
    int _executor = task().executor;
    int tandemNum = task().ball.receiver;
    int DoNotEnterDefenseBox = PlayerStatus::DODGE_OUR_DEFENSE_BOX;
    int AllowDribbleFlag = PlayerStatus::DRIBBLING;
    int ShootAllowDribble = DoNotEnterDefenseBox | AllowDribbleFlag;
    int ShootNotNeedDribble = DoNotEnterDefenseBox & (~AllowDribbleFlag);
    bool frared = RobotSensor::Instance()->IsInfraredOn(_executor);
    if (frared) { infraredOn = infraredOn >= maxFrared ? maxFrared : infraredOn + 1; }
    else { infraredOn = 0; }

    /**********************************************************
    * Description: ��ʼ����������б�
    * Author: ̷���
    * Created Date: 2022/10/10
    ***********************************************************/
    const PlayerVisionT& me = pVision->OurPlayer(_executor);
    const BallVisionT& ball = pVision->Ball();
    int GoalieNumber = 0;
    bool isMeHasBall = false;
    bool isMechHasBall = BallStatus::Instance()->getBallPossession(true, _executor) > 0.3;//infraredOn >= 2;
    printf("%d\n", infraredOn);
    bool visionHasBall = isVisionHasBall(pVision, _executor);
    isMeHasBall = isMechHasBall; //isMechHasBall&&
    if (isMeHasBall) {
        meHasBall = meHasBall >= maxMeHasBall ? maxMeHasBall : meHasBall + 1;
        meLoseBall = 0;
    }
    else {
        meHasBall = 0;
        meLoseBall = meLoseBall >= maxMeHasBall ? maxMeHasBall : meLoseBall + 1;
    }
    opponentID = pVision->TheirPenaltyGoalie();
    double BallToOurGoal = (ball.Pos() - ourGoal).mod();
    CVector me2goal = theirCenter - me.Pos();
    const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
    const CVector opp2ball = ball.Pos() - opp.Pos();
    double ball2oppDist = opp2ball.mod();
    double ball2meDist = (ball.Pos() - me.Pos()).mod();
    KickorPassDir = KickDirection::Instance()->getPointShootDir(pVision, pVision->OurPlayer(_executor).Pos());
    CGeoPoint ShootPoint, PassPoint;/*���������ŵķ��� Ӧ����һ��������ʾ ���пɳ�����������*/

    NormalPlayUtils::generatePassPoint(ball.Pos(), SupportPoint[0], SupportPoint[1], SupportPoint[2], SupportPoint[3]);
    /* Gpu��� */

    /**********************************************************
    * Description: ��Ļ�Ҳ� ����has��lose��ͼ��
    * Author: ̷���
    * Created Date: 2022/10/10
    ***********************************************************/
    if (Advance_DEBUG_ENGINE) {
        GDebugEngine::Instance()->gui_debug_x(CGeoPoint(-545, -int(300 * meHasBall / maxMeHasBall)), COLOR_YELLOW);
        GDebugEngine::Instance()->gui_debug_x(CGeoPoint(-575, -int(300 * meLoseBall / maxMeHasBall)), COLOR_ORANGE);
        GDebugEngine::Instance()->gui_debug_x(CGeoPoint(-545, -int(300 * meHasBall / maxMeHasBall)), COLOR_YELLOW);
        GDebugEngine::Instance()->gui_debug_x(CGeoPoint(-575, -int(300 * meLoseBall / maxMeHasBall)), COLOR_ORANGE);
        GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(-545, -300), "HAS", COLOR_BLACK);
        GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(-575, -300), "LOSE", COLOR_BLACK);
    }

    /**********************************************************
    * Description: ״̬����
    * Author: ������
    * Created Date: 2023/5/9
    ***********************************************************/
    switch (_state) {
    case BEGIN:
        _state = GET;
        break;
    case GET:
        if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -400), "Push GET", COLOR_YELLOW);
        GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -200), to_string(meHasBall).c_str());
        if (BallStatus::Instance()->getBallPossession(true, _executor) > 0.3) {
            
            if (fabs(opp.VelX()) > 200 && opp.X() < 430) {
                _state = BREAKSHOOT; break;
            }
            if (me2goal.mod() < KICK_DIST) {
                if (tendToShoot(pVision, _executor)) {
                    _state = BREAKSHOOT; break;
                }
                else if (Me2OppTooclose(pVision, _executor)) {
                    _state = BREAKSHOOT; break;
                }
                else {
                    _state = BREAKSHOOT; break;
                }

            }
            else {
                if (Me2OppTooclose(pVision, _executor)) {
                    _state = BREAKSHOOT; break;
                }
                else {
                    _state = LIGHT_KICK; break;
                }
            }
        }
        else { _state = GET; break; }

    case KICK:
        if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -400), "Push KICK", COLOR_YELLOW);
        if (meLoseBall > 15 && ball2meDist > 10) _state = GET;
        break;

    case BREAKSHOOT:
        if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -400), "Push BREAKSHOOT", COLOR_YELLOW);
        if (meLoseBall > 18 && ball2meDist > 10) _state = GET;
        break;

    case CHASEKICK:
        if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -400), "Push CHASEKICK", COLOR_YELLOW);
        if (meLoseBall > 18 && ball2meDist > 10) _state = GET;
        break;

    case LIGHT_KICK:
        if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(200, 400), "LIGHT KICk", COLOR_YELLOW);
        if (Advance_DEBUG_ENGINE) { cout << "light kick -> lightkick" << endl; }
        if (meLoseBall > 30 * 1) {
            //if (DEBUG_ENGINE) { cout << "light kick --> goto" << endl; }
            _state = GET;
        }
        else if (me2goal.mod() < KICK_DIST || Me2OppTooclose(pVision, _executor)) {
            if (Advance_DEBUG_ENGINE) { cout << "light kick -> shoot" << endl; }
            if (tendToShoot(pVision, _executor)) {
                if (BallStatus::Instance()->getBallPossession(true, _executor) > 0.3)
                    _state = KICK;
                else
                    _state = GET;
                break;
            }
            else {
                if (BallStatus::Instance()->getBallPossession(true, _executor) > 0.3)
                    _state = BREAKSHOOT;
                else
                    _state = GET;
                break;
            }
        }
        break;
    }


    /**********************************************************
    * Description: ״ִ̬��
    * Author: ̷���
    * Created Date: 2022/10/10
    ***********************************************************/
    switch (_state) {
    case GET:
        if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(200, -400), "let GET", COLOR_YELLOW);
        if (!ball.Valid()) {
            /*�򲻺Ϸ������*/
            if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -400), "Ball invalid", COLOR_ORANGE);
            double faceDir = opp.Dir() + Param::Math::PI;
            setSubTask(PlayerRole::makeItChaseKickV2(_executor, faceDir, ShootNotNeedDribble));
        }
        else {
            /*��û�еõ��� ��Ҫȥgetball*/
            if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -300), "LOSE and GETBALL", COLOR_ORANGE);
            //KickorPassDir = KickDirection::Instance()->getPointShootDir(pVision, pVision->OurPlayer(_executor).Pos());
            /*�˴�����ɳ־û����� ����Ҫ���иı�*/
            //if (pVision->Ball().X() <= 0)
            //    setSubTask(PlayerRole::makeItNoneTrajGetBallForStatic(_executor, KickorPassDir, CVector(0, 0), ShootNotNeedDribble, GetBallBias));
            //else
            setSubTask(PlayerRole::makeItNoneTrajGetBall(_executor, KickorPassDir, CVector(0, 0), ShootNotNeedDribble, GetBallBias));
        }
        break;
    case KICK:   // ����
        if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(200, -400), "KICK", COLOR_YELLOW);
        //KickorPassDir = KickDirection::Instance()->getPointShootDir(pVision, pVision->OurPlayer(_executor).Pos());
        KickorPassDir = getbetterPointShootDir(pVision, pVision->OurPlayer(_executor).Pos());
        KickStatus::Instance()->clearAll();
        //if (Utils::InTheirPenaltyArea(ball.Pos(), 0)) {
            /*������ڶԷ�����*/
            //������ߵ������ˣ����������һ�ξ��룬����ô��
            //if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -400), "ball in their PEN", COLOR_ORANGE);
           // KickStatus::Instance()->setKick(_executor, KICKPOWER);
           // setSubTask(PlayerRole::makeItShootBallV2(_executor, KickorPassDir, ShootNotNeedDribble));
       // }
       // else {
            /*����KICK�׶�  ��Ҫ�����Ƿ����Ѿ�ת��ɹ�  �˴���δ�걸���ܴ���BUG*/
        if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -400), "Try Kick", COLOR_ORANGE);
        GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -200), to_string(KickorPassDir).c_str());
        if (isDirOK(pVision, _executor, KickorPassDir, 1)) {
            if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -350), "Kick isDirOK,make kick", COLOR_ORANGE);
            KickStatus::Instance()->setKick(_executor, KICKPOWER);
            setSubTask(PlayerRole::makeItShootBallV2(_executor, KickorPassDir, ShootNotNeedDribble));
        }
        else {
            DribbleStatus::Instance()->setDribbleCommand(_executor, 1);
            setSubTask(PlayerRole::makeItSimpleGoto(_executor, me.Pos(), KickorPassDir));
            //setSubTask(PlayerRole::makeItGoAndTurnKickV4(_executor, kickDir));
            //setSubTask(PlayerRole::makeItNoneTrajGetBall(_executor, KickorPassDir, CVector(0, 0), ShootNotNeedDribble, GetBallBias));
          //  KickStatus::Instance()->setKick(_executor, KICKPOWER);
            //setSubTask(PlayerRole::makeItShootBallV2(_executor, KickorPassDir, ShootNotNeedDribble));
            if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -350), "Kick is  DirOK after turn", COLOR_ORANGE);
        }
        // }
        break;

    case BREAKSHOOT:
        if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(200, -400), "BREAKSHOOT", COLOR_YELLOW);
        KickStatus::Instance()->clearAll();
        ShootPoint = GenerateBreakShootPoint(pVision, _executor);
        setSubTask(PlayerRole::makeItBreak(_executor, 0, true, true, true));
        break;

    case CHASEKICK:
        if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(200, -400), "CHASEKICK", COLOR_YELLOW);
        KickorPassDir = KickDirection::Instance()->getPointShootDir(pVision, pVision->OurPlayer(_executor).Pos());
        if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -400), "CHASEKICK", COLOR_ORANGE);
        setSubTask(PlayerRole::makeItChaseKickV2(_executor, KickorPassDir, ShootNotNeedDribble));
        break;

    case LIGHT_KICK:
        if (!shootfarflag)
        {
            if (pVision->Ball().X() <= 0)
            {
                KickorPassDir = Param::Math::PI * 0.05;
                setSubTask(PlayerRole::makeItNoneTrajGetBall(_executor, KickorPassDir, CVector(0, 0), ShootNotNeedDribble, GetBallBias));
                GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -350), "try crash", COLOR_ORANGE);
            }
            else {
                KickorPassDir = getPointPUshDir(pVision, pVision->OurPlayer(_executor).Pos());
                setSubTask(PlayerRole::makeItNoneTrajGetBall(_executor, KickorPassDir, CVector(0, 0), ShootNotNeedDribble, GetBallBias));
                GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -350), "try lightkick", COLOR_ORANGE);
            }
            KickStatus::Instance()->clearAll();
            if (isDirOK(pVision, _executor, KickorPassDir, 1) && pVision->Ball().X() <= -100)
            {
                KickStatus::Instance()->setKick(_executor, 250);
                DribbleStatus::Instance()->setDribbleCommand(_executor, 0);
                //if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -350), "Let Shoot FAR", COLOR_ORANGE);
                GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -350), " Let Shoot FAR", COLOR_ORANGE);
            }
            if (isDirOK(pVision, _executor, KickorPassDir, 1) && pVision->Ball().X() >= -100 && pVision->Ball().X() <= 100)
                //if(pVision->Ball().X() <= 50)
            {
                KickStatus::Instance()->setKick(_executor, 190); // kick lightly
                DribbleStatus::Instance()->setDribbleCommand(_executor, 0);
                // setSubTask(PlayerRole::makeItSimpleGoto(_executor, ball.Pos(), KickorPassDir));
                //if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -350), "Let Shoot Near", COLOR_ORANGE);
                GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -350), "Let Shoot Near", COLOR_ORANGE);
            }
            if (isDirOK(pVision, _executor, KickorPassDir, 1) && pVision->Ball().X() >= 100)
                //if(pVision->Ball().X() <= 50)
            {
                KickStatus::Instance()->setKick(_executor, 160); // kick lightly
                DribbleStatus::Instance()->setDribbleCommand(_executor, 0);
                //if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -350), "Let Shoot very Near", COLOR_ORANGE);
                GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -350), "Let Shoot Near", COLOR_ORANGE);
            }
            //else
            //    KickStatus::Instance()->setKick(_executor, 10);
            break;
        }
        else
        {
            KickStatus::Instance()->clearAll();
            KickorPassDir = Param::Math::PI * 0.08;
            DribbleStatus::Instance()->setDribbleCommand(_executor, 1);
            setSubTask(PlayerRole::makeItSimpleGoto(_executor, me.Pos(), KickorPassDir));
            if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -350), "Try Shoot Far", COLOR_ORANGE);
            // if (isDirOK(pVision, _executor, KickorPassDir, 1)) {
             //    if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -350), "Let Shoot Far", COLOR_ORANGE);
              //   KickStatus::Instance()->setChipKick(_executor, 150);
            // }
            if (isDirOK(pVision, _executor, KickorPassDir, 1) && pVision->Ball().X() <= 0) {
                if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -350), "Let Shoot Far", COLOR_ORANGE);
                KickStatus::Instance()->setKick(_executor, 230);
                DribbleStatus::Instance()->setDribbleCommand(_executor, 0);
            }
            if (pVision->Ball().X() >= 0) //if(pVision->Ball().X() <= 50)
            {
                if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -350), "Let Shoot Near", COLOR_ORANGE);
                KickStatus::Instance()->setKick(_executor, 110); // kick lightly
                DribbleStatus::Instance()->setDribbleCommand(_executor, 0);
            }
            //else
            //    KickStatus::Instance()->setKick(_executor, 10);
            break;
        }
    }
    _cycle = pVision->Cycle();
    CStatedTask::plan(pVision);
}

/**********************************************************
    * Description: ����ຯ���������Ӿ���λ�õ��ж�
    * Author: ̷���
    * Created Date: 2022/10/10
***********************************************************/
bool CPenaltyKickV2::isVisionHasBall(const CVisionModule* pVision, const int vecNumber) {
    const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
    const BallVisionT& ball = pVision->Ball();
    double visionJudgDist = 12;
    bool distVisionHasBall = CVector(me.Pos() - ball.Pos()).mod() <= visionJudgDist;
    bool dirVisionHasBall;
    double meDir = me.Dir();
    double me2Ball = (ball.Pos() - me.Pos()).dir();
    double meDir_me2Ball_Diff = abs(Utils::Normalize((meDir - me2Ball)));
    if (meDir_me2Ball_Diff < Param::Math::PI / 6.0)
        dirVisionHasBall = true;
    else
        dirVisionHasBall = false;
    bool isVisionPossession = dirVisionHasBall && distVisionHasBall;
    return isVisionPossession;
}

double getbetterPointShootDir(const CVisionModule* pVision, CGeoPoint startPoint)
{
    double SHOOT_BLOCKED_RANGE = 3;
    int n = 0;
    const BallVisionT& ball = pVision->Ball();
    bool shootBlocked = false;
    CGeoPoint projectionPoint;
    double w = Param::Field::GOAL_WIDTH;
    double k_m = SHOOT_BLOCKED_RANGE;//ball2theirbest / 15;
    double goal[3] = { -0.4 * w,0,0.4 * w };
    CGeoPoint TheShootPoint;
    double opp2LineDist_max = 0;
    int best_i = -1;
    // int theirGoalie = pVision->TheirGoalie();
    for (int i = 0; i < 3; i++) {
        double shootDir = (CGeoPoint(Param::Field::PITCH_LENGTH / 2.0, goal[i]) - startPoint).dir();
        CGeoLine ball2ourGoal = CGeoLine(startPoint, shootDir);
        double opp2LineDist = 1000;
        n = 0;
        while (n <= Param::Field::MAX_PLAYER) {
            if (!pVision->TheirPlayer(n).Valid()) { n++; continue; }
            //ȡ��ֱ�������С
            projectionPoint = ball2ourGoal.projection(pVision->TheirPlayer(n).Pos());
            if (opp2LineDist > (projectionPoint - pVision->TheirPlayer(n).Pos()).mod() && projectionPoint.x() >= startPoint.x()) {
                opp2LineDist = (projectionPoint - pVision->TheirPlayer(n).Pos()).mod();
            }
            n++;
        }
        //ȡ�����������
        if (opp2LineDist_max < opp2LineDist) {
            opp2LineDist_max = opp2LineDist;
            best_i = i;
        }
    }
    double shootDir = (CGeoPoint(Param::Field::PITCH_LENGTH / 2.0, goal[best_i]) - startPoint).dir();
    TheShootPoint = CGeoPoint(Param::Field::PITCH_LENGTH / 2.0, goal[best_i]);
    //���ò����isdirok�Ĳ���
   // if (opp2LineDist_max < k_m * Param::Vehicle::V2::PLAYER_SIZE) {
     //   shootBlocked = true;
        //return 1000;
    //}
    return shootDir;
}
CGeoPoint getbetterPointShoot(const CVisionModule* pVision, CGeoPoint startPoint)
{
    double SHOOT_BLOCKED_RANGE = 3;
    int n = 0;
    const BallVisionT& ball = pVision->Ball();
    bool shootBlocked = false;
    CGeoPoint projectionPoint;
    double w = Param::Field::GOAL_WIDTH;
    double k_m = SHOOT_BLOCKED_RANGE;//ball2theirbest / 15;
    double goal[3] = { -0.4 * w,0,0.4 * w };
    CGeoPoint TheShootPoint;
    double opp2LineDist_max = 0;
    int best_i = -1;
    // int theirGoalie = pVision->TheirGoalie();
    for (int i = 0; i < 3; i++) {
        double shootDir = (CGeoPoint(Param::Field::PITCH_LENGTH / 2.0, goal[i]) - startPoint).dir();
        CGeoLine ball2ourGoal = CGeoLine(startPoint, shootDir);
        double opp2LineDist = 1000;
        n = 0;
        while (n <= Param::Field::MAX_PLAYER) {
            if (!pVision->TheirPlayer(n).Valid()) { n++; continue; }
            //ȡ��ֱ�������С
            projectionPoint = ball2ourGoal.projection(pVision->TheirPlayer(n).Pos());
            if (opp2LineDist > (projectionPoint - pVision->TheirPlayer(n).Pos()).mod() && projectionPoint.x() >= startPoint.x()) {
                opp2LineDist = (projectionPoint - pVision->TheirPlayer(n).Pos()).mod();
            }
            n++;
        }
        //ȡ�����������
        if (opp2LineDist_max < opp2LineDist) {
            opp2LineDist_max = opp2LineDist;
            best_i = i;
        }
    }
    double shootDir = (CGeoPoint(Param::Field::PITCH_LENGTH / 2.0, goal[best_i]) - startPoint).dir();
    TheShootPoint = CGeoPoint(Param::Field::PITCH_LENGTH / 2.0, goal[best_i]);

    return TheShootPoint;
}
bool CPenaltyKickV2::isTheLineBlocked(const CVisionModule* pVision, CGeoPoint startPoint, CGeoPoint targetPoint) {
    /*����·�����Ƿ�ᱻ�����赲*/
    int n = 0;
    const BallVisionT& ball = pVision->Ball();
    bool passBlocked = false;
    double passDir = (targetPoint - startPoint).dir();
    CGeoLine start2Target = CGeoLine(startPoint, passDir);
    CGeoPoint projectionPoint;
    double k_m = 4;
    double opp2LineDist = 1000;
    while (n <= Param::Field::MAX_PLAYER) {
        if (!pVision->TheirPlayer(n).Valid()) { n++; continue; }
        projectionPoint = start2Target.projection(pVision->TheirPlayer(n).Pos());
        if (opp2LineDist > (projectionPoint - pVision->TheirPlayer(n).Pos()).mod() && projectionPoint.x() < Param::Field::PITCH_LENGTH / 2.0 && projectionPoint.x() > startPoint.x()) {
            opp2LineDist = (projectionPoint - pVision->TheirPlayer(n).Pos()).mod();
            if (opp2LineDist < k_m * Param::Vehicle::V2::PLAYER_SIZE) {
                passBlocked = true;
                break;
            }
        }
        n++;
    }
    return passBlocked;
}

//bug here: opp2Ball
bool CPenaltyKickV2::Me2OppTooclose(const CVisionModule* pVision, const int vecNumber) { //�Ƿ�̫����
    const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
    const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
    const BallVisionT& ball = pVision->Ball();
    CVector me2Ball = ball.Pos() - me.Pos();
    CVector me2Opp = opp.Pos() - me.Pos();
    CVector Opp2Ball = ball.Pos() - opp.Pos();
    if (abs(me2Ball.mod()) < 80 && abs(Opp2Ball.mod()) < 200) {
        if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(450, 450), "TOO CLOSE with ball", COLOR_ORANGE);
        return true;
    }
    return false;
}

bool CPenaltyKickV2::isDirOK(const CVisionModule* pVision, int vecNumber, double targetDir, int ShootOrPass) {
    double ShootPrecision = 3.5;
    const BallVisionT& ball = pVision->Ball();
    const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
    const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
    double myDir = me.Dir();
    CVector opp2ball = ball.Pos() - opp.Pos();
    CVector ball2goal = theirCenter - ball.Pos();
    if (fabs(Utils::Normalize(me.Dir() - targetDir)) < ShootPrecision / 180.0 * Param::Math::PI)
        return true;
    return false;
}


/**********************************************************
    * Description: ״̬�л��ж��ຯ��������״̬ת��֮����ж�
    * Author: ̷���
    * Created Date: 2022/10/10
***********************************************************/
bool CPenaltyKickV2::tendToShoot(const CVisionModule* pVision, int vecNumber) {
    /*�ж������ܷ�����*/
    // NEEDMODIFY
    int n = 0;
    int best_n = 0;
    const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
    const BallVisionT& ball = pVision->Ball();
    bool shootBlocked = false;

    double kickDir = KickDirection::Instance()->getPointShootDir(pVision, pVision->OurPlayer(vecNumber).Pos());
    if (fabs(kickDir - 1000.0) < 10) return false;
    CGeoLine ball2ourGoal = CGeoLine(me.Pos(), kickDir);
    CGeoPoint projectionPoint;
    double k_m = WantToLessShoot;
    double opp2LineDist = 1000;
    while (n <= Param::Field::MAX_PLAYER) {
        if (!pVision->TheirPlayer(n).Valid()) { n++; continue; }
        projectionPoint = ball2ourGoal.projection(pVision->TheirPlayer(n).Pos());
        if (opp2LineDist > (projectionPoint - pVision->TheirPlayer(n).Pos()).mod() && projectionPoint.x() >= me.X()) {
            opp2LineDist = (projectionPoint - pVision->TheirPlayer(n).Pos()).mod();
            best_n = n;
            if (opp2LineDist < k_m * Param::Vehicle::V2::PLAYER_SIZE) {
                shootBlocked = true;
                break;
            }
        }
        n++;
    }
    KickDirection::Instance()->GenerateShootDir(vecNumber, pVision->OurPlayer(vecNumber).Pos());

    const PlayerVisionT& opp = pVision->TheirPlayer(best_n);
    double me2theirbest = (me.Pos() - opp.Pos()).mod();
    double me2goal = (me.Pos() - theirCenter).mod();
    bool kickValid = KickDirection::Instance()->getIsKickValid();

    /*
    const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);


    double opp2goal = Param::Field::PITCH_LENGTH / 2.0 - opp.Pos().x();
    if (opp2goal < 10) shootBlocked = true;
    */

    if (shootBlocked) return false;
    else return kickValid;
}


/**********************************************************
    * Description: �����ຯ�������о���ʵ��
    * Author: ̷���
    * Created Date: 2022/10/10
***********************************************************/
double CPenaltyKickV2::JustChipDir(const CVisionModule* pVision, int vecNumber) {
    const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
    CGeoPoint passPointLeft = SupportPos2022::Instance()->getSupportPos(pVision).getLeftSupportPos();
    CGeoPoint passPointRight = SupportPos2022::Instance()->getSupportPos(pVision).getRightSupportPos();
    double LeftShootDir = KickDirection::Instance()->getPointShootDir(pVision, passPointLeft);
    double RightShootDir = KickDirection::Instance()->getPointShootDir(pVision, passPointRight);
    double passDir = 0;
    int pass = 0;
    double me2left = (me.Pos() - passPointLeft).mod();
    double me2right = (me.Pos() - passPointRight).mod();
    if (me2left > me2right) {
        pass = 1;
        passDir = (passPointLeft - me.Pos()).dir();
    }
    if (me2left < me2right) {
        pass = 2;
        passDir = (passPointRight - me.Pos()).dir();
    }
    if (abs(me2left - me2right) < 50) {
        if (last_pass == 1) {
            pass = 1;
            passDir = (passPointLeft - me.Pos()).dir();
        }
        if (last_pass == 2) {
            pass = 2;
            passDir = (passPointRight - me.Pos()).dir();
        }
        if (last_pass == 0) {
            passDir = (passPointLeft - me.Pos()).dir();
            pass = 1;
        }
    }
    if (me2left < 200) {
        pass = 2;
        passDir = (passPointRight - me.Pos()).dir();
    }
    if (me2right < 200) {
        pass = 1;
        passDir = (passPointLeft - me.Pos()).dir();
    }
    last_pass = pass;
    return passDir;
}

CGeoPoint CPenaltyKickV2::GenerateBreakShootPoint(const CVisionModule* pVision, int vecNumber) {
    const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
    //CGeoPoint ShootPoint =KickDirection::Instance()->GetTheShootPoint(pVision, me.Pos());
    CGeoPoint ShootPoint = getbetterPointShoot(pVision, me.Pos());
    ShootPoint.setY(ShootPoint.y() - me.VelY());
    if (me.Y() < -120)ShootPoint.setY(ShootPoint.y() + me.VelX() * 2 + me.Y() * 0.75);
    else if (me.Y() > 120)ShootPoint.setY(ShootPoint.y() - me.VelX() * 2 - me.Y() * 0.75);
    //double MeVel = me.VelY()
    if (ShootPoint.y() < -90)ShootPoint.setY(-90);
    if (ShootPoint.y() > 90)ShootPoint.setY(90);

    return ShootPoint;
}

/**********************************************************
    * Description: for normalpush
    * Author: jlc
    * Created Date: 2022/11/18
***********************************************************/

double CPenaltyKickV2::generateNormalPushDir(const CVisionModule* pVision, const int vecNumber) {
    const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
    const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
    const BallVisionT& ball = pVision->Ball();
    double faceDir = 0.0;
    if (!opp.Valid()) {
        KickDirection::Instance()->GenerateShootDir(vecNumber, pVision->OurPlayer(vecNumber).Pos());
        faceDir = KickDirection::Instance()->getRealKickDir();
        return faceDir;
    }
    if (abs(ball.Pos().y()) < Param::Field::PITCH_WIDTH / 2 * 0.4 || me.Pos().x() < -50 /* || !checkBallFront(pVision, Param::Math::PI / 4.0)*/) {
        //cout << "there there there" << endl;
        KickDirection::Instance()->GenerateShootDir(vecNumber, pVision->OurPlayer(vecNumber).Pos());
        faceDir = KickDirection::Instance()->getRealKickDir();
        return faceDir;
    }
    else if (abs(ball.Pos().y()) > Param::Field::PITCH_WIDTH / 2 * 0.70) {
        //cout << "here here here" << endl;
        faceDir = opp.Dir() + Param::Math::PI;
        return faceDir;
    }
    else {
        KickDirection::Instance()->GenerateShootDir(vecNumber, pVision->OurPlayer(vecNumber).Pos());
        double kickDir = KickDirection::Instance()->getRealKickDir();
        double maxDir = Utils::Normalize(opp.Dir() + Param::Math::PI);
        double diffDir = Utils::Normalize(kickDir - maxDir);
        if (abs(diffDir) < Param::Math::PI / 15 || (kickDir > 0 && maxDir > kickDir) || (kickDir < 0 && maxDir < kickDir)) {
            return kickDir;
        }
        else {
            faceDir = Utils::Normalize(kickDir - diffDir * (3.33 * (abs(ball.Pos().y()) / (Param::Field::PITCH_WIDTH / 2)) - 1.33));
            return faceDir;
        }
    }
}
double getPointPUshDir(const CVisionModule* pVision, CGeoPoint startPoint)
{
    double SHOOT_BLOCKED_RANGE = 3;
    int n = 0;
    const BallVisionT& ball = pVision->Ball();
    bool shootBlocked = false;
    CGeoPoint projectionPoint;
    double w = Param::Field::GOAL_WIDTH;
    double k_m = SHOOT_BLOCKED_RANGE;//ball2theirbest / 15;
    double goal[3] = { -0.4 * w,0,0.4 * w };
    CGeoPoint TheShootPoint;


    double shootDir = (CGeoPoint(Param::Field::PITCH_LENGTH / 2.0, goal[1]) - startPoint).dir();

    return shootDir;
}
CGeoPoint CPenaltyKickV2::generateNormalPushPoint(const CVisionModule* pVision, const int vecNumber) {
    const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
    const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
    const BallVisionT& ball = pVision->Ball();
    double faceDir = 0.0;

    const double DirEpsilon = Param::Math::PI / 180 * 5;
    const double VectorDist = 60;
    const double ThresholdForOpp = 60;



    CGeoPoint FinalTarget = CGeoPoint(-999, -999);
    double Dist = -1e9;
    for (int step = -7; step <= 7; ++step) {
        const CGeoPoint target = me.Pos() + Utils::Polar2Vector(VectorDist, Param::Math::PI / 180 * 5 * step);

        double passDir = (target - me.Pos()).dir();
        CGeoLine start2Target = CGeoLine(me.Pos(), passDir);
        CGeoPoint projectionPoint = start2Target.projection(pVision->TheirPlayer(opponentID).Pos());
        double opp2LineDist = 1000;

        double r = (projectionPoint - me.Pos()).x() * (projectionPoint - target).x() + (projectionPoint - me.Pos()).y() * (projectionPoint - target).y();
        if (opp2LineDist > (projectionPoint - pVision->TheirPlayer(opponentID).Pos()).mod() && r < 0) { // projectionPoint.x() < Param::Field::PITCH_LENGTH / 2.0 && projectionPoint.x() > startPoint.x()) {
            opp2LineDist = (projectionPoint - pVision->TheirPlayer(opponentID).Pos()).mod();
        }

        if (opp2LineDist > Dist) FinalTarget = target, Dist = opp2LineDist;
        //GDebugEngine::Instance()->gui_debug_x(target, COLOR_BLUE);
    }
    if (Dist > ThresholdForOpp) {
        Dist = 1e9;
        for (int step = -10; step <= 10; ++step) {
            const CGeoPoint target = me.Pos() + Utils::Polar2Vector(VectorDist, Param::Math::PI / 180 * 5 * step);

            double passDir = (target - me.Pos()).dir();
            CGeoLine start2Target = CGeoLine(me.Pos(), passDir);
            CGeoPoint projectionPoint = start2Target.projection(pVision->TheirPlayer(opponentID).Pos());
            double opp2LineDist = 1000;

            double r = (projectionPoint - me.Pos()).x() * (projectionPoint - target).x() + (projectionPoint - me.Pos()).y() * (projectionPoint - target).y();
            if (opp2LineDist > (projectionPoint - pVision->TheirPlayer(opponentID).Pos()).mod() && r < 0) { // projectionPoint.x() < Param::Field::PITCH_LENGTH / 2.0 && projectionPoint.x() > startPoint.x()) {
                opp2LineDist = (projectionPoint - pVision->TheirPlayer(opponentID).Pos()).mod();
            }
            if (opp2LineDist > Dist) FinalTarget = target, Dist = opp2LineDist;

            if (Dist < 60) continue;

            const double SingleDist = (target - theirCenter).mod();
            if (SingleDist < Dist) FinalTarget = target, Dist = SingleDist;
            //GDebugEngine::Instance()->gui_debug_x(target, COLOR_BLUE);
        }
    }
    //GDebugEngine::Instance()->gui_debug_x(FinalTarget, COLOR_BLUE);
    return FinalTarget;
}

/*bool CPenaltyKickV2::isMePassedOpp(const CVisionModule* pVision, const int vecNumber) {
    const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
    const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
    const BallVisionT& ball = pVision->Ball();

    CVector me2opp = opp.Pos() - me.Pos();
    CVector opp2Ball = ball.Pos() - opp.Pos();
    CVector ball2Opp = opp.Pos() - ball.Pos();
    CVector me2Ball = ball.Pos() - me.Pos();
    bool meDirControlBall = abs(Utils::Normalize(me2Ball.dir() - me.Dir())) < Param::Math::PI / 4;
    bool meDistControlBall = me2Ball.mod() < Param::Vehicle::V2::PLAYER_FRONT_TO_CENTER + 10.5;
    bool mePassOpp_1 = abs(Utils::Normalize(me2Ball.dir() - me2opp.dir())) > Param::Math::PI / 2.5;
    bool mePassOpp_2 = abs(Utils::Normalize(opp.Dir() - ball2Opp.dir())) < Param::Math::PI / 3.0;
    if (meDistControlBall && meDistControlBall && (mePassOpp_1 || mePassOpp_2) || me2opp.mod() > 25) {
        // GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, 0), "PASS!!!!!!!!!!!", COLOR_WHITE);
        return true;
    }
    else {
        return false;
    }
}*/

CPlayerCommand* CPenaltyKickV2::execute(const CVisionModule* pVision) {
    if (subTask()) {
        return subTask()->execute(pVision);
    }
    if (_directCommand) {
        return _directCommand;
    }
    return 0;
}

