/************************************************************************/
/*		modified by jlc
        22.7.10
        �����˼·��lua�����ܵ��жϾ���ֱ����penaltykick �� break + chase
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
#include <BestPlayer.h>
#include "KickDirection.h"
#include <GDebugEngine.h>
#include <iostream>
#include <BestPlayer.h>
#include <TaskMediator.h>
#include "Global.h"


CPenaltyKickV2::CPenaltyKickV2()
{
    KICK_DIST = paramManager->KICK_DIST - 250;  /*��������Χ Խ��Խ��������*/
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
    int NumofPlayerInFrontfiled = 0;
    bool isMeHasBall = false;
    bool isMechHasBall = infraredOn >= 2;
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
    double BallToOurGoal = (ball.Pos() - ourGoal).mod();
    CVector me2goal = theirCenter - me.Pos();
    bool isOppHasBall = checkOppHasBall(pVision);
    const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
    const CVector opp2ball = ball.Pos() - opp.Pos();
    double ball2oppDist = opp2ball.mod();
    double ball2meDist = (ball.Pos() - me.Pos()).mod();
    double Me2Receiver = (me.Pos() - pVision->OurPlayer(tandemNum).Pos()).mod();
    double me2BestOppDist = CVector(pVision->TheirPlayer(opponentID).Pos() - me.Pos()).mod();
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
    for (int i = 0; i < Param::Field::MAX_PLAYER; i++) {
        if (pVision->OurPlayer(i).Valid() && i != GoalieNumber)
            if (pVision->OurPlayer(i).Pos().x() > Param::Field::PITCH_LENGTH / 10)
                NumofPlayerInFrontfiled++;
    }

    /**********************************************************
    * Description: ״̬����
    * Author: ̷���
    * Created Date: 2022/10/10
    ***********************************************************/
    switch (_state) {
    case BEGIN:
        _state = GET;
        break;
    case GET:
        if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -400), "Push GET", COLOR_YELLOW);
        if (meHasBall > 5) {
            if (me2goal.mod() < KICK_DIST) {
                if (tendToShoot(pVision, _executor)) {
                    _state = KICK; break;
                }
                else if (Me2OppTooclose(pVision, _executor)) {
                    _state = BREAKSHOOT; break;
                }
                else {
                    _state = BREAKSHOOT; break;
                }
            }
            else {
                if (fabs(opp.VelX()) > 300 || Me2OppTooclose(pVision, _executor)) {
                    _state = CHIP; break;
                }
                else if (me2goal.mod() > KICK_DIST + 100){
                    _state = NORMAL_PUSH; break;
                }
                else {
                    _state = BREAKSHOOT; break;
                }
            }
        }
        else { _state = GET; break; }
    case CHIP:
        if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -400), "Push CHIP", COLOR_YELLOW);
        if (meLoseBall > 15 && ball2meDist > 10) _state = GET;
        break;
    case KICK:
        if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -400), "Push KICK", COLOR_YELLOW);
        if (meLoseBall > 15 && ball2meDist > 10) _state = GET;
        break;
    case BREAKSHOOT:
        if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -400), "Push BREAKSHOOT", COLOR_YELLOW);
        if (meLoseBall > 18 && ball2meDist > 10) _state = GET;
        break;
    case NORMAL_PUSH:
        normalPushCnt++;
        if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(200, 400), "GETtoNORMALPUSH", COLOR_YELLOW);
        if (meLoseBall > 10 * 1.25) {
            if (Advance_DEBUG_ENGINE) { cout << "normalpush --> get" << endl; }
            _state = GET;
        }
        else if (me2goal.mod() < KICK_DIST || Me2OppTooclose(pVision, _executor)) {
            if (Advance_DEBUG_ENGINE) { cout << "normalpush-> breakshoot" << endl; }
            if (tendToShoot(pVision, _executor)) {
                _state = KICK;
                break;
            }
            else{
                _state = BREAKSHOOT;
                break;
            }
        }
        else if (abs(Utils::Normalize(me.Dir() - me2goal.dir())) < Param::Math::PI / 36) {
            if (Advance_DEBUG_ENGINE) { cout << "normalpush --> light kick" << endl; }
            _state = LIGHT_KICK;
        }

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
                _state = KICK;
                break;
            }
            else{
                _state = BREAKSHOOT;
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
            setSubTask(PlayerRole::makeItNoneTrajGetBall(_executor, KickorPassDir, CVector(0, 0), ShootNotNeedDribble, GetBallBias));
        }
        break;
    case KICK:   // ����
        if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(200, -400), "KICK", COLOR_YELLOW);
        KickorPassDir = KickDirection::Instance()->getPointShootDir(pVision, pVision->OurPlayer(_executor).Pos());
        KickStatus::Instance()->clearAll();
        if (Utils::InTheirPenaltyArea(ball.Pos(), 0)) {
            /*������ڶԷ�����*/
            if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -400), "ball in their PEN", COLOR_ORANGE);
            KickStatus::Instance()->setKick(_executor, KICKPOWER);
            setSubTask(PlayerRole::makeItShootBallV2(_executor, KickorPassDir, ShootNotNeedDribble));
        }
        else {
            /*����KICK�׶�  ��Ҫ�����Ƿ����Ѿ�ת��ɹ�  �˴���δ�걸���ܴ���BUG*/
            if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -400), "Let Kick", COLOR_ORANGE);
            if (isDirOK(pVision, _executor, KickorPassDir, 1)) {
                if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -350), "Kick isDirOK", COLOR_ORANGE);
                KickStatus::Instance()->setKick(_executor, KICKPOWER);
                setSubTask(PlayerRole::makeItShootBallV2(_executor, KickorPassDir, ShootNotNeedDribble));
            }
            else {
                //setSubTask(PlayerRole::makeItGoAndTurnKickV4(_executor, kickDir));
                setSubTask(PlayerRole::makeItNoneTrajGetBall(_executor, KickorPassDir, CVector(0, 0), ShootNotNeedDribble, GetBallBias));
                if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -350), "Kick is NOT DirOK ", COLOR_ORANGE);
            }
        }
        break;
    case CHIP:
        if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(200, -400), "Let CHIP", COLOR_YELLOW);
        if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(200, -400), "KICK", COLOR_YELLOW);
        KickorPassDir = KickDirection::Instance()->getPointShootDir(pVision, pVision->OurPlayer(_executor).Pos());
        KickStatus::Instance()->clearAll();
            /*����KICK�׶�  ��Ҫ�����Ƿ����Ѿ�ת��ɹ�  �˴���δ�걸���ܴ���BUG*/
        if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -400), "Let Chip", COLOR_ORANGE);
        if (isDirOK(pVision, _executor, KickorPassDir, 1)) {
            if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -350), "Chip isDirOK", COLOR_ORANGE);
            KickStatus::Instance()->setChipKick(_executor, me2goal.mod() - 18);
        }
        else {
            //setSubTask(PlayerRole::makeItGoAndTurnKickV4(_executor, kickDir));
            setSubTask(PlayerRole::makeItNoneTrajGetBall(_executor, KickorPassDir, CVector(0, 0), ShootNotNeedDribble, GetBallBias));
            if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(500, -350), "Chip is NOT DirOK ", COLOR_ORANGE);
        }
        break;
    case BREAKSHOOT:
        if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(200, -400), "BREAKSHOOT", COLOR_YELLOW);
        KickStatus::Instance()->clearAll();
        ShootPoint = GenerateBreakShootPoint(pVision, _executor);
        setSubTask(PlayerRole::makeItBreak(_executor, ShootPoint, 1));
        break;
    case NORMAL_PUSH:
        KickDirection::Instance()->GenerateShootDir(_executor, pVision->OurPlayer(_executor).Pos());
        if (pVision->Ball().X() <= 0) { //our half-field - 50(radius of the middle circle)
            KickorPassDir = KickDirection::Instance()->getRealKickDir();
        }
        else {
            KickorPassDir = generateNormalPushDir(pVision, _executor);
        }
        setSubTask(PlayerRole::makeItNoneTrajGetBall(_executor, KickorPassDir, CVector(0, 0), ShootNotNeedDribble, GetBallBias));
        if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(0, -350), "finish in NorPush", COLOR_ORANGE);
        break;
    case LIGHT_KICK:
        KickStatus::Instance()->clearAll();
        if(pVision->Ball().X() <= 100)
            KickStatus::Instance()->setKick(_executor, 200); // kick lightly
        else
            KickStatus::Instance()->setKick(_executor, 80);
        break;
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

bool CPenaltyKickV2::checkOppHasBall(const CVisionModule* pVision) {
    int _executor = task().executor;
    const BallVisionT& ball = pVision->Ball();
    const PlayerVisionT& me = pVision->OurPlayer(_executor);
    const CVector self2ball = ball.Pos() - me.Pos();
    opponentID = 0;

    const CBestPlayer::PlayerList& oppList = BestPlayer::Instance()->theirFastestPlayerToBallList();
    if (oppList.size() < 1)return false;
    else opponentID = oppList[0].num;
    if (!pVision->TheirPlayer(opponentID).Valid()) {
        opponentID = getTheirMostClosetoPosPlayerNum(pVision, pVision->Ball().Pos());
    }
    const PlayerVisionT& opponent = pVision->TheirPlayer(opponentID);
    if (Advance_DEBUG_ENGINE)GDebugEngine::Instance()->gui_debug_msg(opponent.Pos(), "Best Opp!", COLOR_WHITE);
    CVector opponent2ball = ball.Pos() - opponent.Pos();
    CVector opp2ourGoal = ourGoal - opponent.Pos();
    double opponent2ball_diff = fabs(Utils::Normalize(opponent2ball.dir() - opponent.Dir()));
    double judgeDist = OPP_HAS_BALL_DIST;
    if (opponent2ball.mod() < judgeDist && opponent2ball_diff < Param::Math::PI * 70 / 180)
        return true; // take opponent's direction into consideration.If direction not towards the ball,ignore it
    else
        return false;
}

int CPenaltyKickV2::getTheirMostClosetoPosPlayerNum(const CVisionModule* pVision, CGeoPoint pos) {
    double dist = 1000;
    int num = 0;
    for (int i = 0; i < Param::Field::MAX_PLAYER; i++) {
        if (pVision->TheirPlayer(i).Valid()) {
            if (pVision->TheirPlayer(i).Pos().dist(pos) < dist) {
                dist = pVision->TheirPlayer(i).Pos().dist(pos);
                num = i;
            }
        }
    }
    return num;
}

bool CPenaltyKickV2::checkBallFront(const CVisionModule* pVision, double angle) {
    /*�ж����Ƿ��ڵ���ǰ�� ���ڼн�Ҫ��*/
    const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
    const BallVisionT& ball = pVision->Ball();
    CVector opp2ball = ball.Pos() - opp.Pos();
    bool ballDirFrontOpp = abs(Utils::Normalize(opp.Dir() - opp2ball.dir())) < angle;
    bool ballDistFrontOpp = opp2ball.mod() < OPP_HAS_BALL_DIST + 10;
    //GDebugEngine::Instance()->gui_debug_line(opp.Pos(),opp.Pos() + Utils::Polar2Vector(200 , 0),COLOR_BLACK);
    bool isBallFrontOpp = ballDirFrontOpp && ballDistFrontOpp;
    return isBallFrontOpp;
}

bool CPenaltyKickV2::isPassBalltoMe(const CVisionModule* pVision, int vecNumber) {
    const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
    const BallVisionT& ball = pVision->Ball();
    CVector ball2me = me.Pos() - ball.Pos();
    double diff_ballMoving2Me = Utils::Normalize(ball2me.dir() - ball.Vel().dir());
    if (ball.Valid() && abs(diff_ballMoving2Me) < Param::Math::PI / 12 && ball.Vel().mod() > BalltoMeVel) {//
        return true;
    }
    return false;
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

bool CPenaltyKickV2::IsOurNearHere(const CVisionModule* pVision, CGeoPoint checkPoint, const int vecNumber) {
    const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
    const BallVisionT& ball = pVision->Ball();

    for (int i = 0; i < 8; i++) {
        if (vecNumber == i)continue;
        if (pVision->OurPlayer(i).Valid()) {
            const PlayerVisionT& me = pVision->OurPlayer(i);
            if ((me.Pos() - checkPoint).mod() < 150) {
                return true;
            }
        }
    }
    return false;
}

bool CPenaltyKickV2::Me2OppTooclose(const CVisionModule* pVision, const int vecNumber) { //�Ƿ�̫����
    const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
    const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
    const BallVisionT& ball = pVision->Ball();
    CVector me2Ball = ball.Pos() - me.Pos();
    CVector me2Opp = opp.Pos() - me.Pos();
    CVector Opp2Ball = ball.Pos() - opp.Pos();
    if (abs(me2Ball.mod()) < 80 && abs(Opp2Ball.mod()) < 130) {
        if (Advance_DEBUG_ENGINE) GDebugEngine::Instance()->gui_debug_msg(CGeoPoint(450, 450), "TOO CLOSE with ball", COLOR_ORANGE);
        return true;
    }
    return false;
}

bool CPenaltyKickV2::isDirOK(const CVisionModule* pVision, int vecNumber, double targetDir, int ShootOrPass) {
    double ShootPrecision = SHOOT_PRECISION;
    const BallVisionT& ball = pVision->Ball();
    const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
    const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
    double myDir = me.Dir();
    CVector opp2ball = ball.Pos() - opp.Pos();
    CVector ball2goal = theirCenter - ball.Pos();
    if (!ShootOrPass) ShootPrecision = ShootPrecision * 0.85;
    last_target_dir = targetDir;
    //printf("%d %.5lf %.5lf\n", ShootOrPass, myDir, targetDir);
    if (abs(targetDir - last_target_dir) > 0.3 * Param::Math::PI / SHOOT_PRECISION) {
        last_dir_deviation = 100;  //���ýǶȲ�
    }
    if (Me2OppTooclose(pVision, vecNumber)) {
        /*̫���� ����*/
        if (abs(myDir - targetDir) < Param::Math::PI / SHOOT_PRECISION) {
            last_dir_deviation = 100;
            return true;
        }
    }
    if ((ShootOrPass && ball2goal.mod() < 250) || (opp2ball.mod() < 200)) {
        /*��������������Ҿ��������㹻�� ����Ҫ����ĵ���
          ����������ұȽϽ��� �ٵ������޷�������*/
        if (abs(myDir - targetDir) < 0.35 * Param::Math::PI / SHOOT_PRECISION) {
            last_dir_deviation = 100;
            return true;
        }
    }
    if (abs(myDir - targetDir) > 0.25 * Param::Math::PI / SHOOT_PRECISION) {
        /*����Ƕȹ��� Ӧ��Ϊfalse*/
        last_dir_deviation = myDir - targetDir;
        return false;
    }
    else if ((abs(myDir - targetDir) > abs(last_dir_deviation) || (myDir - targetDir) * last_dir_deviation <= 0)) {
        /*�������ϴνǶȲ�û�еõ���Ч����*/
        if (abs(myDir - targetDir) < 0.2 * Param::Math::PI / SHOOT_PRECISION) {
            /*�����Զ��Կ��Խ�����*/
            last_dir_deviation = 100;
            return true;
        }
    }
    else if (abs(myDir - targetDir) < 0.15 * Param::Math::PI / SHOOT_PRECISION) {
        /*������Ȼ�ڵ��� ���ǿ������������ķ�Χ������*/
        last_dir_deviation = 100;
        return true;
    }
    last_dir_deviation = myDir - targetDir;
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

int CPenaltyKickV2::CanSupportKick(const CVisionModule* pVision, int vecNumber) {
    CGeoPoint passPointLeft = SupportPos2022::Instance()->getSupportPos(pVision).getLeftSupportPos();
    CGeoPoint passPointRight = SupportPos2022::Instance()->getSupportPos(pVision).getRightSupportPos();
    const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
    bool isBlockLeft = isTheLineBlocked(pVision, me.Pos(), passPointLeft);
    bool isBlockRight = isTheLineBlocked(pVision, me.Pos(), passPointRight);
    double LeftShootDir = KickDirection::Instance()->getPointShootDir(pVision, passPointLeft);

    double RightShootDir = KickDirection::Instance()->getPointShootDir(pVision, passPointRight);
    double kickDir = KickDirection::Instance()->getPointShootDir(pVision, me.Pos());
    double me2left = (me.Pos() - passPointLeft).mod();
    double me2right = (me.Pos() - passPointRight).mod();
    double left2goal = (CGeoPoint(Param::Field::PITCH_LENGTH / 2.0, 0) - passPointLeft).mod();
    double right2goal = (CGeoPoint(Param::Field::PITCH_LENGTH / 2.0, 0) - passPointRight).mod();

    if (!isBlockLeft && LeftShootDir != 1000 && me2left < CanPassToWingDist && left2goal < CanWingShootDist)
        return 1;
    if (!isBlockRight && RightShootDir != 1000 && me2right < CanPassToWingDist && right2goal < CanWingShootDist)
        return 1;
    return 0;
}

int CPenaltyKickV2::toChipOrToFlat(const CVisionModule* pVision, int vecNumber) {
    /*����0��ʾ��ҪCHIP������1��ʾ��ҪFLAT*/
    CGeoPoint passPointLeft = SupportPos2022::Instance()->getSupportPos(pVision).getLeftSupportPos();
    CGeoPoint passPointRight = SupportPos2022::Instance()->getSupportPos(pVision).getRightSupportPos();
    const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
    bool isBlockLeft = isTheLineBlocked(pVision, me.Pos(), passPointLeft);
    bool isBlockRight = isTheLineBlocked(pVision, me.Pos(), passPointRight);
    double LeftShootDir = KickDirection::Instance()->getPointShootDir(pVision, passPointLeft);
    double RightShootDir = KickDirection::Instance()->getPointShootDir(pVision, passPointRight);
    if (isBlockLeft && isBlockRight) return 0;
    else return 1;
}

/**********************************************************
    * Description: �����ຯ����������GET��ʹ��
    * Author: ̷���
    * Created Date: 2022/10/10
***********************************************************/
bool CPenaltyKickV2::isOppFaceOurDoor(const CVisionModule* pVision, double angle) {
    //�ж��Ƿ������opp�ܽ���opp�����������źܽ�
    const BallVisionT& ball = pVision->Ball();
    const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
    double opp2BallDist = (opp.Pos() - ball.Pos()).mod();
    bool isBallNearOpp = opp2BallDist < OPP_HAS_BALL_DIST;
    double judgeAngle = abs(Utils::Normalize((opp.Dir() - CVector(CGeoPoint(-Param::Field::PITCH_LENGTH / 2.0, 0) - opp.Pos()).dir())));
    bool isFaceOurDoor = judgeAngle < angle || judgeAngle == angle;
    return isFaceOurDoor && isBallNearOpp;
}

bool CPenaltyKickV2::checkTheyCanShoot(const CVisionModule* pVision, int vecNumber) {
    const BallVisionT& ball = pVision->Ball();
    const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
    const PlayerVisionT& opp = pVision->TheirPlayer(opponentID);
    CVector opp2ball = ball.Pos() - opp.Pos();
    CVector me2ball = ball.Pos() - me.Pos();
    double opp2ballDist = opp2ball.mod();
    double me2ballDist = me2ball.mod();
    if (isOppFaceOurDoor(pVision, Param::Math::PI / 6.0) && checkBallFront(pVision, Param::Math::PI / 6.0) && ((me2ballDist - opp2ballDist) > -5) && (opp.Pos() - ourGoal).mod() < Param::Field::GOAL_WIDTH / 2) {
        return true;
    }
    else { return false; }
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
double CPenaltyKickV2::flatPassDir(const CVisionModule* pVision, int vecNumber) {
    const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
    /*
      CGeoPoint passPointLeft = SupportPos2022::Instance()->getSupportPos(pVision).getLeftSupportPos();
      CGeoPoint passPointRight = SupportPos2022::Instance()->getSupportPos(pVision).getRightSupportPos();
    */
    CGeoPoint passPointLeft = SupportPoint[0];
    CGeoPoint passPointRight = SupportPoint[1];

    bool isOursNearLeft = IsOurNearHere(pVision, passPointLeft, vecNumber);
    bool isOursNearRight = IsOurNearHere(pVision, passPointRight, vecNumber);

    bool isBlockLeft = isTheLineBlocked(pVision, me.Pos(), passPointLeft);
    bool isBlockRight = isTheLineBlocked(pVision, me.Pos(), passPointRight);
    double LeftShootDir = KickDirection::Instance()->getPointShootDir(pVision, passPointLeft);
    double RightShootDir = KickDirection::Instance()->getPointShootDir(pVision, passPointRight);
    double passDir = 0;
    int pass = 0;

    /*�ж��Ƿ���û�˵����*/
    if (!isOursNearLeft) {
        passDir = (passPointRight - me.Pos()).dir();
        last_pass = pass;
        return passDir;
    }
    if (!isOursNearRight) {
        passDir = (passPointLeft - me.Pos()).dir();
        last_pass = pass;
        return passDir;
    }

    if (!isBlockLeft && isBlockRight) {
        pass = 1;
        passDir = (passPointLeft - me.Pos()).dir();
    }
    if (isBlockLeft && !isBlockRight) {
        pass = 2;
        passDir = (passPointRight - me.Pos()).dir();
    }
    if (!isBlockLeft && !isBlockRight) {
        if (LeftShootDir != 1000 && RightShootDir == 1000) {
            pass = 1;
            passDir = (passPointLeft - me.Pos()).dir();
        }
        if (LeftShootDir == 1000 && RightShootDir != 1000) {
            pass = 2;
            passDir = (passPointRight - me.Pos()).dir();
        }
        if (LeftShootDir != 1000 && RightShootDir != 1000) {
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
            if (abs(me2left - me2right) < 50) {/*����ȶ���*/
                if (last_pass == 1) {
                    passDir = (passPointLeft - me.Pos()).dir();
                    pass = 1;
                }
                if (last_pass == 2) {
                    passDir = (passPointRight - me.Pos()).dir();
                    pass = 2;
                }
                if (last_pass == 0) {
                    passDir = (passPointLeft - me.Pos()).dir();
                    pass = 1;
                }

            }

        }
    }

    if (isBlockLeft && isBlockRight) {
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
    }
    last_pass = pass;
    return passDir;
}

CGeoPoint CPenaltyKickV2::GenerateBreakShootPoint(const CVisionModule* pVision, int vecNumber) {
    const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
    CGeoPoint ShootPoint = KickDirection::Instance()->GetTheShootPoint(pVision, me.Pos());
    ShootPoint.setY(ShootPoint.y() - me.VelY()*3);
    if(me.Y() < -120)ShootPoint.setY(ShootPoint.y() + me.VelX()*2 + me.Y()*0.75);
    else if(me.Y() > 120)ShootPoint.setY(ShootPoint.y() - me.VelX()*2 - me.Y()*0.75);
    //double MeVel = me.VelY()
    if(ShootPoint.y() < -80)ShootPoint.setY(-80);
    if(ShootPoint.y() > 80)ShootPoint.setY(80);

    return ShootPoint;
}

CGeoPoint CPenaltyKickV2::GenerateBreakPassPoint(const CVisionModule* pVision, int vecNumber) {
    const PlayerVisionT& me = pVision->OurPlayer(vecNumber);
    /*
      CGeoPoint passPointLeft = SupportPos2022::Instance()->getSupportPos(pVision).getLeftSupportPos();
      CGeoPoint passPointRight = SupportPos2022::Instance()->getSupportPos(pVision).getRightSupportPos();
    */
    CGeoPoint passPointLeft = SupportPoint[0];
    CGeoPoint passPointRight = SupportPoint[1];
    bool isOursNearLeft = IsOurNearHere(pVision, passPointLeft, vecNumber);
    bool isOursNearRight = IsOurNearHere(pVision, passPointRight, vecNumber);

    bool isBlockLeft = isTheLineBlocked(pVision, me.Pos(), passPointLeft);
    bool isBlockRight = isTheLineBlocked(pVision, me.Pos(), passPointRight);
    double LeftShootDir = KickDirection::Instance()->getPointShootDir(pVision, passPointLeft);
    double RightShootDir = KickDirection::Instance()->getPointShootDir(pVision, passPointRight);
    double passDir = 0;
    int pass = 0;

    if (!isOursNearLeft) {
        last_pass = pass;
        return passPointRight;
    }
    if (!isOursNearRight) {
        last_pass = pass;
        return passPointLeft;
    }

    if (!isBlockLeft && isBlockRight) pass = 1;
    if (isBlockLeft && !isBlockRight) pass = 2;
    if (!isBlockLeft && !isBlockRight) {
        if (LeftShootDir != 1000 && RightShootDir == 1000) pass = 1;
        if (LeftShootDir == 1000 && RightShootDir != 1000) pass = 2;
        if (LeftShootDir != 1000 && RightShootDir != 1000) {
            double me2left = (me.Pos() - passPointLeft).mod();
            double me2right = (me.Pos() - passPointRight).mod();

            if (me2left > me2right) pass = 1;
            if (me2left < me2right) pass = 2;
            if (abs(me2left - me2right) < 50) {/*����ȶ���*/
                if (last_pass == 1) pass = 1;
                if (last_pass == 2) pass = 2;
                if (last_pass == 0) pass = 1;
            }
        }
    }

    if (isBlockLeft && isBlockRight) {
        double me2left = (me.Pos() - passPointLeft).mod();
        double me2right = (me.Pos() - passPointRight).mod();
        if (me2left > me2right) pass = 1;
        if (me2left < me2right) pass = 2;
        if (abs(me2left - me2right) < 50) {
            if (last_pass == 1) pass = 1;
            if (last_pass == 2) pass = 2;
            if (last_pass == 0) pass = 1;
        }
    }
    last_pass = pass;
    if (pass == 1)return passPointLeft;
    else return passPointRight;
}

/**********************************************************
    * Description: for normalpush
    * Author: jlc
    * Created Date: 2022/11/18
***********************************************************/
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

CPlayerCommand* CPenaltyKickV2::execute(const CVisionModule* pVision) {
    if (subTask()) {
        return subTask()->execute(pVision);
    }
    if (_directCommand) {
        return _directCommand;
    }
    return 0;
}

