/**
* @file utils.h
* ���ļ�ΪһЩ���ߺ���������.
* @date $Date: 2004/05/04 04:11:49 $
* @version $Revision: 1.20 $
* @author peter@mail.ustc.edu.cn
*/
#ifndef _UTILS_H_
#define _UTILS_H_
#include <geometry.h>
#include <param.h>
#include "ParamManagerNew.h"

#define EPSILON (1.0E-10)

//#define STD_DEBUG_ENABLE

#ifdef STD_DEBUG_ENABLE
	#define STD_DEBUG_OUT(header,content) \
	std::cout << header << " : " << content << std::endl;
#else
	#define STD_DEBUG_OUT(header,content) 
#endif

#ifdef _WIN32
#define finite _finite
#define isnan _isnan
#endif

class CVisionModule;
namespace Utils{
	extern double Normalize(double angle);///<�ѽǶȹ淶����(-PI,PI]
	extern CVector Polar2Vector(double m,double angle);///<������ת����ֱ������
	extern double VectorDot(const CVector& v1, const CVector& v2);  // �������
	extern double dirDiff(const CVector& v1, const CVector& v2); // 2014/2/28 ���� �������������ļн� yys
	extern bool InBetween(const CGeoPoint& p,const CGeoPoint& p1,const CGeoPoint& p2);// �ж�p�Ƿ���p1,p2֮��
	extern bool InBetween(double v,double v1,double v2);// �ж�v�Ƿ���v1��v2֮��
	// ������Ϊ�����ķ��򻡶�, �ж��Ƿ�����v�ķ������v1��v2֮��
	// ���v��v1��v2�е�����һ���н�С��buffer, ��Ҳ��Ϊ��������.
	extern bool AngleBetween(double d, double d1, double d2, double buffer = Param::Math::PI/30); 
	inline CGeoPoint CenterOfTwoPoint(const CGeoPoint& p1,const CGeoPoint& p2){return CGeoPoint((p1.x()+p2.x())/2,(p1.y()+p2.y())/2);}
	// �ж��������������, v�ķ����Ƿ����v1��v2֮��, 
	// buffer��ʾ����, ��ʾ��v����v1,v2֮��ʱ, 
	// ���v��v1��v2�е�����һ���н�С��buffer, ��Ҳ��Ϊ��������.
	extern bool InBetween(const CVector& v, const CVector& v1, const CVector& v2, double buffer = Param::Math::PI/30);
	/*@brief	�ж�һ���������Ƿ�����������������֮��*/
	inline   bool CBetween(float v,float v1,float v2){ return (v > v1 && v < v2) || (v < v1 && v > v2); }
	/*@brief	����㵽ֱ�ߵľ���*/
	inline double pointToLineDist(const CGeoPoint& p, const CGeoLine& l) { return p.dist(l.projection(p));}
	inline double Deg2Rad(double angle){ return angle * Param::Math::PI / 180; }
	inline double Rad2Deg(double angle){ return angle * 180 / Param::Math::PI; }
	extern CGeoPoint MakeInField(const CGeoPoint& p,const double buffer = 0);///<�õ��ڳ���
	extern bool OutOfField(const CGeoPoint& p,const double buffer = 0); ///<�жϵ��Ƿ��ڳ���
	bool IsInField(const CGeoPoint p, double buffer);
    inline double FieldLeft(){ return -Param::Field::PITCH_LENGTH/2 + Param::Field::PITCH_MARGIN + Param::Field::FIELD_WALL_DIST; }
    inline double FieldRight(){ return Param::Field::PITCH_LENGTH/2 - Param::Field::PITCH_MARGIN - Param::Field::FIELD_WALL_DIST; }
    inline double FieldTop(){ return -Param::Field::PITCH_WIDTH/2   + Param::Field::PITCH_MARGIN + Param::Field::FIELD_WALL_DIST; }
    inline double FieldBottom(){ return Param::Field::PITCH_WIDTH/2 - Param::Field::PITCH_MARGIN - Param::Field::FIELD_WALL_DIST; }
	inline CGeoPoint LeftTop(){ return CGeoPoint(FieldLeft(),FieldTop()); }
	inline CGeoPoint RightBottom(){ return CGeoPoint(FieldRight(),FieldBottom()); }
	inline int Sign(double d){ return (d >= 0) ? 1 : -1; }
	extern CGeoPoint MakeOutOfOurPenaltyArea(const CGeoPoint& p,const double buffer);
	extern CGeoPoint MakeOutOfTheirPenaltyArea(const CGeoPoint& p, const double buffer);
	extern CGeoPoint MakeOutOfCircleAndInField(const CGeoPoint& center,const double radius,const CGeoPoint& p,const double buffer); // ȷ������Բ��
	extern CGeoPoint MakeOutOfCircle(const CGeoPoint& center, const double radius, const CGeoPoint& target, const double buffer, const bool isBack = false, const CGeoPoint& mePos = CGeoPoint(1e8, 1e8), const CVector adjustVec = CVector(1e8, 1e8));
	extern CGeoPoint MakeOutOfLongCircle(const CGeoPoint& seg_start, const CGeoPoint& seg_end, const double radius, const CGeoPoint& target, const double buffer, const CVector adjustVec = CVector(1e8, 1e8));
	extern CGeoPoint MakeOutOfRectangle(const CGeoPoint& recP1, const CGeoPoint& recP2, const CGeoPoint& target, const double buffer);

	extern bool InOurPenaltyArea(const CGeoPoint& p,const double buffer);
	extern bool InTheirPenaltyArea(const CGeoPoint& p,const double buffer);
	extern bool PlayerNumValid(int num);
	extern CGeoPoint GetOutSidePenaltyPos(double dir, double delta, const CGeoPoint targetPoint = CGeoPoint(-(Param::Field::PITCH_LENGTH)/2, 0));
	extern CGeoPoint GetOutTheirSidePenaltyPos(double dir, double delta, const CGeoPoint& targetPoint);
	extern CGeoPoint GetInterPos(double dir, const CGeoPoint targetPoint = CGeoPoint(-(Param::Field::PITCH_LENGTH)/2, 0));
	extern CGeoPoint GetTheirInterPos(double dir, const CGeoPoint& targetPoint);
	extern float SquareRootFloat(float number); 
	extern bool canGo(const CVisionModule* pVision, const int num, const CGeoPoint& target, const int flag, const double avoidBuffer);
	extern double dist_SegToPoint(double segFirstX, double segFirstY, double segSecondX, double segSecondY, double pointX, double pointY);
	extern CGeoPoint MakeOutOfLongCircle(double segFirstX, double segFirstY, double segSecondX, double segSecondY, const CGeoPoint &p);
	extern bool point_OnLineOnSeg(const double segFirstX, const double segFirstY, const double segSecondX, const double segSecondY, const double pointX, const double pointY);
	extern double min_Obs(double a, double b);
	extern double max_Obs(double a, double b);
}
#endif
