/************************************************************************/
/* Copyright (c) CSC-RL, Zhejiang University							*/
/* Team��		SSL-ZJUNlict											*/
/* HomePage:	http://www.nlict.zju.edu.cn/ssl/WelcomePage.html		*/
/************************************************************************/
/* File:	  gpuBestAlgThread.h										*/
/* Func:	  ���������λ����̣߳�֧��CPUģʽ��GPUģʽ				*/
/* Author:	  ��Ⱥ 2012-08-18											*/
/*            Ҷ���� 2014-03-01                                         */
/* Refer:	  ###														*/
/* E-mail:	  wangqun1234@zju.edu.cn									*/
/* Version:	  0.0.1														*/
/************************************************************************/

#ifndef _GPU_BEST_ALG_THREAD_H
#define _GPU_BEST_ALG_THREAD_H

#include <QMutex>

#include <fstream>
#include <singleton.h>
#include <deque>
#include "Vision/VisionModule.h"
#include "geometry.h"
#include "NormalPlayUtils.h"
#include "param.h"

#define AREANUM 9 // ������Ŀ

struct PointValueStruct {
	float pos_x;
	float pos_y;
	float pos;
	float value;
	PointValueStruct() : pos_x(0), pos_y(0), pos(0), value(0) {}
	bool operator < (const PointValueStruct& p) const {
		if (value < p.value)
		{
			return true;
		}
		return false;
	}
	bool operator == (const PointValueStruct& p) const {
		if (value == p.value)
		{
			return true;
		}
		return false;
	}
	bool operator > (const PointValueStruct& p) const {
		if (value > p.value)
		{
			return true;
		}
		return false;
	}
};
typedef std::vector<PointValueStruct> PointValueList;

struct AreaStruct {
	CGeoPoint _pos;
	float _value;
	int _area_idx;
	bool _conflict;

	AreaStruct(CGeoPoint& pos, float& value, int& area_idx, bool conflict) {
		_pos = pos;
		_value = value;
		_area_idx = area_idx;
		_conflict = conflict;
	}

	bool operator < (const AreaStruct& area) const {
		if (_value < area._value) {
			return true;
		}
		else {
			return false;
		}
	}
	bool operator > (const AreaStruct& area) const {
		if (_value > area._value) {
			return true;
		}
		else {
			return false;
		}
	}
	bool operator == (const AreaStruct& area) const {
		if (_value == area._value) {
			return true;
		}
		else {
			return false;
		}
	}
};
typedef std::deque<AreaStruct> AreaStructList;

// �����һ���ṹ�壬ʵ������������
// ����ʱ�������µ������ϵ�
// centerArea����һ�����ĵ㲻�䣬������Ϊ0.9��������
struct FieldRectangle {
	FieldRectangle(CGeoPoint ld, CGeoPoint ru) {
		_leftDownPos = ld;
		_rightUpPos = ru;
		_leftUpPos = CGeoPoint(ld.x(), ru.y());
		_rightDownPos = CGeoPoint(ru.x(), ld.y());
		double tempX = (ld.x() + ru.x()) / 2, tempY = (ld.y() + ru.y()) / 2;
		_centerPos = CGeoPoint(tempX, tempY);
		_rangeX = (ru.x() - ld.x()) * 0.45;
		_rangeY = (ld.y() - ru.y()) * 0.45;
	}
	// ������ӹ����ع���δʵ�֣��˲��ִ�����Ҫ���x�����ң�y�����ϵĵ�����ϵ���� from siyuan chen
	//FieldRectangle operator +(FieldRectangle& param){
	//	if (param._leftDownPos.x()<this->_leftDownPos.x()
	//		&&param._leftDownPos.y()<this->_leftDownPos.y()){
	//		return FieldRectangle(param._leftDownPos,this->_rightUpPos);
	//	}else{
	//		return FieldRectangle(this->_leftDownPos,param._rightUpPos);
	//	}
	//}
	FieldRectangle centerArea() {
		return FieldRectangle(CGeoPoint(_centerPos.x() - _rangeX, _centerPos.y() + _rangeY), CGeoPoint(_centerPos.x() + _rangeX, _centerPos.y() - _rangeY));
	}
	CGeoPoint getCenter() {
		return _centerPos;
	}
	double _rangeX, _rangeY;
	CGeoPoint _leftDownPos;
	CGeoPoint _rightUpPos;
	CGeoPoint _leftUpPos;
	CGeoPoint _rightDownPos;
	CGeoPoint _centerPos;
};


/**
@brief    ���������λ����̣߳�֧��GPU����
@details  �����Ƕ��������̵߳��㷨�̣߳��ṩȫ��������������λ��ļ��㡣
@note	ע�⣬��ÿһ֡���У���ʹ�ö���ӿ�getBestPoint����ǰ��Ҫ����һ��setSendPoint�������趨
����ĵ㡣һ��˴�Ϊ��ǰ���ڵĵ�*/

class CGPUBestAlgThread {
public:
	CGPUBestAlgThread();
	~CGPUBestAlgThread();

	/**
	@brief	ע���Ӿ���Ϣ���ҿ����㷨�߳�*/
	void initialize(CVisionModule* _pVision);

	/**
	@brief	��ģ��Ķ���ӿ�
	@param	���������id���������λ����gpuBestAlgThread.cpp�ж���
	Ŀǰ������ҪGPU���Ĺ��ܾ����øýӿڣ��պ������ҪҲ���Ը���getBestPoint()�Զ��������ӿڣ���Ϊ�˴���Ĺ淶�Բ��Ƽ�*/
	CGeoPoint getBestPointFromArea(int area_idx);

	/**
	@brief ȫ�����е������ֵ������*/
	void generatePointValue();

	/**
	@brief	�ж�ĳһ����ǰһ֡�������λ���Ƿ�����Ч*/
	// TODO Ŀǰ���ã����������׼��ɾ��
	bool isLastOneValid(const CGeoPoint& p);
	/**
	@brief	�趨�����
	@param	passPoint ��ǰ�����*/
	// TODO Ŀǰ��δʹ��
	void setSendPoint(const CGeoPoint passPoint);

	// ĿǰΪ�պ���
	double getPosPotential(const CGeoPoint p);

	// ���س���ͼ��size
	int getMapSize() { return _w * _h * sizeof(float); }

	// owl2���ӻ����躯��
	void startComm();
	void setPointValue();
	void sendPointValue();

	/**
	@brief	�����߳����õĺ���*/
    void doBestCalculation();

private:
	/**
	@brief	��һ�������ڵ�value����Ϊ255�����ֵ��
	@param	cneterPoint��������������ĵ�
	@param	length������ĳ�����x�᷽��
	@param	width������Ŀ�����y�᷽��*/
	void erasePointPotentialValue(const CGeoPoint centerPoint, float length, float width);

	/**
	@brief	Ѱ��һ������������ŵ�
	@param	leftUp ȡ����������ϵ�(x�������ҷ����ֱ������ϵ��)
	@param	rightDown ȡ����������µ�(x�������ҷ����ֱ������ϵ��)
	ԭ��֧��ѡ��������״�Լ�ȥ������ϵͳ���Ʋ����Ĵ��룬Ŀǰ��δʵ�֣���ʵ�ֻ������ܣ���������Ҫ���Լ���*/
	void getBestPoint(const CGeoPoint leftUp, const CGeoPoint rightDown, CGeoPoint& bestPoint, float& minValue);

	/**
	@brief ����ȫ�����ܵ㣬������ǰ���������е����ŵ�
	Ŀǰ������Ŀ��Ϊ�������������ɵĵ����*/
	void processPointValue();

	/**
	@brief �ж��������Ƿ����Ͻ���Ŀǰ���ж��Ƿ���һ��������
	���ڱ����������ɵĵ����*/
	bool isClose(const CGeoPoint pos1, const CGeoPoint pos2, float x_distance, float y_distance);

private:
	float* _PointPotentialOrigin, * _PointPotential, * _start_pos_cpu;	     ///GPU����ֵ���顢�������GPU����ֵ���顢�����ϻ����˼������Ϣ����
	// ע�⣺_start_pos_cpu�д洢��С���������Ϣ�����ᱻ���Ƶ�GPU�У�����޸���������Ϣ�Ĵ��ݹ�������ϸ�Ķ���ֵ���ִ�����GPU���ֽ�������
	//

	CVisionModule* _pVision;			             ///<ͼ��ָ��
	int _lastGPUCycle;					 ///��һ֡GPU֡��
	int _lastCycle[AREANUM] = { 0 };           ///  ��һ֡9�������CPU֡��
	CGeoPoint _bestPoint[AREANUM];         /// ��ǰ֡9����������ŵ�

	int _start_pos_x, _start_pos_y, _width, _height, _step;     ///��������������ֱ�Ϊ���Ͻ����ꡢ�����������������
	int _w, _h;                                       /// ����ռ����  
	int _palyer_pos_num;                              //һ�����������贫�ݵ���Ϣ��Ŀ��ĿǰΪС����λ�á������ٶȣ���λΪ�Ƿ�valid��

	CGeoPoint sendPoint;				///<����㣬һ��Ϊ�����ڵĵ�
	int halfLength; // ���ذ볤
	int halfWidth;  // ���ذ��
	int halfGoalWidth;
    float _pitch_info[4] = { static_cast<float>(Param::Field::PITCH_LENGTH), static_cast<float>(Param::Field::PITCH_WIDTH), static_cast<float>(Param::Field::PENALTY_AREA_DEPTH), static_cast<float>(Param::Field::PENALTY_AREA_WIDTH) }; // ������Ϣ������Ϊ���س�������������ȡ���

	PointValueList pointValueList;
};

typedef NormalSingleton<CGPUBestAlgThread> GPUBestAlgThread;
#endif