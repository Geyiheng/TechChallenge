/************************************************************************/
/* Copyright (c) CSC-RL, Zhejiang University							*/
/* Team��		SSL-ZJUNlict											*/
/* HomePage:	http://www.nlict.zju.edu.cn/ssl/WelcomePage.html		*/
/************************************************************************/
/* File:	  gpuBestAlgThread.cpp										*/
/* Func:	  ���������λ����̣߳�֧��CPUģʽ��GPUģʽ				*/
/* Author:	  ��Ⱥ 2012-08-18											*/
/*            Ҷ���� 2014-03-01                                         */
/* Refer:	  ###														*/
/* E-mail:	  wangqun1234@zju.edu.cn									*/
/************************************************************************/
/* �����޸ģ�															*/
/* ֧��GPU��㣬������Ϣ���͵�owl2���������ӻ�	from chen siyaun		*/
/************************************************************************/

#include "gpuBestAlgThread.h"
#include "Vision/VisionModule.h"
#include "GDebugEngine.h"
#include "ShootRangeList.h"
#include "param.h"
#include "src_heatMap.pb.h"
#include <time.h>
#include <thread>
#include "Semaphore.h"
extern Semaphore vision_to_cuda;

#define has_GPU false

#if has_GPU
extern "C" void calc_with_gpu(float* map_cpu, float* start_pos_cpu, int height, int width, int pos_num, float* pitch_info);
#else
void calc_with_gpu(float* map_cpu, float* start_pos_cpu, int height, int width, int pos_num, float* pitch_info) {};
#endif // 


#define OURPLAYER_NUM	8
#define THEIRPLAYER_NUM 8
#define BALL_NUM		1

namespace gpuCalcArea {
    std::thread* _best_calculation_thread = nullptr;
    QUdpSocket* heatMap_socket;
    int heatMap_port;
	const double PI = 3.1415926;
	const int Color_Size = 256;

	const double middleFrontBorderX = Param::Field::PITCH_LENGTH / 6;
	const double middleBackBorderX = -Param::Field::PITCH_LENGTH / 6;
	const double centerLeftBorderY = -Param::Field::PENALTY_AREA_WIDTH / 2;
	const double centerRightBorderY = Param::Field::PENALTY_AREA_WIDTH / 2;

	const double sideLineLeftBorderY = -ParamManager::Instance()->SUPPORT_DIST * Param::Field::PITCH_WIDTH / 2;
    //-450
    const double sideLineRightBorderY = ParamManager::Instance()->SUPPORT_DIST * Param::Field::PITCH_WIDTH / 2;
    //450
	const double goalLineFrontBorderX = Param::Field::PITCH_LENGTH / 2 - Param::Field::PENALTY_AREA_DEPTH;
    //480
	const double goalLineBackBorderX = -Param::Field::PITCH_LENGTH / 2 + Param::Field::PENALTY_AREA_DEPTH;
    //-480
	const double penaltyFrontBorderX = (Param::Field::PITCH_LENGTH / 2 - Param::Field::PENALTY_AREA_DEPTH) / 2;
    //240
    const double penaltyBackBorderX = -(Param::Field::PITCH_LENGTH / 2 + Param::Field::PENALTY_AREA_DEPTH) / 2;
    //-240
	// ���ر��
	// �µĳ�����Ϣ
	//           6 3 0  
	//  �������� 7 4 1  �з�����
	//           8 5 2
	// �ò��ִ�������Ӧ�µ���λ���Լ��µ�GPU���㷽��
	// from siyuan chen
	// Ŀǰ��������Ѿ�����

	FieldRectangle fieldRectangleArray[AREANUM] = {
		FieldRectangle(CGeoPoint(middleFrontBorderX,centerLeftBorderY),CGeoPoint(goalLineFrontBorderX,sideLineLeftBorderY)),
        FieldRectangle(CGeoPoint(middleFrontBorderX + 150.0,centerRightBorderY),CGeoPoint(goalLineFrontBorderX - 50,centerLeftBorderY)),
        //FieldRectangle(CGeoPoint(450,0),CGeoPoint(450,0)),
        FieldRectangle(CGeoPoint(middleFrontBorderX,sideLineRightBorderY),CGeoPoint(goalLineFrontBorderX,centerRightBorderY)),

		FieldRectangle(CGeoPoint(middleBackBorderX,centerLeftBorderY),CGeoPoint(middleFrontBorderX,sideLineLeftBorderY)),
        FieldRectangle(CGeoPoint(middleBackBorderX,centerRightBorderY),CGeoPoint(middleFrontBorderX + 150.0,centerLeftBorderY)),
		FieldRectangle(CGeoPoint(middleBackBorderX,sideLineRightBorderY),CGeoPoint(middleFrontBorderX,centerRightBorderY)),

		FieldRectangle(CGeoPoint(goalLineBackBorderX,centerLeftBorderY),CGeoPoint(middleBackBorderX,sideLineLeftBorderY)),
		FieldRectangle(CGeoPoint(goalLineBackBorderX,centerRightBorderY),CGeoPoint(middleBackBorderX,centerLeftBorderY)),
		FieldRectangle(CGeoPoint(goalLineBackBorderX,sideLineRightBorderY),CGeoPoint(middleBackBorderX,centerRightBorderY)),
	};
}

extern QMutex* _best_visiondata_copy_mutex;
extern QMutex* _value_getter_mutex;

CGPUBestAlgThread::CGPUBestAlgThread() {
	sendPoint = CGeoPoint(0, 0);
	_pVision = NULL;

	for (int i = 0; i < AREANUM; i++) {
		_lastCycle[i] = 0;
		_bestPoint[i] = CGeoPoint(0, 0);
	}
	_lastGPUCycle = 0;

	if (has_GPU) {
		// ��Ҫ���ҵ�����
		_start_pos_x = -(int)(Param::Field::PITCH_LENGTH / 2);
		_start_pos_y = -(int)(Param::Field::PITCH_WIDTH / 2);
		// TODO �����width��height�Ǻ�������owl2�ߵ��ģ��������޸�һ��
		_height = Param::Field::PITCH_LENGTH;
		_width = Param::Field::PITCH_WIDTH;
		_step = 10; // �����Ĳ���
		if (_height % _step != 0 || _width % _step != 0) {
			cout << "warning warning ���سߴ粻��step��������" << endl;
		}
		// ������Ҫ����Ŀռ�
		_w = _width / _step;
		_h = _height / _step;
		int map_size = _w * _h * sizeof(float);
		// (2+1+2+2+OURPLAYER_NUM*_palyer_pos_num+THEIRPLAYER_NUM*_palyer_pos_num) * sizeof(float)
		// ����Ϊ����������ʼλ�á�����step�����λ�á�����ٶȡ��ҷ�С����λ�á������ٶȣ���λΪ�Ƿ�valid�����з�С����λ�á������ٶȣ���λΪ�Ƿ�valid��
		// ����޸��ⲿ�ִ��룬����ϸ�Ķ���ֵ��GPU���ִ��벢��֮������Ӧ���޸�
		// ��Ҫע��Ĳ�����CPU�ռ�����룬������и�ֵ�����俽����GPU��ʱ����Ŀռ䡢GPU�Ը��б���Ϣ�Ľ���
		_palyer_pos_num = 6; //һ�����������贫�ݵ���Ϣ��Ŀ
		int pos_size = (2 + 1 + 2 + 2 + OURPLAYER_NUM * _palyer_pos_num + THEIRPLAYER_NUM * _palyer_pos_num) * sizeof(float);

		_PointPotentialOrigin = (float*)malloc(map_size); // ���ڴ洢�����Ľ��
		_PointPotential = (float*)malloc(map_size);
		_start_pos_cpu = (float*)malloc(pos_size); // ����GPU���������
		for (int i = 0; i < 2 + 1 + 2 + 2 + OURPLAYER_NUM * _palyer_pos_num + THEIRPLAYER_NUM * _palyer_pos_num; i++) {
			_start_pos_cpu[i] = 0;
		}
	}
}

CGPUBestAlgThread::~CGPUBestAlgThread() {
	free(_PointPotentialOrigin);
	free(_PointPotential);
	free(_start_pos_cpu);
    delete gpuCalcArea::heatMap_socket;
    gpuCalcArea::heatMap_socket = nullptr;
    delete gpuCalcArea::_best_calculation_thread;
    gpuCalcArea::_best_calculation_thread = nullptr;
}

void CGPUBestAlgThread::initialize(CVisionModule* pVision) {
	_pVision = pVision;
	// ���� GPU ������߳�
	if (has_GPU) {
        gpuCalcArea::_best_calculation_thread = new std::thread([=] {doBestCalculation();});
        gpuCalcArea::_best_calculation_thread->detach();
	}
}

void CGPUBestAlgThread::startComm() {
    gpuCalcArea::heatMap_socket = new QUdpSocket();
    COptionModule* pOption = new COptionModule();
    gpuCalcArea::heatMap_port = pOption->MyColor() == TEAM_YELLOW ? CParamManager::Instance()->yellow_heat : CParamManager::Instance()->blue_heat;
    delete pOption;
}

void CGPUBestAlgThread::setSendPoint(const CGeoPoint passPoint) {
	sendPoint = passPoint;
}

bool CGPUBestAlgThread::isClose(const CGeoPoint pos1, const CGeoPoint pos2, float x_distance, float y_distance) {
	if (abs(pos1.x() - pos2.x()) < x_distance && abs(pos1.y() - pos2.y()) < y_distance) {
		return true;
	}
	else {
		return false;
	}
}

void CGPUBestAlgThread::generatePointValue() {
	//if (_pVision->Cycle() != _lastGPUCycle) {
		//_lastGPUCycle = _pVision->Cycle();
		/************************************************************************/
		/* �����㷨���ݴ��룺����λ����Ϣ                                       */
		/************************************************************************/
		// ����
		_best_visiondata_copy_mutex->lock();
		// ����
		_start_pos_cpu[0] = _start_pos_x;
		_start_pos_cpu[1] = _start_pos_y;
		_start_pos_cpu[2] = _step;
		_start_pos_cpu[3] = _pVision->Ball().Pos().x();
		_start_pos_cpu[4] = _pVision->Ball().Pos().y();
		_start_pos_cpu[5] = _pVision->Ball().VelX();
		_start_pos_cpu[6] = _pVision->Ball().VelX();
		// ������������Ϣ
		int our_start_idx = 7;    // �������п�ʼ�洢��λ��
		float* our_player_info = _start_pos_cpu + our_start_idx;
		for (int i = 0; i < OURPLAYER_NUM; i++) {
			if (_pVision->OurPlayer(i).Valid()) {
				const PlayerVisionT& ourPlayer = _pVision->OurPlayer(i);
				our_player_info[_palyer_pos_num * i] = 1.0;
				our_player_info[_palyer_pos_num * i + 1] = ourPlayer.X();
				our_player_info[_palyer_pos_num * i + 2] = ourPlayer.Y();
				our_player_info[_palyer_pos_num * i + 3] = ourPlayer.Dir();
				our_player_info[_palyer_pos_num * i + 4] = ourPlayer.VelX();
				our_player_info[_palyer_pos_num * i + 5] = ourPlayer.VelY();
			}
			else {
				for (int j = 0; j < _palyer_pos_num; j++) {
					our_player_info[_palyer_pos_num * i + j] = 0.0;
				}
			}
		}
		// �з���������Ϣ
		int their_start_idx = 7 + _palyer_pos_num * OURPLAYER_NUM; // �������п�ʼ�洢��λ��
		float* their_player_info = _start_pos_cpu + their_start_idx;
		for (int i = 0; i < THEIRPLAYER_NUM; i++) {
			if (_pVision->TheirPlayer(i).Valid()) {
				const PlayerVisionT& theirPlayer = _pVision->TheirPlayer(i);
				their_player_info[_palyer_pos_num * i] = 1.0;
				their_player_info[_palyer_pos_num * i + 1] = theirPlayer.X();
				their_player_info[_palyer_pos_num * i + 2] = theirPlayer.Y();
				their_player_info[_palyer_pos_num * i + 3] = theirPlayer.Dir();
				their_player_info[_palyer_pos_num * i + 4] = theirPlayer.VelX();
				their_player_info[_palyer_pos_num * i + 5] = theirPlayer.VelY();
			}
			else {
				for (int j = 0; j < _palyer_pos_num; j++) {
					their_player_info[_palyer_pos_num * i + j] = 0.0;
				}
			}
		}
		// ����
		_best_visiondata_copy_mutex->unlock();
		int pos_num = 2 + 1 + 2 + 2 + OURPLAYER_NUM * _palyer_pos_num + THEIRPLAYER_NUM * _palyer_pos_num;

		_value_getter_mutex->lock();
		calc_with_gpu(_PointPotentialOrigin, _start_pos_cpu, _h, _w, pos_num, _pitch_info);
		memcpy(_PointPotential, _PointPotentialOrigin, getMapSize());
		processPointValue();
		_value_getter_mutex->unlock();
	//}
	//else {
    //    std::this_thread::sleep_for(std::chrono::microseconds(10000));
		//cout << "not in cycle" << endl;
	//}
	// cout << "genarate time" << ends - start << endl;
}

CGeoPoint CGPUBestAlgThread::getBestPointFromArea(int area_idx) {
	if (area_idx > AREANUM) { // ����Խ����������Ǻ��������λ�ò�û������
		return _bestPoint[0];
	}
	else {
		return _bestPoint[area_idx];
	}
}

// ��ĳһ�����ڵ�ֵ��Ϊ���ֵ���Ӷ���������
void CGPUBestAlgThread::erasePointPotentialValue(const CGeoPoint centerPoint, float length, float width) {
	// ���ز���
	float halfPitchLength = Param::Field::PITCH_LENGTH / 2;
	float halfPitchWidth = Param::Field::PITCH_WIDTH / 2;
	// �䷽��������getBestPoint�е���ͬ��ֻ�ǽ�������Ϊ����
	// ������ϵ��
	float left_up_pos_x = centerPoint.x() - length / 2;
	float left_up_pos_y = centerPoint.y() - width / 2;
	float right_down_pos_x = centerPoint.x() + length / 2;
	float right_down_pos_y = centerPoint.y() + width / 2;

	// ���ǲ��������������ʼ��λ��
	int start_pos_x_idx = ceil((left_up_pos_x + halfPitchLength) / _step);
	int start_pos_y_idx = ceil((left_up_pos_y + halfPitchWidth) / _step);

	// ���ǲ�������������������λ��
	int end_pos_x_idx = floor((right_down_pos_x - left_up_pos_x) / _step) + start_pos_x_idx;
	int end_pos_y_idx = floor((right_down_pos_y - left_up_pos_y) / _step) + start_pos_y_idx;

	for (int i = start_pos_x_idx; i < end_pos_x_idx + 1; i++) {
		for (int j = start_pos_y_idx; j < end_pos_y_idx + 1; j++) {
			_PointPotential[i * _w + j] = 255;
		}
	}
}

// ����ÿ����������ŵ������ֵ����generatePointValue���Ѿ����˽���������������û�мӣ��������������׼���������
void CGPUBestAlgThread::getBestPoint(const CGeoPoint leftUp, const CGeoPoint rightDown, CGeoPoint& bestPoint, float& minValue) {
	// ���¾Ÿ���������ŵ��Թ����ã�������Ҫ��¼ÿ�����ڵ�ǰcycle�Ƿ��Ѿ�����
	if (has_GPU) {
		// ��ʼ������
		minValue = 255;
		// ���ز���
		float halfPitchLength = Param::Field::PITCH_LENGTH / 2;
		float halfPitchWidth = Param::Field::PITCH_WIDTH / 2;

		// ������ϵ��
		float left_up_pos_x = leftUp.x();
		float left_up_pos_y = leftUp.y();
		float right_down_pos_x = rightDown.x();
		float right_down_pos_y = rightDown.y();

		// ���ǲ��������������ʼ��λ��
		int start_pos_x_idx = ceil((left_up_pos_x + halfPitchLength) / _step);
		int start_pos_y_idx = ceil((left_up_pos_y + halfPitchWidth) / _step);

		// ���ǲ�������������������λ��
		int end_pos_x_idx = floor((right_down_pos_x - left_up_pos_x) / _step) + start_pos_x_idx;
		int end_pos_y_idx = floor((right_down_pos_y - left_up_pos_y) / _step) + start_pos_y_idx;

		//// һЩ���debug��Ϣ
		//float start_pos_x = start_pos_x_idx * _step - halfPitchLength;
		//float start_pos_y = start_pos_y_idx * _step - halfPitchWidth;
		//float end_pos_x = end_pos_x_idx * _step - halfPitchLength;
		//float end_pos_y = end_pos_y_idx * _step - halfPitchWidth;
		//GDebugEngine::Instance()->gui_debug_x(CGeoPoint(start_pos_x, start_pos_y), COLOR_PURPLE);
		//GDebugEngine::Instance()->gui_debug_x(CGeoPoint(end_pos_x, end_pos_y), COLOR_RED);
		//GDebugEngine::Instance()->gui_debug_x(CGeoPoint(start_pos_x_idx * _step - halfPitchLength, end_pos_y_idx * _step - halfPitchWidth), COLOR_BLUE);
		//GDebugEngine::Instance()->gui_debug_x(CGeoPoint(end_pos_x_idx * _step - halfPitchLength, start_pos_y_idx * _step - halfPitchWidth), COLOR_GREEN);

		for (int i = start_pos_x_idx; i < end_pos_x_idx + 1; i++) {
			for (int j = start_pos_y_idx; j < end_pos_y_idx + 1; j++) {
				if (_PointPotential[i * _w + j] < minValue) {
					minValue = _PointPotential[i * _w + j];
					bestPoint = CGeoPoint(i * _step - halfPitchLength, j * _step - halfPitchWidth);
				}
			}
		}
	}
	else {
		minValue = 255;
		bestPoint = leftUp.midPoint(rightDown);
	}
}

// ����ÿ��������generatePointValue���Ѿ����˽���������������û�мӣ��������������׼���������
void CGPUBestAlgThread::processPointValue() {
	// ��_PointPotential���ݽ��д������ҳ�ǰ�������е�����ֵ�����洢��_bestPoint��
	AreaStructList areaStructList;
	CGeoPoint bestPoint;
	float minValue;
	int area_idx;

	// �����������������ʱ���ŵ�
	for (int area_idx = 0; area_idx < 6; area_idx++) {
		getBestPoint(gpuCalcArea::fieldRectangleArray[area_idx].centerArea()._leftUpPos, gpuCalcArea::fieldRectangleArray[area_idx].centerArea()._rightDownPos, bestPoint, minValue);
		areaStructList.push_back(AreaStruct(bestPoint, minValue, area_idx, false));
	}

	while (areaStructList.size()) {
		// ������
		sort(areaStructList.begin(), areaStructList.end(), greater<AreaStruct>());
		// �ж�value��С�ĵ��Ƿ���ѡ�����ͻ
		if (areaStructList.at(0)._conflict) { // �����ͻ�����¼���õ㣬�����¸õ���Ϣ
			area_idx = areaStructList.at(0)._area_idx;
			getBestPoint(gpuCalcArea::fieldRectangleArray[area_idx].centerArea()._leftUpPos, gpuCalcArea::fieldRectangleArray[area_idx].centerArea()._rightDownPos, bestPoint, minValue);
			areaStructList.at(0)._pos = bestPoint;
			areaStructList.at(0)._value = minValue;
			areaStructList.at(0)._conflict = false;
		}
		else { // �������ͻ�����õ��Ƴ����д�����滮�������У����ݸõ�Գ��ƽ���erase�������ж�������Ƿ��ͻ
			bestPoint = areaStructList.at(0)._pos;
			area_idx = areaStructList.at(0)._area_idx;
			areaStructList.pop_front();
			_bestPoint[area_idx] = bestPoint;
			// �ж��Ƿ��Ѿ�ȫ���Ƴ����У����ڸò�����������һ���㲢����earse����
			if (areaStructList.empty()) {
				break;
			}
			else {
				float length = 200;
				float width = 200;
				erasePointPotentialValue(bestPoint, length, width);
				for (int i = 0; i < areaStructList.size(); i++) {
					if (!areaStructList.at(i)._conflict) {  //����Ѿ���ͻ����Ҫ�޸�
						if (isClose(bestPoint, areaStructList.at(i)._pos, length / 2, width / 2)) {
							areaStructList.at(i)._conflict = true;
						}
					}
				}
			}
		}
	}


}

bool CGPUBestAlgThread::isLastOneValid(const CGeoPoint& p) {
	CShootRangeList shootRangeList(_pVision, 1, p);
	const CValueRangeList& shootRange = shootRangeList.getShootRange();
	if (shootRange.size() > 0) {
		const CValueRange* bestRange = shootRange.getMaxRangeWidth();
		if (bestRange->getWidth() > Param::Field::BALL_SIZE * 2.0) {
			return true;
		}
	}
	return false;
}

void CGPUBestAlgThread::doBestCalculation() {
    startComm();
	while (true) {
		vision_to_cuda.Wait();
		GPUBestAlgThread::Instance()->generatePointValue();
		GPUBestAlgThread::Instance()->setPointValue();
        GPUBestAlgThread::Instance()->sendPointValue();
	}
}

double CGPUBestAlgThread::getPosPotential(const CGeoPoint p) {
	return 0;
}

void CGPUBestAlgThread::setPointValue() {
	_value_getter_mutex->lock();
	pointValueList.clear();
	int size = _h * _w;
	for (int i = 0; i < size; i++) {
		if (_PointPotentialOrigin[i] < 0) _PointPotentialOrigin[i] = 0;
		PointValueStruct p;
		p.pos = i;
		p.value = _PointPotentialOrigin[i];
		pointValueList.push_back(p);
	}
	_value_getter_mutex->unlock();
}

void CGPUBestAlgThread::sendPointValue() {
	//�������Ϊ���ɸ���ɫ,�������Ϊ�����е㰴��ֵ��С����Ϊ256���֣�ÿ���ֶ�Ӧһ����ɫ
	sort(pointValueList.begin(), pointValueList.end(), greater<PointValueStruct>());
	int point_size = pointValueList.size();
    //Heat_Map msgs;
    OWL::Protocol::Heat_Map_New msgs;
	for (int m = gpuCalcArea::Color_Size - 1; m >= 0; m--) { //�Ȱ���Ҫ�ĵ㷢��ȥ
		msgs.set_login_name(OParamManager::Instance()->LoginName);
		auto points = msgs.add_points();
		for (int n = m * point_size / gpuCalcArea::Color_Size; n < (m + 1) * point_size / gpuCalcArea::Color_Size; n++) {
			points->add_pos(pointValueList.at(n).pos);
			points->set_color(max(255 - (int)pointValueList.at(n).value, 0));
			//auto pos = points->add_pos();
			//pos->set_x(pointValueList.at(n).pos_x);
			//pos->set_y(pointValueList.at(n).pos_y);
		}
		//����UDP������С�����ƣ����Խ���ɫ��Ϣ�����η���
		//����message//
		int size = msgs.ByteSize();
        QByteArray output(size, 0);
        msgs.SerializeToArray(output.data(), size);
        gpuCalcArea::heatMap_socket->writeDatagram(output.data(), size, QHostAddress(CParamManager::Instance()->local_address), gpuCalcArea::heatMap_port);
		//ɾ����message�Ĳ��֣����տռ�//
		msgs.Clear();
		//delete[] msgs;
	}
}
