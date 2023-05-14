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
#include <sstream>
#include "TaskMediator.h"
extern Semaphore vision_to_cuda;


#ifdef ENABLE_CUDA
extern "C" void calc_with_gpu(float* map_cpu, float* start_pos_cpu, int length, int width, int pos_num, float* pitch_info);
extern "C" void ball_model_calc_with_gpu(float* vel_data_cpu, float* predict_results, float* a_1_matrix_cpu, float* bias_1_matrix_cpu, float* a_2_matrix_cpu, float* bias_2_matrix_cpu);
#else
void calc_with_gpu(float* map_cpu, float* start_pos_cpu, int length, int width, int pos_num, float* pitch_info) {};
void ball_model_calc_with_gpu(float* vel_data_cpu, float* predict_results, float* a_1_matrix_cpu, float* bias_1_matrix_cpu, float* a_2_matrix_cpu, float* bias_2_matrix_cpu) {};
#endif // 


#define OURPLAYER_NUM	6
#define THEIRPLAYER_NUM 6
#define BALL_NUM		1

namespace gpuCalcArea {
    std::thread* _best_calculation_thread = nullptr;
    QUdpSocket* heatMap_socket;
    int heatMap_port;
	const double PI = 3.1415926;
	const int Color_Size = 256;

	const double differenceX = Param::Field::PITCH_LENGTH / 9;

	const double middleFrontBorderX = Param::Field::PITCH_LENGTH / 6;
	const double middleBackBorderX = -Param::Field::PITCH_LENGTH / 6;
	const double centerLeftBorderY = -Param::Field::PENALTY_AREA_WIDTH / 2;
	const double centerRightBorderY = Param::Field::PENALTY_AREA_WIDTH / 2;
	
	const double sideLineFrontBorderX = ParamManager::Instance()->SUPPORT_DIST * Param::Field::PITCH_LENGTH / 2;

	const double sideLineBackBorderX = -ParamManager::Instance()->SUPPORT_DIST * Param::Field::PITCH_LENGTH / 2;

	const double sideLineLeftBorderY = -ParamManager::Instance()->SUPPORT_DIST * Param::Field::PITCH_WIDTH / 2;
    //-450
    const double sideLineRightBorderY = ParamManager::Instance()->SUPPORT_DIST * Param::Field::PITCH_WIDTH / 2;
    //450
	const double goalLineFrontBorderX = Param::Field::PITCH_LENGTH / 2 - Param::Field::PENALTY_AREA_DEPTH * 2 / 3;
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
		FieldRectangle(CGeoPoint(middleFrontBorderX,centerLeftBorderY),CGeoPoint(goalLineFrontBorderX, sideLineLeftBorderY)),
        FieldRectangle(CGeoPoint(middleFrontBorderX + differenceX,centerRightBorderY),CGeoPoint(goalLineFrontBorderX,centerLeftBorderY)),
        //FieldRectangle(CGeoPoint(450,0),CGeoPoint(450,0)), 
        FieldRectangle(CGeoPoint(middleFrontBorderX,sideLineRightBorderY),CGeoPoint(goalLineFrontBorderX,centerRightBorderY)),

		FieldRectangle(CGeoPoint(middleBackBorderX,centerLeftBorderY),CGeoPoint(middleFrontBorderX,sideLineLeftBorderY)),
        FieldRectangle(CGeoPoint(middleBackBorderX,centerRightBorderY),CGeoPoint(middleFrontBorderX + differenceX,centerLeftBorderY)),
		FieldRectangle(CGeoPoint(middleBackBorderX,sideLineRightBorderY),CGeoPoint(middleFrontBorderX,centerRightBorderY)),

		// FieldRectangle(CGeoPoint(goalLineBackBorderX,centerLeftBorderY),CGeoPoint(middleBackBorderX,sideLineLeftBorderY)),
		// FieldRectangle(CGeoPoint(goalLineBackBorderX,centerRightBorderY),CGeoPoint(middleBackBorderX,centerLeftBorderY)),
		// FieldRectangle(CGeoPoint(goalLineBackBorderX,sideLineRightBorderY),CGeoPoint(middleBackBorderX,centerRightBorderY)),
	};
	FieldRectangle processed_fieldRectangleArray[AREANUM] = {
		FieldRectangle(CGeoPoint(middleFrontBorderX,centerLeftBorderY),CGeoPoint(goalLineFrontBorderX,sideLineLeftBorderY)),
		FieldRectangle(CGeoPoint(middleFrontBorderX + differenceX,centerRightBorderY),CGeoPoint(goalLineFrontBorderX,centerLeftBorderY)),
		//FieldRectangle(CGeoPoint(450,0),CGeoPoint(450,0)),
		FieldRectangle(CGeoPoint(middleFrontBorderX,sideLineRightBorderY),CGeoPoint(goalLineFrontBorderX,centerRightBorderY)),

		FieldRectangle(CGeoPoint(middleBackBorderX,centerLeftBorderY),CGeoPoint(middleFrontBorderX,sideLineLeftBorderY)),
		FieldRectangle(CGeoPoint(middleBackBorderX,centerRightBorderY),CGeoPoint(middleFrontBorderX + differenceX,centerLeftBorderY)),
		FieldRectangle(CGeoPoint(middleBackBorderX,sideLineRightBorderY),CGeoPoint(middleFrontBorderX,centerRightBorderY)),

		// FieldRectangle(CGeoPoint(goalLineBackBorderX,centerLeftBorderY),CGeoPoint(middleBackBorderX,sideLineLeftBorderY)),
		// FieldRectangle(CGeoPoint(goalLineBackBorderX,centerRightBorderY),CGeoPoint(middleBackBorderX,centerLeftBorderY)),
		// FieldRectangle(CGeoPoint(goalLineBackBorderX,sideLineRightBorderY),CGeoPoint(middleBackBorderX,centerRightBorderY)),
	};

}

extern QMutex* _best_visiondata_copy_mutex;
extern QMutex* _value_getter_mutex;
extern QMutex* _ball_pos_prediction_mutex;

CGPUBestAlgThread::CGPUBestAlgThread() {
	sendPoint = CGeoPoint(0, 0);
	_pVision = NULL;
	for (int i = 0; i < AREANUM; i++) {
		_lastCycle[i] = 0;
		_bestPoint[i] = CGeoPoint(0, 0);
	}
	_lastGPUCycle = 0;

	// ����Ԥ�ⲿ��
	input_dim = 10;
	hidden_layer_dim = 80;
	output_dim = 50;

	_history_ball_vel = (float*)malloc(input_dim * sizeof(float));
	for (int i = 0; i < input_dim; i++) {
		_history_ball_vel[i] = 0.0f;
	}
	_ball_pos_prediction_results = (float*)malloc(output_dim * sizeof(float));
	for (int i = 0; i < output_dim; i++) {
		_ball_pos_prediction_results[i] = 0.0f;
	}

#ifdef ENABLE_CUDA
		// ��Ҫ���ҵ�����
		_length = Param::Field::PITCH_LENGTH / 2 - gpuCalcArea::middleBackBorderX;
		_width = Param::Field::PITCH_WIDTH;
		_step = ParamManager::Instance()->step; // �����Ĳ���
		// ��ʼ��ʹ�����е㶼��������а�������ľ���
		_start_pos_x = ParamManager::Instance()->startPosX + _step * 0.5;
		_start_pos_y = ParamManager::Instance()->startPosY + _step * 0.5;
		if (_length % _step != 0 || _width % _step != 0) {
			cout << "warning warning ���سߴ粻��step��������" << endl;
		}
		// ������Ҫ����Ŀռ�
		_w = _width / _step;
		_l = _length / _step;
		int map_size = _w * _l * sizeof(float);
		// (2+1+2+2+OURPLAYER_NUM*_player_pos_num+THEIRPLAYER_NUM*_player_pos_num) * sizeof(float)
		// ����Ϊ����������ʼλ�á�����step�����λ�á�����ٶȡ��ҷ�С����λ�á������ٶȣ���λΪ�Ƿ�valid�����з�С����λ�á������ٶȣ���λΪ�Ƿ�valid��
		// ����޸��ⲿ�ִ��룬����ϸ�Ķ���ֵ��GPU���ִ��벢��֮������Ӧ���޸�
		// ��Ҫע��Ĳ�����CPU�ռ�����룬������и�ֵ�����俽����GPU��ʱ����Ŀռ䡢GPU�Ը��б���Ϣ�Ľ���
		_player_pos_num = 6; //һ�����������贫�ݵ���Ϣ��Ŀ
		int pos_size = (2 + 1 + 2 + 2 + OURPLAYER_NUM * _player_pos_num + THEIRPLAYER_NUM * _player_pos_num) * sizeof(float);

		_PointPotentialOrigin = (float*)malloc(map_size); // ���ڴ洢�����Ľ��
		_PointPotential = (float*)malloc(map_size);
		_start_pos_cpu = (float*)malloc(pos_size); // ����GPU���������
		for (int i = 0; i < 2 + 1 + 2 + 2 + OURPLAYER_NUM * _player_pos_num + THEIRPLAYER_NUM * _player_pos_num; i++) {
			_start_pos_cpu[i] = 0.0f;
		}

		// ��ȡ����
		// ���Բ㣺y=x*A+b  �������A��pytorch param�еľ�����ת�ù�ϵ
		a_1_matrix_cpu = (float*)malloc((input_dim * hidden_layer_dim) * sizeof(float));
		bias_1_matrix_cpu = (float*)malloc(hidden_layer_dim * sizeof(float));
		a_2_matrix_cpu = (float*)malloc((hidden_layer_dim * output_dim) * sizeof(float));
		bias_2_matrix_cpu = (float*)malloc(output_dim * sizeof(float));
		
		int status1 = getMatrix("../data/BallModel/model_param/a_1.txt", input_dim, hidden_layer_dim, a_1_matrix_cpu);
		int status2 = getMatrix("../data/BallModel/model_param/b_1.txt", hidden_layer_dim, 1, bias_1_matrix_cpu);
		int status3 = getMatrix("../data/BallModel/model_param/a_2.txt", hidden_layer_dim, output_dim, a_2_matrix_cpu);
		int status4 = getMatrix("../data/BallModel/model_param/b_2.txt", output_dim, 1, bias_2_matrix_cpu);


		if (status1 && status2 && status3 && status4) {
			//std::cout << "a1 matrix:";
			//for (int i = 0; i < 5; i++) {
			//	std::cout << a_1_matrix_cpu[i] << " ";
			//}
			//std::cout << std::endl;
			matrix_ok = true;
		}
#endif
}

CGPUBestAlgThread::~CGPUBestAlgThread() {
	free(_PointPotentialOrigin);
	free(_PointPotential);
	free(_start_pos_cpu);
	free(_history_ball_vel);
	free(_ball_pos_prediction_results);
	free(a_1_matrix_cpu);
	free(bias_1_matrix_cpu);
	free(a_2_matrix_cpu);
	free(bias_2_matrix_cpu);
    delete gpuCalcArea::heatMap_socket;
    gpuCalcArea::heatMap_socket = nullptr;
    delete gpuCalcArea::_best_calculation_thread;
    gpuCalcArea::_best_calculation_thread = nullptr;
}

void CGPUBestAlgThread::initialize(CVisionModule* pVision) {
	_pVision = pVision;
	// ���� GPU ������߳�
#ifdef ENABLE_CUDA
        gpuCalcArea::_best_calculation_thread = new std::thread([=] {doBestCalculation();});
        gpuCalcArea::_best_calculation_thread->detach();
#endif
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
		// _best_visiondata_copy_mutex->lock();
		// ����
		_start_pos_cpu[0] = _start_pos_x;
		_start_pos_cpu[1] = _start_pos_y;
		_start_pos_cpu[2] = _step;
		_start_pos_cpu[3] = _pVision->Ball().Pos().x();
		_start_pos_cpu[4] = _pVision->Ball().Pos().y();
		_start_pos_cpu[5] = _pVision->Ball().VelX();
		_start_pos_cpu[6] = _pVision->Ball().VelY();
		// ������������Ϣ
		int our_start_idx = 7;    // �������п�ʼ�洢��λ��
		float* our_player_info = _start_pos_cpu + our_start_idx;
		int count = 0;
		for (int i = 0; i < Param::Field::MAX_PLAYER; i++) {
			if (_pVision->OurPlayer(i).Valid()) {
				const PlayerVisionT& ourPlayer = _pVision->OurPlayer(i);
				our_player_info[_player_pos_num * count] = 1.0;
				our_player_info[_player_pos_num * count + 1] = ourPlayer.X();
				our_player_info[_player_pos_num * count + 2] = ourPlayer.Y();
				our_player_info[_player_pos_num * count + 3] = ourPlayer.Dir();
				our_player_info[_player_pos_num * count + 4] = ourPlayer.VelX();
				our_player_info[_player_pos_num * count + 5] = ourPlayer.VelY();
				count++;
			}
		}
		if (count < OURPLAYER_NUM) {
			for (int i = count; i < OURPLAYER_NUM; i++) {
				for (int j = 0; j < _player_pos_num; j++) {
					our_player_info[_player_pos_num * i + j] = 0.0;
				}
			}
		}
		count = 0;
		// �з���������Ϣ
		int their_start_idx = 7 + _player_pos_num * OURPLAYER_NUM; // �������п�ʼ�洢��λ��
		float* their_player_info = _start_pos_cpu + their_start_idx;
		for (int i = 0; i < Param::Field::MAX_PLAYER; i++) {
			if (_pVision->TheirPlayer(i).Valid()) {
				const PlayerVisionT& theirPlayer = _pVision->TheirPlayer(i);
				their_player_info[_player_pos_num * i] = 1.0;
				their_player_info[_player_pos_num * i + 1] = theirPlayer.X();
				their_player_info[_player_pos_num * i + 2] = theirPlayer.Y();
				their_player_info[_player_pos_num * i + 3] = theirPlayer.Dir();
				their_player_info[_player_pos_num * i + 4] = theirPlayer.VelX();
				their_player_info[_player_pos_num * i + 5] = theirPlayer.VelY();
				count++;
			}
		}
		if (count < THEIRPLAYER_NUM) {
			for (int i = count; i < THEIRPLAYER_NUM; i++) {
				for (int j = 0; j < _player_pos_num; j++) {
					their_player_info[_player_pos_num * i + j] = 0.0;
				}
			}
		}
		// ����
		// _best_visiondata_copy_mutex->unlock();
		int pos_num = 2 + 1 + 2 + 2 + OURPLAYER_NUM * _player_pos_num + THEIRPLAYER_NUM * _player_pos_num;

		_value_getter_mutex->lock();
		calc_with_gpu(_PointPotentialOrigin, _start_pos_cpu, _l, _w, pos_num, _pitch_info);
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

void CGPUBestAlgThread::predictBallPos() {
	//clock_t begin, end;
	//begin = clock();
	
	// _best_visiondata_copy_mutex->lock();
	// ����
	
	for (int i = 0; i < input_dim - 1; i++) {
		_history_ball_vel[i] = _history_ball_vel[i + 1];
	}
	_history_ball_vel[input_dim - 1] = _pVision->Ball().Vel().mod();
	//std::cout << "ball vel: ";
	//for (int i = 0; i < 10; i++) {
	//	std::cout << _history_ball_vel[i] << " ";
	//}
	//std::cout << std::endl;
	// ����
	// _best_visiondata_copy_mutex->unlock();

	if (matrix_ok) {
		// setģ�͵Ĳ���
		//int set_status = set_ball_model_param(a_1_matrix_cpu, bias_1_matrix_cpu, a_2_matrix_cpu, bias_2_matrix_cpu);
		//for (int i = 0; i < 5; i++) {
		//	std::cout << a_1_matrix_cpu[i] << " ";
		//}
		//std::cout << std::endl;
		float* results = (float*)malloc(output_dim * sizeof(float));
		ball_model_calc_with_gpu(_history_ball_vel, results, a_1_matrix_cpu, bias_1_matrix_cpu, a_2_matrix_cpu, bias_2_matrix_cpu);

		_ball_pos_prediction_mutex->lock();
		memcpy(_ball_pos_prediction_results, results, output_dim * sizeof(float));
		//for (int i = 0; i < 3; i++) {
		//	std::cout << results[i * 10] << " ";
		//}
		//std::cout << std::endl;
		free(results);
		_ball_pos_prediction_mutex->unlock();
	}

	//end = clock();
	//std::cout << "ball pos predict calc time (GPU): " << double(end - begin) / CLOCKS_PER_SEC * 1000 << "ms" << std::endl;
}

int CGPUBestAlgThread::getBallArea() {
	CGeoPoint ballPos = _pVision->Ball().Pos();
	int advancer = TaskMediator::Instance()->advancer();
	if (advancer != 0)
		ballPos = _pVision->OurPlayer(advancer).Pos();
	int areaNum;
	for (areaNum = 0; areaNum < AREANUM; areaNum++){
		if (gpuCalcArea::processed_fieldRectangleArray[areaNum].check4inclusion(ballPos))
			break;
	}
	return areaNum;
}

// ���մ��ķ�ʽ����֧�ŵ��б�
void CGPUBestAlgThread::supportSort() {
	int ball_area = getBallArea();
	if (ball_area == AREANUM)
		ball_area = 1; //��ֹԽ�磬���ضԷ�������������
	//           3 0  
	//  �������� 4 1  �з�����
	//           5 2
	switch (ball_area)
	{ // ����ע�;�Ϊ��ǰ��Ҫ�����������������Ϊ������ȼ�
	case 0:
		//           3 ��  
		//  �������� 2 0  �з�����
		//           4 1
		_bestSupport[0] = _bestPoint[1];
		_bestSupport[1] = _bestPoint[2];
		_bestSupport[2] = _bestPoint[4];
		_bestSupport[3] = _bestPoint[3];
		_bestSupport[4] = _bestPoint[5];
		_bestSupport[5] = _bestPoint[0];
		break;
	case 1:
		//           3 0
		//  �������� 2 ��  �з�����
		//           4 1
		_bestSupport[0] = _bestPoint[0];
		_bestSupport[1] = _bestPoint[2];
		_bestSupport[2] = _bestPoint[4];
		_bestSupport[3] = _bestPoint[3];
		_bestSupport[4] = _bestPoint[5];
		_bestSupport[5] = _bestPoint[1];
		break;
	case 2:
		//           4 1
		//  �������� 2 0 �з�����
		//           3 ��
		_bestSupport[0] = _bestPoint[1];
		_bestSupport[1] = _bestPoint[0];
		_bestSupport[2] = _bestPoint[4];
		_bestSupport[3] = _bestPoint[5];
		_bestSupport[4] = _bestPoint[3];
		_bestSupport[5] = _bestPoint[2];
		break;
	case 3:
		//           ��1
		//  �������� 2 0 �з�����
		//           4 3
		_bestSupport[0] = _bestPoint[1];
		_bestSupport[1] = _bestPoint[0];
		_bestSupport[2] = _bestPoint[4];
		_bestSupport[3] = _bestPoint[2];
		_bestSupport[4] = _bestPoint[5];
		_bestSupport[5] = _bestPoint[3];
		break;
	case 4:
		//           3 1
		//  �������� ��0 �з�����
		//           4 2
		_bestSupport[0] = _bestPoint[1];
		_bestSupport[1] = _bestPoint[0];
		_bestSupport[2] = _bestPoint[2];
		_bestSupport[3] = _bestPoint[3];
		_bestSupport[4] = _bestPoint[5];
		_bestSupport[5] = _bestPoint[4];
		break;
	case 5:
		//           4 3
		//  �������� 2 0�з�����
		//           ��1
		_bestSupport[0] = _bestPoint[1];
		_bestSupport[1] = _bestPoint[2];
		_bestSupport[2] = _bestPoint[4];
		_bestSupport[3] = _bestPoint[0];
		_bestSupport[4] = _bestPoint[3];
		_bestSupport[5] = _bestPoint[5];
		break;
	default://�������ڽ�����
		//           3 0
		//  �������� 2 ��  �з�����
		//           4 1
		_bestSupport[0] = _bestPoint[0];
		_bestSupport[1] = _bestPoint[2];
		_bestSupport[2] = _bestPoint[4];
		_bestSupport[3] = _bestPoint[3];
		_bestSupport[4] = _bestPoint[5];
		_bestSupport[5] = _bestPoint[1];
		break;
	}
}

// ���յ�ķ�ֵ����֧�ŵ��б�
void CGPUBestAlgThread::supportSortV2() {
	int ball_area = getBallArea();
	std::vector<PointValueStruct> pointValueList;
	for (int areaNum = 0; areaNum < AREANUM; areaNum++) {
		if (areaNum != ball_area) { // ����������������
			PointValueStruct p;
			p.pos = areaNum;
			p.value = _pointPotential[areaNum];
			pointValueList.push_back(p);
		}
	}
	sort(pointValueList.begin(), pointValueList.end()); // ���򣬴�С����
	int needRemove = (int)(ball_area < AREANUM);
	if (needRemove) { // ������������ķ������
		_bestSupport[AREANUM - 1] = _bestPoint[ball_area];
	}
	for (int index = 0; index < AREANUM - needRemove; index++) {
		_bestSupport[index] = _bestPoint[(int)pointValueList.at(index).pos];
	}	
}

CGeoPoint CGPUBestAlgThread::getBestPointFromArea(int support_idx) {
	
	CGeoPoint temp_bestSupport;
	_value_getter_mutex->lock();
	if (support_idx > AREANUM) { // ����Խ����������Ǻ��������λ�ò�û������
		temp_bestSupport = _bestSupport[0];
	}
	else {
		if (support_idx == 0)
			sendFieldRectangle(); // ��̬�߽�debug��Ϣ
		temp_bestSupport =  _bestSupport[support_idx];
	}
	_value_getter_mutex->unlock();
	return temp_bestSupport;
}

CGeoPoint CGPUBestAlgThread::getBallPosFromFrame(CGeoPoint ball_pos, CVector ball_vel, int frame) {
	frame = max(min(frame, output_dim - 1), 0);
	float ball_move_dist = 0.0;
	_ball_pos_prediction_mutex->lock();
	ball_move_dist = _ball_pos_prediction_results[frame];
	_ball_pos_prediction_mutex->unlock();
	return ball_pos + ball_vel / (ball_vel.mod() + 1e-8) * ball_move_dist;
}

// ��ĳһ�����ڵ�ֵ��Ϊ���ֵ���Ӷ���������
void CGPUBestAlgThread::erasePointPotentialValue(const CGeoPoint centerPoint, float length, float width) {
	// ���ز���
	// float halfPitchLength = Param::Field::PITCH_LENGTH / 2;
	// float halfPitchWidth = Param::Field::PITCH_WIDTH / 2;
	// �䷽��������getBestPoint�е���ͬ��ֻ�ǽ�������Ϊ����
	// ������ϵ��
	float left_up_pos_x = centerPoint.x() - length / 2;
	float left_up_pos_y = centerPoint.y() - width / 2;
	float right_down_pos_x = centerPoint.x() + length / 2;
	float right_down_pos_y = centerPoint.y() + width / 2;

	// ���ǲ��������������ʼ��λ��
	int start_pos_x_idx = ceil((left_up_pos_x - _start_pos_x) / _step);
	int start_pos_y_idx = ceil((left_up_pos_y - _start_pos_y) / _step);
	// int start_pos_x_idx = ceil((left_up_pos_x + halfPitchLength) / _step);
	// int start_pos_y_idx = ceil((left_up_pos_y + halfPitchWidth) / _step);

	// ���ǲ�������������������λ��
	int end_pos_x_idx = floor((right_down_pos_x - _start_pos_x) / _step);
	int end_pos_y_idx = floor((right_down_pos_y - _start_pos_y) / _step);
	// int end_pos_x_idx = floor((right_down_pos_x + halfPitchLength) / _step);
	// int end_pos_y_idx = floor((right_down_pos_y + halfPitchWidth) / _step);
	// int end_pos_x_idx = floor((right_down_pos_x - left_up_pos_x) / _step) + start_pos_x_idx;
	// int end_pos_y_idx = floor((right_down_pos_y - left_up_pos_y) / _step) + start_pos_y_idx;

	for (int i = max(start_pos_x_idx, 0); i < min(end_pos_x_idx + 1, _l); i++) {
		for (int j = max(start_pos_y_idx, 0); j < min(end_pos_y_idx + 1, _w); j++) {
			if (i * _w + j >= _w * _l) {
				std::cout << "index error";
			}
			_PointPotential[i * _w + j] = 255;
		}
	}
}

// ����ÿ����������ŵ������ֵ����generatePointValue���Ѿ����˽���������������û�мӣ��������������׼���������
void CGPUBestAlgThread::getBestPoint(const CGeoPoint leftUp, const CGeoPoint rightDown, CGeoPoint& bestPoint, float& minValue) {
	// ���¾Ÿ���������ŵ��Թ����ã�������Ҫ��¼ÿ�����ڵ�ǰcycle�Ƿ��Ѿ�����
#ifdef ENABLE_CUDA
		// ��ʼ������
		minValue = 255;
		// ���ز���
		// float halfPitchLength = Param::Field::PITCH_LENGTH / 2;
		// float halfPitchWidth = Param::Field::PITCH_WIDTH / 2;

		// ������ϵ��
		float left_up_pos_x = leftUp.x();
		float left_up_pos_y = leftUp.y();
		float right_down_pos_x = rightDown.x();
		float right_down_pos_y = rightDown.y();

		// ���ǲ��������������ʼ��λ��
		int start_pos_x_idx = ceil((left_up_pos_x - _start_pos_x) / _step);
		int start_pos_y_idx = ceil((left_up_pos_y - _start_pos_y) / _step);
		// int start_pos_x_idx = ceil((left_up_pos_x + halfPitchLength) / _step);
		// int start_pos_y_idx = ceil((left_up_pos_y + halfPitchWidth) / _step);

		// ���ǲ�������������������λ��
		int end_pos_x_idx = floor((right_down_pos_x - _start_pos_x) / _step);
		int end_pos_y_idx = floor((right_down_pos_y - _start_pos_y) / _step);
		// int end_pos_x_idx = floor((right_down_pos_x + halfPitchLength) / _step);
		// int end_pos_y_idx = floor((right_down_pos_y + halfPitchWidth) / _step);
		// int end_pos_x_idx = floor((right_down_pos_x - left_up_pos_x) / _step) + start_pos_x_idx;
		// int end_pos_y_idx = floor((right_down_pos_y - left_up_pos_y) / _step) + start_pos_y_idx;

		//// һЩ���debug��Ϣ
		//float start_pos_x = start_pos_x_idx * _step - halfPitchLength;
		//float start_pos_y = start_pos_y_idx * _step - halfPitchWidth;
		//float end_pos_x = end_pos_x_idx * _step - halfPitchLength;
		//float end_pos_y = end_pos_y_idx * _step - halfPitchWidth;
		//GDebugEngine::Instance()->gui_debug_x(CGeoPoint(start_pos_x, start_pos_y), COLOR_PURPLE);
		//GDebugEngine::Instance()->gui_debug_x(CGeoPoint(end_pos_x, end_pos_y), COLOR_RED);
		//GDebugEngine::Instance()->gui_debug_x(CGeoPoint(start_pos_x_idx * _step - halfPitchLength, end_pos_y_idx * _step - halfPitchWidth), COLOR_BLUE);
		//GDebugEngine::Instance()->gui_debug_x(CGeoPoint(end_pos_x_idx * _step - halfPitchLength, start_pos_y_idx * _step - halfPitchWidth), COLOR_GREEN);

		for (int i = max(start_pos_x_idx, 0); i < min(end_pos_x_idx + 1, _l); i++) {
		  	for (int j = max(start_pos_y_idx, 0); j < min(end_pos_y_idx + 1, _w); j++) { 
				if (_PointPotential[i * _w + j] <= minValue) { // ��ֵ��ͬ��ѡ����������ŵ�
					if (_PointPotential[i * _w + j] == minValue) {
						CGeoPoint theirGoal = CGeoPoint(Param::Field::PITCH_LENGTH / 2, 0);
						CGeoPoint temp_bestPoint = CGeoPoint(i * _step + _start_pos_x, j * _step + _start_pos_y);
						if (temp_bestPoint.dist(theirGoal) < bestPoint.dist(theirGoal)) {
							minValue = _PointPotential[i * _w + j];
							bestPoint = temp_bestPoint;
						}
					}
					else {
						minValue = _PointPotential[i * _w + j];
						bestPoint = CGeoPoint(i * _step + _start_pos_x, j * _step + _start_pos_y);
					}
					// bestPoint = CGeoPoint(i * _step - halfPitchLength, j * _step - halfPitchWidth);
				}
			}
		}
#else
		minValue = 255;
		bestPoint = leftUp.midPoint(rightDown);
#endif
}

double CGPUBestAlgThread::limitPosY(double y) {
	return std::min(std::max(y, gpuCalcArea::sideLineLeftBorderY), gpuCalcArea::sideLineRightBorderY);
}

double CGPUBestAlgThread::limitPosX(double x) {
	return std::min(std::max(x, gpuCalcArea::middleBackBorderX), gpuCalcArea::sideLineFrontBorderX);
}

void CGPUBestAlgThread::obscureBoundaryV2() {
	// �ƶ��߽�
	double ball_X = _pVision->Ball().Pos().x();
	double ball_Y = _pVision->Ball().Pos().y();
	double avoidBallDist = 100;
	double up_Y = limitPosY(ball_Y - avoidBallDist), down_Y = limitPosY(ball_Y + avoidBallDist);
	float left_X = limitPosX(ball_X - avoidBallDist), right_X = limitPosX(ball_X + avoidBallDist);
	float middle_X = (left_X + right_X) * 0.5;
	float sideLineFrontBorderX = gpuCalcArea::sideLineFrontBorderX - (gpuCalcArea::goalLineFrontBorderX - right_X);
	sideLineFrontBorderX = sideLineFrontBorderX > gpuCalcArea::sideLineFrontBorderX ? gpuCalcArea::sideLineFrontBorderX : sideLineFrontBorderX;
	// �ַ���������Ϊ����ǰ�����һ���
	gpuCalcArea::processed_fieldRectangleArray[0] = FieldRectangle(CGeoPoint(middle_X, up_Y), CGeoPoint(sideLineFrontBorderX, gpuCalcArea::sideLineLeftBorderY));
	gpuCalcArea::processed_fieldRectangleArray[1] = FieldRectangle(CGeoPoint(right_X, down_Y), CGeoPoint(sideLineFrontBorderX, up_Y));
	gpuCalcArea::processed_fieldRectangleArray[2] = FieldRectangle(CGeoPoint(middle_X, gpuCalcArea::sideLineRightBorderY), CGeoPoint(sideLineFrontBorderX, down_Y));
	gpuCalcArea::processed_fieldRectangleArray[3] = FieldRectangle(CGeoPoint(gpuCalcArea::middleBackBorderX, up_Y), CGeoPoint(middle_X, gpuCalcArea::sideLineLeftBorderY));
	gpuCalcArea::processed_fieldRectangleArray[4] = FieldRectangle(CGeoPoint(gpuCalcArea::middleBackBorderX, down_Y), CGeoPoint(left_X, up_Y));
	gpuCalcArea::processed_fieldRectangleArray[5] = FieldRectangle(CGeoPoint(gpuCalcArea::middleBackBorderX, gpuCalcArea::sideLineRightBorderY), CGeoPoint(middle_X, down_Y));
	// gpuCalcArea::processed_fieldRectangleArray[6] = FieldRectangle(CGeoPoint(gpuCalcArea::goalLineBackBorderX, up_Y), CGeoPoint(gpuCalcArea::middleBackBorderX, gpuCalcArea::sideLineLeftBorderY));
	// gpuCalcArea::processed_fieldRectangleArray[7] = FieldRectangle(CGeoPoint(gpuCalcArea::goalLineBackBorderX, down_Y), CGeoPoint(gpuCalcArea::middleBackBorderX, up_Y));
	// gpuCalcArea::processed_fieldRectangleArray[8] = FieldRectangle(CGeoPoint(gpuCalcArea::goalLineBackBorderX, gpuCalcArea::sideLineRightBorderY), CGeoPoint(gpuCalcArea::middleBackBorderX, down_Y));
	// ��������
	// ��ѹ��������Ҳ������ѱ�
	if (up_Y < gpuCalcArea::sideLineLeftBorderY + Param::Vehicle::V2::PLAYER_SIZE * 7) {
		double fixed_Y = (down_Y + gpuCalcArea::sideLineRightBorderY) * 0.5;
		gpuCalcArea::processed_fieldRectangleArray[0] = FieldRectangle(CGeoPoint(middle_X, gpuCalcArea::sideLineRightBorderY), CGeoPoint(sideLineFrontBorderX, fixed_Y));
		gpuCalcArea::processed_fieldRectangleArray[1] = FieldRectangle(CGeoPoint(right_X, down_Y), CGeoPoint(sideLineFrontBorderX, gpuCalcArea::sideLineLeftBorderY));
		gpuCalcArea::processed_fieldRectangleArray[2] = FieldRectangle(CGeoPoint(middle_X, fixed_Y), CGeoPoint(sideLineFrontBorderX, down_Y));
		gpuCalcArea::processed_fieldRectangleArray[3] = FieldRectangle(CGeoPoint(gpuCalcArea::middleBackBorderX, gpuCalcArea::sideLineRightBorderY), CGeoPoint(middle_X, fixed_Y));
		gpuCalcArea::processed_fieldRectangleArray[4] = FieldRectangle(CGeoPoint(gpuCalcArea::middleBackBorderX, down_Y), CGeoPoint(left_X, gpuCalcArea::sideLineLeftBorderY));
		gpuCalcArea::processed_fieldRectangleArray[5] = FieldRectangle(CGeoPoint(gpuCalcArea::middleBackBorderX, fixed_Y), CGeoPoint(middle_X, down_Y));
		// gpuCalcArea::processed_fieldRectangleArray[6] = FieldRectangle(CGeoPoint(gpuCalcArea::goalLineBackBorderX, gpuCalcArea::sideLineRightBorderY), CGeoPoint(gpuCalcArea::middleBackBorderX, fixed_Y));
		// gpuCalcArea::processed_fieldRectangleArray[7] = FieldRectangle(CGeoPoint(gpuCalcArea::goalLineBackBorderX, down_Y), CGeoPoint(gpuCalcArea::middleBackBorderX, gpuCalcArea::sideLineLeftBorderY));
		// gpuCalcArea::processed_fieldRectangleArray[8] = FieldRectangle(CGeoPoint(gpuCalcArea::goalLineBackBorderX, fixed_Y), CGeoPoint(gpuCalcArea::middleBackBorderX, down_Y));
	}
	// ��ѹ�Ҳ�������������ѱ�
	else if (down_Y > gpuCalcArea::sideLineRightBorderY - Param::Vehicle::V2::PLAYER_SIZE * 7) {
		double fixed_Y = (up_Y + gpuCalcArea::sideLineLeftBorderY) * 0.5;
		gpuCalcArea::processed_fieldRectangleArray[0] = FieldRectangle(CGeoPoint(middle_X, up_Y), CGeoPoint(sideLineFrontBorderX, fixed_Y));
		gpuCalcArea::processed_fieldRectangleArray[1] = FieldRectangle(CGeoPoint(right_X, gpuCalcArea::sideLineRightBorderY), CGeoPoint(sideLineFrontBorderX, up_Y));
		gpuCalcArea::processed_fieldRectangleArray[2] = FieldRectangle(CGeoPoint(middle_X, fixed_Y), CGeoPoint(sideLineFrontBorderX, gpuCalcArea::sideLineLeftBorderY));
		gpuCalcArea::processed_fieldRectangleArray[3] = FieldRectangle(CGeoPoint(gpuCalcArea::middleBackBorderX, up_Y), CGeoPoint(middle_X, fixed_Y));
		gpuCalcArea::processed_fieldRectangleArray[4] = FieldRectangle(CGeoPoint(gpuCalcArea::middleBackBorderX, gpuCalcArea::sideLineRightBorderY), CGeoPoint(left_X, up_Y));
		gpuCalcArea::processed_fieldRectangleArray[5] = FieldRectangle(CGeoPoint(gpuCalcArea::middleBackBorderX, fixed_Y), CGeoPoint(middle_X, gpuCalcArea::sideLineLeftBorderY));
		// gpuCalcArea::processed_fieldRectangleArray[6] = FieldRectangle(CGeoPoint(gpuCalcArea::goalLineBackBorderX, up_Y), CGeoPoint(gpuCalcArea::middleBackBorderX, fixed_Y));
		// gpuCalcArea::processed_fieldRectangleArray[7] = FieldRectangle(CGeoPoint(gpuCalcArea::goalLineBackBorderX, gpuCalcArea::sideLineRightBorderY), CGeoPoint(gpuCalcArea::middleBackBorderX, up_Y));
		// gpuCalcArea::processed_fieldRectangleArray[8] = FieldRectangle(CGeoPoint(gpuCalcArea::goalLineBackBorderX, fixed_Y), CGeoPoint(gpuCalcArea::middleBackBorderX, gpuCalcArea::sideLineLeftBorderY));
	}
	// ��ѹ��ǰ���������������ѱ䣬ע�Ᵽ��֮ǰ�Ľ���
	if (right_X > gpuCalcArea::sideLineFrontBorderX - Param::Vehicle::V2::PLAYER_SIZE * 7) {
		double fixed_X = (left_X + gpuCalcArea::middleBackBorderX) * 0.5;
		gpuCalcArea::processed_fieldRectangleArray[1] = FieldRectangle(gpuCalcArea::processed_fieldRectangleArray[4]._leftDownPos, CGeoPoint(fixed_X, gpuCalcArea::processed_fieldRectangleArray[4]._rightUpPos.y()));
		gpuCalcArea::processed_fieldRectangleArray[4] = FieldRectangle(CGeoPoint(fixed_X, gpuCalcArea::processed_fieldRectangleArray[4]._leftDownPos.y()), gpuCalcArea::processed_fieldRectangleArray[4]._rightUpPos);
	}
	// ��ѹ�������ǰ�������ѱ䲢���������
	else if (left_X < gpuCalcArea::middleBackBorderX + Param::Vehicle::V2::PLAYER_SIZE * 7) {
		double fixed_X = (right_X + gpuCalcArea::sideLineFrontBorderX) * 0.5;
		gpuCalcArea::processed_fieldRectangleArray[4] = FieldRectangle(CGeoPoint(fixed_X, gpuCalcArea::processed_fieldRectangleArray[1]._leftDownPos.y()), CGeoPoint(gpuCalcArea::sideLineFrontBorderX, gpuCalcArea::processed_fieldRectangleArray[1]._rightUpPos.y()));
		gpuCalcArea::processed_fieldRectangleArray[1] = FieldRectangle(gpuCalcArea::processed_fieldRectangleArray[1]._leftDownPos, CGeoPoint(fixed_X, gpuCalcArea::processed_fieldRectangleArray[1]._rightUpPos.y()));
		gpuCalcArea::processed_fieldRectangleArray[3] = FieldRectangle(CGeoPoint(fixed_X, gpuCalcArea::processed_fieldRectangleArray[0]._leftDownPos.y()), CGeoPoint(gpuCalcArea::sideLineFrontBorderX, gpuCalcArea::processed_fieldRectangleArray[0]._rightUpPos.y()));
		gpuCalcArea::processed_fieldRectangleArray[0] = FieldRectangle(CGeoPoint(gpuCalcArea::middleBackBorderX, gpuCalcArea::processed_fieldRectangleArray[0]._leftDownPos.y()), CGeoPoint(fixed_X, gpuCalcArea::processed_fieldRectangleArray[0]._rightUpPos.y()));
		gpuCalcArea::processed_fieldRectangleArray[5] = FieldRectangle(CGeoPoint(fixed_X, gpuCalcArea::processed_fieldRectangleArray[2]._leftDownPos.y()), CGeoPoint(gpuCalcArea::sideLineFrontBorderX, gpuCalcArea::processed_fieldRectangleArray[2]._rightUpPos.y()));
		gpuCalcArea::processed_fieldRectangleArray[2] = FieldRectangle(CGeoPoint(gpuCalcArea::middleBackBorderX, gpuCalcArea::processed_fieldRectangleArray[2]._leftDownPos.y()), CGeoPoint(fixed_X, gpuCalcArea::processed_fieldRectangleArray[2]._rightUpPos.y()));
	}
	// ��ѹбǰ�����򣬲�̫�����
	if (middle_X > gpuCalcArea::sideLineFrontBorderX - Param::Vehicle::V2::PLAYER_SIZE * 7) {
		double fixed_X = (middle_X + gpuCalcArea::middleBackBorderX) * 0.5;
		gpuCalcArea::processed_fieldRectangleArray[0] = FieldRectangle(gpuCalcArea::processed_fieldRectangleArray[3]._leftDownPos, CGeoPoint(fixed_X, gpuCalcArea::processed_fieldRectangleArray[3]._rightUpPos.y()));
		gpuCalcArea::processed_fieldRectangleArray[3] = FieldRectangle(CGeoPoint(fixed_X, gpuCalcArea::processed_fieldRectangleArray[3]._leftDownPos.y()), CGeoPoint(gpuCalcArea::sideLineFrontBorderX, gpuCalcArea::processed_fieldRectangleArray[3]._rightUpPos.y()));
		gpuCalcArea::processed_fieldRectangleArray[2] = FieldRectangle(gpuCalcArea::processed_fieldRectangleArray[5]._leftDownPos, CGeoPoint(fixed_X, gpuCalcArea::processed_fieldRectangleArray[5]._rightUpPos.y()));
		gpuCalcArea::processed_fieldRectangleArray[5] = FieldRectangle(CGeoPoint(fixed_X, gpuCalcArea::processed_fieldRectangleArray[5]._leftDownPos.y()), CGeoPoint(gpuCalcArea::sideLineFrontBorderX, gpuCalcArea::processed_fieldRectangleArray[5]._rightUpPos.y()));
	}
}

// ��̬�߽磺��������߽�˵㵽��ľ��������ɱ����ƶ�
void CGPUBestAlgThread::obscureBoundary() {
	float ball_X = _pVision->Ball().Pos().x();
	float ball_Y = _pVision->Ball().Pos().y();
	float obsRate = 0.2; // �ú���Ψһ������ģ��ϵ������ΧΪ[0, 1]
	float mov_luX[AREANUM]; float mov_luY[AREANUM];
	float mov_rdX[AREANUM]; float mov_rdY[AREANUM];

	for (int area_idx = 0; area_idx < AREANUM; area_idx++) {
		mov_luX[area_idx] = obsRate * (ball_X - gpuCalcArea::fieldRectangleArray[area_idx]._leftUpPos.x());
		mov_luY[area_idx] = obsRate * (ball_Y - gpuCalcArea::fieldRectangleArray[area_idx]._leftUpPos.y());
		mov_rdX[area_idx] = obsRate * (ball_X - gpuCalcArea::fieldRectangleArray[area_idx]._rightDownPos.x());
		mov_rdY[area_idx] = obsRate * (ball_Y - gpuCalcArea::fieldRectangleArray[area_idx]._rightDownPos.y());//��ȡ����˵㵽��ľ�������
		// qDebug() << area_idx << mov_X[area_idx] << mov_Y[area_idx];
	}
	CGeoPoint temp_leftUp;
	CGeoPoint temp_rightDown;
	
	//��˳�����������±߽硾012345��
	temp_leftUp    = CGeoPoint(gpuCalcArea::fieldRectangleArray[0]._leftUpPos.x()    + mov_luX[0], gpuCalcArea::fieldRectangleArray[0]._leftUpPos.y());
	temp_rightDown = CGeoPoint(gpuCalcArea::fieldRectangleArray[0]._rightDownPos.x()           , gpuCalcArea::fieldRectangleArray[0]._rightDownPos.y() + mov_rdY[0]);
	gpuCalcArea::processed_fieldRectangleArray[0].processField(temp_leftUp, temp_rightDown);

	temp_leftUp    = CGeoPoint(gpuCalcArea::fieldRectangleArray[1]._leftUpPos.x()    + mov_luX[1], gpuCalcArea::fieldRectangleArray[1]._leftUpPos.y()    + mov_luY[1]);
	temp_rightDown = CGeoPoint(gpuCalcArea::fieldRectangleArray[1]._rightDownPos.x() + mov_rdX[1], gpuCalcArea::fieldRectangleArray[1]._rightDownPos.y() + mov_rdY[1]);
	gpuCalcArea::processed_fieldRectangleArray[1].processField(temp_leftUp, temp_rightDown);

	temp_leftUp    = CGeoPoint(gpuCalcArea::fieldRectangleArray[2]._leftUpPos.x()    + mov_luX[2], gpuCalcArea::fieldRectangleArray[2]._leftUpPos.y()    + mov_luY[2]);
	temp_rightDown = CGeoPoint(gpuCalcArea::fieldRectangleArray[2]._rightDownPos.x()           , gpuCalcArea::fieldRectangleArray[2]._rightDownPos.y());
	gpuCalcArea::processed_fieldRectangleArray[2].processField(temp_leftUp, temp_rightDown);

	temp_leftUp    = CGeoPoint(gpuCalcArea::fieldRectangleArray[3]._leftUpPos.x()    + mov_luX[3], gpuCalcArea::fieldRectangleArray[3]._leftUpPos.y());
	temp_rightDown = CGeoPoint(gpuCalcArea::fieldRectangleArray[3]._rightDownPos.x() + mov_rdX[3], gpuCalcArea::fieldRectangleArray[3]._rightDownPos.y() + mov_rdY[3]);
	gpuCalcArea::processed_fieldRectangleArray[3].processField(temp_leftUp, temp_rightDown);

	temp_leftUp    = CGeoPoint(gpuCalcArea::fieldRectangleArray[4]._leftUpPos.x()    + mov_luX[4], gpuCalcArea::fieldRectangleArray[4]._leftUpPos.y()    + mov_luY[4]);
	temp_rightDown = CGeoPoint(gpuCalcArea::fieldRectangleArray[4]._rightDownPos.x() + mov_rdX[4], gpuCalcArea::fieldRectangleArray[4]._rightDownPos.y() + mov_rdY[4]);
	gpuCalcArea::processed_fieldRectangleArray[4].processField(temp_leftUp, temp_rightDown);

	temp_leftUp    = CGeoPoint(gpuCalcArea::fieldRectangleArray[5]._leftUpPos.x()    + mov_luX[5], gpuCalcArea::fieldRectangleArray[5]._leftUpPos.y()    + mov_luY[5]);
	temp_rightDown = CGeoPoint(gpuCalcArea::fieldRectangleArray[5]._rightDownPos.x() + mov_rdX[5], gpuCalcArea::fieldRectangleArray[5]._rightDownPos.y());
	gpuCalcArea::processed_fieldRectangleArray[5].processField(temp_leftUp, temp_rightDown);

	/*
	temp_leftUp    = CGeoPoint(gpuCalcArea::fieldRectangleArray[6]._leftUpPos.x()              , gpuCalcArea::fieldRectangleArray[6]._leftUpPos.y() );
	temp_rightDown = CGeoPoint(gpuCalcArea::fieldRectangleArray[6]._rightDownPos.x() + mov_X[6], gpuCalcArea::fieldRectangleArray[6]._rightDownPos.y() + mov_Y[6]);
	gpuCalcArea::processed_fieldRectangleArray[6].processField(temp_leftUp, temp_rightDown);

	temp_leftUp    = CGeoPoint(gpuCalcArea::fieldRectangleArray[7]._leftUpPos.x()              , gpuCalcArea::fieldRectangleArray[7]._leftUpPos.y()    + mov_Y[7]);
	temp_rightDown = CGeoPoint(gpuCalcArea::fieldRectangleArray[7]._rightDownPos.x() + mov_X[7], gpuCalcArea::fieldRectangleArray[7]._rightDownPos.y() + mov_Y[7]);
	gpuCalcArea::processed_fieldRectangleArray[7].processField(temp_leftUp, temp_rightDown);

	temp_leftUp    = CGeoPoint(gpuCalcArea::fieldRectangleArray[8]._leftUpPos.x()              , gpuCalcArea::fieldRectangleArray[8]._leftUpPos.y()    + mov_Y[8]);
	temp_rightDown = CGeoPoint(gpuCalcArea::fieldRectangleArray[8]._rightDownPos.x() + mov_X[8], gpuCalcArea::fieldRectangleArray[8]._rightDownPos.y());
	gpuCalcArea::processed_fieldRectangleArray[8].processField(temp_leftUp, temp_rightDown);
	*/


	return;
}

void CGPUBestAlgThread::increaseRobust() {
	cum = (cum+1)%10;
	if (abs(_prePointPotential[0] - _pointPotential[0]) < thresholdValue)
	{
		for (int i = 0; i < AREANUM; i++) {
			_bestSupport[i] = _preBestSupport[i];
			_pointPotential[i] = _prePointPotential[i];
		}
	}
	for (int i = 0; cum == 0 && i < AREANUM; i++) {
		_preBestSupport[i] = _bestSupport[i];
		_prePointPotential[i] = _pointPotential[i];
	}
}


// ����ÿ��������generatePointValue���Ѿ����˽���������������û�мӣ��������������׼���������
void CGPUBestAlgThread::processPointValue() {
	// ��_PointPotential���ݽ��д������ҳ�ǰ�������е�����ֵ�����洢��_bestPoint��
	AreaStructList areaStructList;
	CGeoPoint bestPoint;
	float minValue;
	int area_idx;

	if (ParamManager::Instance()->boundaryVersion == 1) {
		obscureBoundary(); //��̬ģ���߽�
	}
	else if (ParamManager::Instance()->boundaryVersion == 2) {
		obscureBoundaryV2();
	}


	// �����������������ʱ���ŵ�
	for (int area_idx = 0; area_idx < AREANUM; area_idx++) {
		getBestPoint(gpuCalcArea::processed_fieldRectangleArray[area_idx].centerArea()._leftUpPos, gpuCalcArea::processed_fieldRectangleArray[area_idx].centerArea()._rightDownPos, bestPoint, minValue);
		areaStructList.push_back(AreaStruct(bestPoint, minValue, area_idx, false));
	}

	while (areaStructList.size()) {
		// �����򣬴�С����
		sort(areaStructList.begin(), areaStructList.end());
		// �ж�value��С�ĵ��Ƿ���ѡ�����ͻ
		if (areaStructList.at(0)._conflict) { // �����ͻ�����¼���õ㣬�����¸õ���Ϣ
			area_idx = areaStructList.at(0)._area_idx;
			getBestPoint(gpuCalcArea::processed_fieldRectangleArray[area_idx].centerArea()._leftUpPos, gpuCalcArea::processed_fieldRectangleArray[area_idx].centerArea()._rightDownPos, bestPoint, minValue);
			areaStructList.at(0)._pos = bestPoint;
			areaStructList.at(0)._value = minValue;
			areaStructList.at(0)._conflict = false;
		}
		else { // �������ͻ�����õ��Ƴ����д�����滮�������У����ݸõ�Գ��ƽ���erase�������ж�������Ƿ��ͻ
			minValue = areaStructList.at(0)._value;
			bestPoint = areaStructList.at(0)._pos;
			area_idx = areaStructList.at(0)._area_idx;
			areaStructList.pop_front();
			_pointPotential[area_idx] = minValue;
			_bestPoint[area_idx] = bestPoint;
			// �ж��Ƿ��Ѿ�ȫ���Ƴ����У����ڸò�����������һ���㲢����earse����
			if (areaStructList.empty()) {
				break;
			}
			else {
				float length = 130; // 200;
				float width = 130; // 200;
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

	// if (ParamManager::Instance()->boundaryVersion == 1) {
	 	supportSortV2(); // ������Ҫ�Զ�֧�ŵ��������	
	// }
	// else if (ParamManager::Instance()->boundaryVersion == 2) {
	// 	supportSortV2(); 
	// }
	// increaseRobust(); // ��ֹ������趨����0��ʼ����ɸѡ

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
		GPUBestAlgThread::Instance()->predictBallPos();
		// GPUBestAlgThread::Instance()->sendFieldRectangle();
		GPUBestAlgThread::Instance()->setPointValue();
        GPUBestAlgThread::Instance()->sendPointValue();
	}
}

double CGPUBestAlgThread::getPosPotential(const CGeoPoint p) {
	return 0;
}

void CGPUBestAlgThread::setPointValue() {
	// _value_getter_mutex->lock();
	pointValueList.clear();
	int size = _l * _w;
	for (int i = 0; i < size; i++) {
		if (_PointPotentialOrigin[i] < 0) _PointPotentialOrigin[i] = 0;
		else if (_PointPotentialOrigin[i] > 255) _PointPotentialOrigin[i] = 255;
		PointValueStruct p;
		p.pos = i;
		p.value = _PointPotentialOrigin[i];
		pointValueList.push_back(p);
	}
	// _value_getter_mutex->unlock();
}

void CGPUBestAlgThread::sendPointValue() {
	//�������Ϊ���ɸ���ɫ,�������Ϊ�����е㰴��ֵ��С����Ϊ256���֣�ÿ���ֶ�Ӧһ����ɫ
	//���򣬴Ӵ�С
	sort(pointValueList.begin(), pointValueList.end(), greater<PointValueStruct>());
	int point_size = pointValueList.size();
    //Heat_Map msgs;
    OWL::Protocol::Heat_Map_New msgs;
	for (int m = gpuCalcArea::Color_Size - 1; m >= 0; m--) { //�Ȱ���Ҫ�ĵ㷢��ȥ����ֵ��С��
		msgs.set_login_name(OParamManager::Instance()->LoginName);
		auto points = msgs.add_points();
		for (int n = m * point_size / gpuCalcArea::Color_Size; n < (m + 1) * point_size / gpuCalcArea::Color_Size; n++) {
			points->add_pos(pointValueList.at(n).pos);
			points->set_color(255 - pointValueList.at(n).value);
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

int CGPUBestAlgThread::getMatrix(const string file_name, int max_row_num, int max_col_num, float* matrix)
{
	ifstream file_stream;
	string one_line = "";	//�����ļ���ĳһ��
	double tmp = 0;		//��ǰλ���ϵ���ֵ
	int row_count = 0;	// ����������
	int col_count = 0;	// ����������
	int max_index = max_row_num * (max_col_num - 1) - 1;
	string line;

	// ���ļ�
	file_stream.open(file_name, ios::in);	//ios::in ��ʾ��ֻ���ķ�ʽ��ȡ�ļ�
	if (file_stream.fail()){ //�ļ���ʧ��:����0
		cout << "matrix: " << file_name << "doesn't exit." << endl;
		file_stream.close();
		system("pause");
		return 0;
	}

	while (getline(file_stream, line)) // line�в�����ÿ�еĻ��з�
	{
		string number;
		istringstream readstr(line); //string��������
		//��һ�����ݰ�'��'�ָ�
		for (int col = 0; col < max_col_num; col++) { //�ɸ������ݵ�ʵ�����ȡѭ����ȡ
			getline(readstr, number, ' '); //ѭ����ȡ����
			int index = row_count * max_col_num + col;
			matrix[index] = atof(number.c_str());
		}
		row_count++;
	}
	return 1;
}// END OF getInputData

void CGPUBestAlgThread::sendFieldRectangle() {
	//����߽�debug��Ϣ
	for (int i = 0; i < AREANUM; i++) {
		CGeoPoint leftUpPos = gpuCalcArea::processed_fieldRectangleArray[i]._leftUpPos;
		CGeoPoint rightUpPos = gpuCalcArea::processed_fieldRectangleArray[i]._rightUpPos;
		CGeoPoint leftDownPos = gpuCalcArea::processed_fieldRectangleArray[i]._leftDownPos;
		CGeoPoint rightDownPos = gpuCalcArea::processed_fieldRectangleArray[i]._rightDownPos;
		CGeoPoint centerPos = gpuCalcArea::processed_fieldRectangleArray[i].getCenter();
		/*
		GDebugEngine::Instance()->gui_debug_line(leftUpPos, rightUpPos, COLOR_BLACK);
		GDebugEngine::Instance()->gui_debug_line(rightUpPos, rightDownPos, COLOR_BLACK);
		GDebugEngine::Instance()->gui_debug_line(rightDownPos, leftDownPos, COLOR_BLACK);
		GDebugEngine::Instance()->gui_debug_line(leftDownPos, leftUpPos, COLOR_BLACK);
		GDebugEngine::Instance()->gui_debug_x(_bestSupport[i], COLOR_WHITE);
		GDebugEngine::Instance()->gui_debug_msg(_bestSupport[i], QString::number(i).toStdString().c_str(), COLOR_BLACK);
		GDebugEngine::Instance()->gui_debug_msg(centerPos, QString::number(_pointPotential[i]).toStdString().c_str(), COLOR_BLACK);
		*/
	}
	//֧�ŵ�˳��debug��Ϣ
	/*
	//GDebugEngine::Instance()->gui_debug_msg(_bestSupport[0], QString::number(cum).toStdString().c_str(), COLOR_BLACK);
	GDebugEngine::Instance()->gui_debug_msg(_bestSupport[0], "000", COLOR_YELLOW);
	GDebugEngine::Instance()->gui_debug_msg(_bestSupport[1], "111", COLOR_YELLOW);
	GDebugEngine::Instance()->gui_debug_msg(_bestSupport[2], "222", COLOR_YELLOW);
	GDebugEngine::Instance()->gui_debug_msg(_bestSupport[3], "333", COLOR_YELLOW);
	GDebugEngine::Instance()->gui_debug_msg(_bestSupport[4], "444", COLOR_YELLOW);
	GDebugEngine::Instance()->gui_debug_msg(_bestSupport[5], "555", COLOR_YELLOW);
	*/
}
