#include "device_launch_parameters.h"
#include <iostream>
#include "cuda_runtime.h"
#include "time.h"
#include "param.h"
#include <ctime>
#include "cublas_v2.h"

//namespace {
//    const float PI = 3.1415926;
//    const float M_2PI = PI * 2;
//    const float halfLength = 600;
//    float goal_pos[2] = { halfLength, 0 };
//    const float POINT_PASS_OFF = 200;
//}

// С������
#define ENEMY_NUM 6
#define SELF_NUM 6  // ������������Ŀ
#define POS_INFO_LENGTH 6 // ÿ��������λ����ռ��float��Ŀ
#define GPU_COEF_BLOCKSCORE -1
#define GPU_COEF_DISTSCORE 0
#define GPU_COEF_NEARSCORE 1.5
#define INPUT_DIM 10
#define HIDDEN_LAYER_DIM 80
#define OUTPUT_DIM 50
__constant__ float PI = 3.1415926;
__constant__ float M_2PI = 6.2831852;
__constant__ float PITCH_WIDTH = 600;
__constant__ float PITCH_LENGTH = 900;
__constant__ float PENALTY_WIDTH = 200;
__constant__ float PENALTY_DEPTH = 100;
__constant__ float POINT_PASS_OFF = 80.0;
// �󳡲���
//#define ENEMY_NUM 8
//#define SELF_NUM 8  // ������������Ŀ
//#define POS_INFO_LENGTH 6 // ÿ��������λ����ռ��float��Ŀ
//#define GPU_COEF_BLOCKSCORE -1
//#define GPU_COEF_DISTSCORE 0
//#define GPU_COEF_NEARSCORE 1.5
//#define INPUT_DIM 10
//#define HIDDEN_LAYER_DIM 80
//#define OUTPUT_DIM 50
//__constant__ float PI = 3.1415926;
//__constant__ float M_2PI = 6.2831852;
//__constant__ float PITCH_WIDTH = 900;
//__constant__ float PITCH_LENGTH = 1200;
//__constant__ float PENALTY_WIDTH = 280;
//__constant__ float PENALTY_DEPTH = 120;
//__constant__ float POINT_PASS_OFF = 80.0;

// ����ʹ��constant����ٶȣ�������û�е�ͨ
//__constant__ float GPU_A1_MATRIX[INPUT_DIM * HIDDEN_LAYER_DIM];
//__constant__ float GPU_BIAS1_MATRIX[HIDDEN_LAYER_DIM];
//__constant__ float GPU_A2_MATRIX[HIDDEN_LAYER_DIM * OUTPUT_DIM];
//__constant__ float GPU_BIAS2_MATRIX[OUTPUT_DIM];


extern "C" void get_gpu_info() {
    int deviceCount;
    cudaGetDeviceCount(&deviceCount);
    for (int i = 0; i < deviceCount; i++)
    {
        cudaDeviceProp devProp;
        cudaGetDeviceProperties(&devProp, i);
//        std::cout << "ʹ��GPU device " << i << ": " << devProp.name << std::endl;
//        std::cout << "�豸ȫ���ڴ������� " << devProp.totalGlobalMem / 1024 / 1024 << "MB" << std::endl;
//        std::cout << "SM��������" << devProp.multiProcessorCount << std::endl;
//        std::cout << "ÿ���߳̿�Ĺ����ڴ��С��" << devProp.sharedMemPerBlock / 1024.0 << " KB" << std::endl;
//        std::cout << "ÿ���߳̿������߳�����" << devProp.maxThreadsPerBlock << std::endl;
//        std::cout << "�豸��һ���߳̿飨Block���ֿ��õ�32λ�Ĵ��������� " << devProp.regsPerBlock << std::endl;
//        std::cout << "ÿ��EM������߳�����" << devProp.maxThreadsPerMultiProcessor << std::endl;
//        std::cout << "ÿ��EM������߳�������" << devProp.maxThreadsPerMultiProcessor / 32 << std::endl;
//        std::cout << "�豸�϶ദ������������ " << devProp.multiProcessorCount << std::endl;
//        std::cout << "======================================================" << std::endl;
        std::cout << "GPU device " << i << ": " << devProp.name << std::endl;
        std::cout << devProp.totalGlobalMem / 1024 / 1024 << "MB" << std::endl;
    }
}

// �ж�һ�����Ƿ��ڽ���
inline __device__ bool is_in_penalty(float pos_x, float pos_y) {
    if (abs(pos_y) <= PENALTY_WIDTH / 2 && abs(pos_x) >= (PITCH_LENGTH / 2 - PENALTY_DEPTH)) {
        return true;
    }
    else {
        return false;
    }
}

// ���������ľ���
inline __device__ float dist(float* pos_ptr1, float* pos_ptr2) {
    return sqrt((pos_ptr1[0] - pos_ptr2[0]) * (pos_ptr1[0] - pos_ptr2[0]) + (pos_ptr1[1] - pos_ptr2[1]) * (pos_ptr1[1] - pos_ptr2[1]));
}

// ���������ľ���
inline __device__ float dist(float x1, float y1, float * pos_ptr2) {
    return sqrt((x1 - pos_ptr2[0]) * (x1 - pos_ptr2[0]) + (y1 - pos_ptr2[1]) * (y1 - pos_ptr2[1]));
}

// ������������ƽ��
inline __device__ float dist2(float* pos_ptr1, float* pos_ptr2) {
    return (pos_ptr1[0] - pos_ptr2[0]) * (pos_ptr1[0] - pos_ptr2[0]) + (pos_ptr1[1] - pos_ptr2[1]) * (pos_ptr1[1] - pos_ptr2[1]);
}

// �������㹹�������ķ���
inline __device__ float dir(float* pos_ptr1, float* pos_ptr2) {
    return std::atan2(pos_ptr2[1] - pos_ptr1[1], pos_ptr2[0] - pos_ptr1[0]);
}

// ֱ����ֲ��Normalize������ϸ��û�п���
inline __device__ float Normalize(float angle)
{
    // ���ٴֵ���
    angle -= (int)(angle / M_2PI) * M_2PI;
    // ϸ���� (-PI,PI]
    while (angle > PI) {
        angle -= M_2PI;
    }

    while (angle <= -PI) {
        angle += M_2PI;
    }
    return angle;
}

// line_statusΪ1��ʾֱ��б�ʲ�Ϊ������������ʱ��ֱ�߷��� y=ax+b
// line_statusΪ0��ʾֱ��б��Ϊ������������ʱ��ֱ�߷���Ϊx=a
__device__ int get_line(float* line_point1, float* line_point2, float& a, float& b) {
    if (abs(line_point1[0] - line_point2[0]) < 1e-5) {
        a = line_point1[0];
        b = 0;
        return 0;
    }
    else {
        // ֱ�߷���:y=ax+b
        a = (line_point1[1] - line_point2[1]) / (line_point1[0] - line_point2[0]);
        b = line_point1[1] - a * line_point1[0];
        return 1;
    }
}

// ��һ��������point��������������ֱ���ϵ�������
__device__ float point2line_dist(float *point, float *line_point1, float *line_point2) {
    if (abs(line_point1[0] - line_point2[0]) < 1e-5) {
        return abs(point[0] - line_point1[0]);
    }
    else {
        // ֱ�߷���:y=ax+b
        float a = (line_point1[1] - line_point2[1]) / (line_point1[0] - line_point2[0]);
        float b = line_point1[1] - a * line_point1[0];
        return abs(a * point[0] + b - point[1]) / sqrt(a * a + 1);
    }
}

// ��һ��������point��������������ֱ�߷��̵�a��b��y=ax+b
__device__ float point2line_dist(float* point, float a, float b) {
    return abs(a * point[0] + b - point[1]) / sqrt(a * a + 1);
}

// ����ͶӰ�㣨���㣩
__device__ void get_projection(float a, float b, int line_status, float* point, float * projection_point) {
    if (line_status) { // ֱ��б��Ϊ����ֵ
        if (abs(a) < 1e-5) { // ֱ����x��ƽ��
            projection_point[0] = point[0];
            projection_point[1] = b;
        }
        else { 
            projection_point[0] = (point[0] / a + point[1] - b) / (a + 1 / a);
            projection_point[1] = a * projection_point[0] + b;
        }
    }
    else { // ֱ��б��Ϊ����
        projection_point[0] = a;
        projection_point[1] = point[1];
    }
}

// ��һ�������볡����
__device__ void MakeInField(float* point) {
    if (point[0] < - PITCH_LENGTH / 2) point[0] = - PITCH_LENGTH / 2;
    if (point[0] > PITCH_LENGTH / 2) point[0] = PITCH_LENGTH / 2;
    if (point[1] < -PITCH_WIDTH / 2) point[1] = -PITCH_WIDTH / 2;
    if (point[1] > PITCH_WIDTH / 2) point[1] = PITCH_WIDTH / 2;
}

// ��һ�����Ƴ��з�����
__device__ void MakeOutOfTheirPenaltyArea(float* point) {
    if (point[1] > 0) {
        if (point[0] - PITCH_LENGTH / 2 + PENALTY_DEPTH < PENALTY_WIDTH / 2 - point[1]) {
            point[0] = PITCH_LENGTH / 2 - PENALTY_DEPTH;
        }
        else {
            point[1] = PENALTY_WIDTH / 2;
        }
    }
    else {
        if (point[0] - PITCH_LENGTH / 2 + PENALTY_DEPTH < PENALTY_WIDTH / 2 + point[1]) {
            point[0] = PITCH_LENGTH / 2 - PENALTY_DEPTH;
        }
        else {
            point[1] = -PENALTY_WIDTH / 2;
        }
    }
}

/************************************************************************/
/* ���ۺ�������������                                                   */
/************************************************************************/
inline __device__ float evaluate_dist(float dist) {
    if (dist < 200) { // ���뿪���һ������֮��
        return 200 - dist + 100;  // 200 - dist
    }
    else if (dist > 500) {
        return (dist - 500) / 100;  // dist - 500
    }
    else {
        return 0;
    }
}

/************************************************************************/
/* ���ۺ�������������                                                   */
/************************************************************************/
__device__ float evaluate_receive(float *me_pos, float *ball_pos, float * their_player_ptr) {
    float reverseReceiveP = 0.0;

    float a,b;
    int line_status;
    line_status = get_line(ball_pos, me_pos, a, b);
    for (int i = 0; i < ENEMY_NUM; i++)
    {
        if (their_player_ptr[i * POS_INFO_LENGTH]) {
            // �򵽵з��ľ���
            float* their_palyer_pos = their_player_ptr + i * POS_INFO_LENGTH + 1;
            float dist_ball2opp = dist(ball_pos, their_palyer_pos);
            if (dist_ball2opp < 20) {
                continue;
            }
            //// �жϵз��Ƿ��������Լ��м�
            float r = ((their_palyer_pos[0]-me_pos[0]) * (ball_pos[0] - me_pos[0]) + (their_palyer_pos[1] - me_pos[1]) * (ball_pos[1] - me_pos[1])) / (dist2(me_pos, ball_pos));
            if (r < 0 || r > 1) {
                continue;
            }
            // ����ͶӰ��
            float projection_point[2];
            get_projection(a, b, line_status, their_palyer_pos, projection_point);
            float opp2proj_dist = dist(projection_point, their_palyer_pos);
            float ball2proj_dist = dist(projection_point, ball_pos);

            if (opp2proj_dist > 300 || ball2proj_dist < 10) { // �з����봫���߽�Զ��з�������Ͻ������Բ����ǵз��Դ����ߵ�Ӱ��
                continue;
            }
            float ratio_dist = opp2proj_dist / ball2proj_dist;
            // ����з���Ա�ٶȽ������ٵ�0.6����ñ�ֵ����0.6ʱ�õз���Ա�Դ�����Ӱ��
            // TODO ����ԽԶ�����ֵӦ��Խ��
            if (ratio_dist < 0.6) { 
                reverseReceiveP += -100 * ratio_dist + 60;
            }
        }
    }

    // ��Ҫ�赲�Լ�������
    float goal_pos[2] = { PITCH_LENGTH / 2 , 0 };
    float me2ball_dist = dist(me_pos, ball_pos);
    float blockMyself = point2line_dist(me_pos, ball_pos, goal_pos);
    if (blockMyself <= 50 && me2ball_dist < 400 && ball_pos[0] > 100) {
        reverseReceiveP += POINT_PASS_OFF;
    }
    else {
        reverseReceiveP += 20.0 / (blockMyself + 0.1);
    }

    // ���ܹ��ڿ������ߣ����ߴ����׽Ӳ�����
    if (abs(me_pos[1]) > PITCH_WIDTH / 2 - 100) {
        reverseReceiveP += (abs(me_pos[1]) - (PITCH_WIDTH / 2 - 100));
    }

    return reverseReceiveP;
}

/************************************************************************/
/* ���ۺ�������������                                                   */
/************************************************************************/
__device__ float evaluate_goal(float* me_pos, float* ball_pos, float* their_player_ptr) {
    float reverseGoalP = 0.0;
    float a, b;
    int line_status;
    float goal_pos[2] = { PITCH_LENGTH / 2 , 0 };
    line_status = get_line(goal_pos, me_pos, a, b);
    for (int i = 0; i < ENEMY_NUM; i++)
    {
        if (their_player_ptr[i * POS_INFO_LENGTH]) {
            // ���ŵ��з��ľ���
            float* their_palyer_pos = their_player_ptr + i * POS_INFO_LENGTH + 1;
            float dist_goal2opp = dist(goal_pos, their_palyer_pos);
            if (dist_goal2opp < 60) {
                continue;
            }
            // �жϵз��Ƿ��������Լ��м�
            float r = ((their_palyer_pos[0] - me_pos[0]) * (ball_pos[0] - me_pos[0]) + (their_palyer_pos[1] - me_pos[1]) * (ball_pos[1] - me_pos[1])) / (dist2(me_pos, goal_pos));
            if (r < 0 || r > 1) {
                continue;
            }
            // ����ͶӰ��
            float projection_point[2];
            get_projection(a, b, line_status, their_palyer_pos, projection_point);
            float opp2proj_dist = dist(projection_point, their_palyer_pos);
            float ball2proj_dist = dist(projection_point, ball_pos);

            if (opp2proj_dist > 300 || ball2proj_dist < 10) { // �з����봫���߽�Զ��з�������Ͻ������Բ����ǵз��Դ����ߵ�Ӱ��
                continue;
            }
            float ratio_dist = opp2proj_dist / ball2proj_dist;
            // ����ʱ���ٽϴ��������㷨ʹ�������е���м��㣬ʵ�����Ż��и���ѡ����˵��˶��Լ����ŵ��赲���С
            if (ratio_dist < 0.3) {
                reverseGoalP += -50 * ratio_dist + 15;
            }
        }
    }
//    // ���ڿ���y=0���߲��׽��������
//    if (abs(me_pos[1]) < 50) {
//        reverseGoalP += 10;
//    }
    return reverseGoalP;
}

__global__ void gpu_calc(float startPos[], float map[])
{
    int i = blockDim.x * blockIdx.x + threadIdx.x;
    // ��������Ĳ���
    int step = startPos[2];
    int me_x = blockIdx.x*step + startPos[0];
    int me_y = threadIdx.x*step + startPos[1];
    // Ϊ������ɫһһ��Ӧ�����ֵ��Ҫ����255
    if (is_in_penalty(me_x, me_y)) {
        map[i] = 255;
    }
    else {
        float me_pos_ptr[2] = { me_x, me_y };
        float* ball_pos_ptr = startPos + 3;
        float* ball_vel_ptr = startPos + 5;
        float* our_player_ptr = startPos + 7;
        float* their_player_ptr = startPos + (7 + POS_INFO_LENGTH * SELF_NUM);
        // ����
        // ��Ҫʹ�õ�һЩ����

        float me2ball_dist = dist(me_pos_ptr, ball_pos_ptr);

        // ����ֵ
        float dist_value = evaluate_dist(me2ball_dist);
        float receive_value = evaluate_receive(me_pos_ptr, ball_pos_ptr, their_player_ptr);
        float goal_value = evaluate_goal(me_pos_ptr, ball_pos_ptr, their_player_ptr);
        float total_value = dist_value + receive_value + goal_value;
        // float total_value = receive_value;
        // map[i] = me2ball_dist;
        if (total_value < 255) {
            map[i] = total_value;
        }
        else {
            map[i] = 255;
        }
    }
}

extern "C" void calc_with_gpu(float* map_cpu, float* start_pos_cpu, int height, int width, int pos_num, float *pitch_info) {
    //clock_t begin, end;
    //begin = clock();
    float* map_gpu, * start_pos_gpu;

    int map_size = height * width * sizeof(float);
    int pos_size = pos_num * sizeof(float);

    cudaError_t  status1 = cudaMalloc((void**)&map_gpu, map_size); // ���ڴ洢�����Ľ��
    cudaError_t  status2 = cudaMalloc((void**)&start_pos_gpu, pos_size);
    if (status1 != cudaSuccess || status2 != cudaSuccess)
    {
        size_t available, total;
        cudaMemGetInfo(&available, &total);
        std::cout << "free memory: " << available << " | total memory: " << total << std::endl;
        printf("**************** map calc cuda malloc dbuf error ******************* \r\n");
        return;
    }

    // ��������
    cudaError_t status_cpy = cudaMemcpy(start_pos_gpu, start_pos_cpu, pos_size, cudaMemcpyHostToDevice);
    if (status_cpy != cudaSuccess)
    {
        printf("**************** map calc cuda cpy error ******************* \r\n");
        return;
    }

    // ��GPU�п���������Ϣ
    //cudaMemcpyToSymbol("PITCH_LENGTH", pitch_info, sizeof(float));
    //cudaMemcpyToSymbol("PITCH_WIDTH", pitch_info + 1, sizeof(float));
    //cudaMemcpyToSymbol("PENALTY_DEPTH", pitch_info + 2, sizeof(float));
    //cudaMemcpyToSymbol("PENALTY_WIDTH", pitch_info + 3, sizeof(float));


    // ����kernelִ�����ã���1024*1024/512����block��ÿ��block������512���߳�
    dim3 dimGrid(height);
    dim3 dimBlock(width);

    // ִ��kernel
    gpu_calc << <dimGrid, dimBlock >> > (start_pos_gpu, map_gpu);

    // ����GPU�˼���õĽ��������CPU��
    cudaMemcpy(map_cpu, map_gpu, map_size, cudaMemcpyDeviceToHost);

    // �ͷ�GPU�˵��ڴ�
    cudaFree(map_gpu);
    cudaFree(start_pos_gpu);
    //end = clock();
    //std::cout << "best support point calc time (GPU): " << double(end - begin) / CLOCKS_PER_SEC * 1000 << "ms" << std::endl;
}


////////////////////////////////////////////////
//       break point calculator from csy      //

// pos info����
// (2+1+2+1+2+2+OURPLAYER_NUM*_palyer_pos_num+THEIRPLAYER_NUM*_palyer_pos_num) * sizeof(float)
// ����Ϊ����������Բ�ġ�max_drbble_dist����ǰ��Աλ�á���Ա�������λ�á�����ٶȡ��ҷ�С����λ�á������ٶȣ���λΪ�Ƿ�valid�����з�С����λ�á������ٶȣ���λΪ�Ƿ�valid��
// ����޸��ⲿ�ִ��룬����ϸ�Ķ���ֵ��GPU���ִ��벢��֮������Ӧ���޸�
// ��Ҫע��Ĳ�����CPU�ռ�����룬������и�ֵ�����俽����GPU��ʱ����Ŀռ䡢GPU�Ը��б���Ϣ�Ľ���
__global__ void break_gpu_calc(float pos_info[], float target_info[], float results[], int angle_mod, int dist_mod, float vis_points[])
{
    extern __shared__  float score_map[];

    int idx = blockDim.x * blockIdx.x + threadIdx.x;
    int target_point_idx = blockIdx.x;
    int ang_dist_idx = threadIdx.x;
    int dist_idx = ang_dist_idx % (dist_mod - 1);
    int ang_idx = (ang_dist_idx - dist_idx) / (dist_mod - 1);
    
    // �Ӿ���Ϣ������
    float* dribble_center_point = pos_info;
    float max_dribble_dist = pos_info[2];
    float* self_pos = pos_info + 3;
    float* ball_pos = pos_info + 6;
    float* target_point = target_info + 2 * target_point_idx;
    float* our_player_ptr = pos_info + 10;
    float* their_player_ptr = pos_info + (10 + POS_INFO_LENGTH * SELF_NUM);

    float point_score = 10000;

    //float mod = max_dribble_dist * 2 / (dist_idx + 1);
    float mod = 2 * max_dribble_dist * (dist_idx + 1) / (dist_mod - 1);
    float angle = (ang_idx - angle_mod) * PI / angle_mod + dir(self_pos, target_point);// +(ang_idx - angle_mod) * PI / angle_mod;

    float test_point[2];
    test_point[0] = self_pos[0] + mod * cos(angle);
    test_point[1] = self_pos[1] + mod * sin(angle);
    
    // ʹ��������Բ��
    float vec_dist = dist(test_point, dribble_center_point);
    if (vec_dist > max_dribble_dist) {
        float boundary_point[2];
        boundary_point[0] = (test_point[0] - dribble_center_point[0]) / vec_dist * max_dribble_dist;
        boundary_point[1] = (test_point[1] - dribble_center_point[1]) / vec_dist * max_dribble_dist;
        test_point[0] = dribble_center_point[0] + boundary_point[0];
        test_point[1] = dribble_center_point[1] + boundary_point[1];
    }

    // ���ƶ���������
    MakeInField(test_point);

    // ���ƶ����з�����
    //MakeOutOfTheirPenaltyArea(test_point);

    bool temp_flag = false;
    float a, b;
    int line_status = get_line(self_pos, test_point, a, b);
    for (int i = 0; i < ENEMY_NUM; i++) {
        if (their_player_ptr[i * POS_INFO_LENGTH]) {
            float* their_player_pos = their_player_ptr + i * POS_INFO_LENGTH + 1;
            float test2enemy_dist = dist(their_player_pos, test_point);
            if (test2enemy_dist < 20) {
                temp_flag = true;
                break;
            }
            //// �жϵз��Ƿ���test_point���Լ��м�
            float r = ((their_player_pos[0] - self_pos[0]) * (test_point[0] - self_pos[0]) + (their_player_pos[1] - self_pos[1]) * (test_point[1] - self_pos[1])) / (dist2(self_pos, test_point));
            if (r < 0 || r > 1) {
                continue;
            }
            float projection_point[2];
            get_projection(a, b, line_status, their_player_pos, projection_point);
            if (dist(projection_point, their_player_pos) < 40) {
                temp_flag = true;
                break;
            }
        }
    }

    int cnt = 0;
    if (temp_flag) {
        point_score = 9999;
    }
    else {
        float a1, b1;
        int line_status1 = get_line(test_point, target_point, a1, b1);
        
        // ����ָ��
        float dist_score = dist(test_point, target_point);
        
        float block_score = 8888, near_score = 9999;
        
        for (int i = 0; i < ENEMY_NUM; i++) {
            if (their_player_ptr[i * POS_INFO_LENGTH]) {
                float* their_player_pos = their_player_ptr + i * POS_INFO_LENGTH + 1;
                float straight_dist = dist(their_player_pos, test_point);
                near_score = min(near_score, straight_dist);
                //// �жϵз��Ƿ���test_point��target_point�м�
                float r = ((their_player_pos[0] - target_point[0]) * (test_point[0] - target_point[0]) + (their_player_pos[1] - target_point[1]) * (test_point[1] - target_point[1])) / (dist2(target_point, test_point));
                if (r < 0 || r > 1) {
                    continue;
                }
                cnt++;
                float projection_point[2];
                get_projection(a1, b1, line_status1, their_player_pos, projection_point);
                float projection_dist = dist(projection_point, their_player_pos);
                if (projection_dist > 60) {
                    continue;
                }
                block_score = min(projection_dist, block_score);
            }
        }
        near_score = 1 / (near_score + 1e-8);
        point_score = GPU_COEF_BLOCKSCORE * block_score + GPU_COEF_DISTSCORE * dist_score + GPU_COEF_NEARSCORE * near_score;
    }

    score_map[3 * ang_dist_idx] = point_score;
    score_map[3 * ang_dist_idx + 1] = test_point[0];
    score_map[3 * ang_dist_idx + 2] = test_point[1];
    if (blockIdx.x == 0) {
        vis_points[3 * ang_dist_idx] = point_score;
        vis_points[3 * ang_dist_idx + 1] = test_point[0];
        vis_points[3 * ang_dist_idx + 2] = test_point[1];
    }
    __syncthreads();

    float best_score = 20000;
    int best_idx = 0;
    if (threadIdx.x == 0) {
        for (int i = 0; i < (angle_mod * 2 - 1) * (dist_mod - 1); i++) {
            if (score_map[3 * i] < best_score) {
                best_score = score_map[3 * i];
                best_idx = i;
            }
        }
        results[3 * blockIdx.x] = best_score;
        results[3 * blockIdx.x + 1] = score_map[3 * best_idx + 1];
        results[3 * blockIdx.x + 2] = score_map[3 * best_idx + 2];
    }
    //if (blockIdx.x == 0 && threadIdx.x == 0) {
    //    memcpy(vis_points, score_map, 3 * (angle_mod * 2 - 1) * (dist_mod - 1) * sizeof(float));
    //}
}


// target_point_cpu�������ϵĵ�
// target_point_num��Ŀ�����Ŀ
// pos_info_cpu���Ӿ���Ϣ����ͻ����˵�λ��
// pos_info_num���Ӿ���Ϣ��Ŀ

extern "C" int break_calc_with_gpu(float* target_point_cpu, int target_point_num, float* pos_info_cpu, int pos_info_num, int angle_mod, int dist_mod, float* results, float* vis_points_cpu) {
    //clock_t begin, end;
    //begin = clock();
    float* results_gpu, * pos_info_gpu, * target_point_gpu, * vis_points_gpu;

    int result_size = target_point_num * 3 * sizeof(float);
    int pos_size = pos_info_num * sizeof(float);
    int target_info_size = target_point_num * 2 * sizeof(float);

    //std::cout << "pos_info_cpu: " << std::endl;
    //for (int i = 0; i < pos_info_num; i++) {
    //    std::cout << pos_info_cpu[i] << " ";
    //}
    //std::cout << std::endl;
    // ��ȡ�ռ�
    cudaError_t  status1 = cudaMalloc((void**)&results_gpu, result_size); // ���ڴ洢�����Ľ��
    cudaError_t  status2 = cudaMalloc((void**)&pos_info_gpu, pos_size);
    cudaError_t  status3 = cudaMalloc((void**)&target_point_gpu, target_info_size);
    cudaError_t  status4 = cudaMalloc((void**)&vis_points_gpu, 3 * (angle_mod * 2 - 1) * (dist_mod - 1) * sizeof(float));
    if (status1 != cudaSuccess || status2 != cudaSuccess || status3 != cudaSuccess)
    {
        printf("****************cuda malloc dbuf error ******************* \r\n");
        return 0;
    }

    // ��������
    cudaError_t status_cpy1 = cudaMemcpy(pos_info_gpu, pos_info_cpu, pos_size, cudaMemcpyHostToDevice);
    cudaError_t status_cpy2 = cudaMemcpy(target_point_gpu, target_point_cpu, target_info_size, cudaMemcpyHostToDevice);
    if (status_cpy1 != cudaSuccess || status_cpy2 != cudaSuccess)
    {
        printf("****************cuda cpy error ******************* \r\n");
        return 0;
    }

    // ��GPU�п���������Ϣ
    //cudaMemcpyToSymbol("PITCH_LENGTH", pitch_info, sizeof(float));
    //cudaMemcpyToSymbol("PITCH_WIDTH", pitch_info + 1, sizeof(float));
    //cudaMemcpyToSymbol("PENALTY_DEPTH", pitch_info + 2, sizeof(float));
    //cudaMemcpyToSymbol("PENALTY_WIDTH", pitch_info + 3, sizeof(float));


    // ����kernelִ�����ã���1024*1024/512����block��ÿ��block������512���߳�
    dim3 dimGrid(target_point_num);
    dim3 dimBlock((angle_mod * 2 - 1) * (dist_mod - 1));

    // ִ��kernel
    break_gpu_calc << <dimGrid, dimBlock, 3 * (angle_mod * 2 - 1)* (dist_mod - 1) * sizeof(float) >> > (pos_info_gpu, target_point_gpu, results_gpu, angle_mod, dist_mod, vis_points_gpu); // ����������ָ�������ڴ��С

    // ����GPU�˼���õĽ��������CPU��
    cudaMemcpy(results, results_gpu, result_size, cudaMemcpyDeviceToHost);
    cudaMemcpy(vis_points_cpu, vis_points_gpu, 3 * (angle_mod * 2 - 1) * (dist_mod - 1) * sizeof(float), cudaMemcpyDeviceToHost);
    // �ͷ�GPU�˵��ڴ�
    cudaFree(target_point_gpu);
    cudaFree(pos_info_gpu);
    cudaFree(results_gpu);
    return 1;

    //end = clock();
    //std::cout << "best break point calc time (GPU): " << double(end - begin) / CLOCKS_PER_SEC * 1000 << "ms" << std::endl;
}


// mat_c = mat_a * mat_b
//�Ż�������cublas��cublasSgemm���پ�������
// mat_a:M*N, mat_b:N*K, mat_c:M*K
//__global__ void matrix_multi(float* mat_a, float* mat_b, float* mat_c, int M, int N, int K) {
void matrix_multi(float* mat_a, float* mat_b, float* mat_c, int M, int N, int K) {

    cublasHandle_t handle;
    // initialize CUBLAS context
    cublasStatus_t stat = cublasCreate(&handle);

    float al = 1.0f;
    float bet = 0.0f;
    
    //��ǰ�϶�ԭ����洢��ʽΪ������
    stat = cublasSgemm(handle, CUBLAS_OP_T, CUBLAS_OP_T, M, K, N, 
        &al, mat_a, N, mat_b, K, 
        &bet, mat_c, M);
    
    //��Ϊ�����ȿ��������д���
    //stat = cublasSgemm(handle, CUBLAS_OP_N, CUBLAS_OP_N, M, N, K,
    //    &al, mat_a, M, mat_b, K,
    //    &bet, mat_c, M);
    cublasDestroy(handle);
    //int row_idx = blockIdx.x;
    //int col_idx = threadIdx.x;
    //int idx = blockDim.x * blockIdx.x + threadIdx.x;
    //float c = 0;
    //for (int i = 0; i < N; i++) {
    //    c += mat_a[row_idx * N + i] * mat_b[col_idx * N + i];
    //}
    //mat_c[idx] = c;

}

__global__ void matrix_add(float* MatA, float* MatB, float* MatC, int nx, int ny)
{
    int idx = blockDim.x * blockIdx.x + threadIdx.x;
    MatC[idx] = MatA[idx] + MatB[idx];
}

__global__ void matrix_add_with_relu(float* MatA, float* MatB, float* MatC, int nx, int ny)
{
    int idx = blockDim.x * blockIdx.x + threadIdx.x;
    MatC[idx] = max(MatA[idx] + MatB[idx], 0.0f);
}

//// ��������Ԥ��Ĳ���
//extern "C" int set_ball_model_param(float* a_1_matrix_cpu, float* bias_1_matrix_cpu, float* a_2_matrix_cpu, float* bias_2_matrix_cpu) {
//
//    float* a_1_matrix_gpu, * bias_1_matrix_gpu, * a_2_matrix_gpu, * bias_2_matrix_gpu;
//
//    int result_size = OUTPUT_DIM * sizeof(float);
//    int vel_data_size = INPUT_DIM * sizeof(float);
//    int hidden_layer_size = HIDDEN_LAYER_DIM * sizeof(float);
//
//    cudaError_t status1 = cudaMemcpyToSymbol(GPU_A1_MATRIX, a_1_matrix_cpu, INPUT_DIM * HIDDEN_LAYER_DIM * sizeof(float));
//    cudaError_t status2 = cudaMemcpyToSymbol(GPU_BIAS1_MATRIX, bias_1_matrix_cpu, HIDDEN_LAYER_DIM * sizeof(float));
//    cudaError_t status3 = cudaMemcpyToSymbol(GPU_A2_MATRIX, a_2_matrix_cpu, HIDDEN_LAYER_DIM * OUTPUT_DIM * sizeof(float));
//    cudaError_t status4 = cudaMemcpyToSymbol(GPU_BIAS2_MATRIX, bias_2_matrix_cpu, OUTPUT_DIM * sizeof(float));
//
//    //for (int i = 0; i < 10; i++) {
//    //    std::cout << bias_1_matrix_cpu[i] << " ";
//    //}
//    //std::cout << std::endl << std::endl;
//
//    if (status1 != cudaSuccess || status2 != cudaSuccess || status3 != cudaSuccess || status4 != cudaSuccess)
//    {
//        printf("**************** set ball model cuda malloc dbuf error ******************* \r\n");
//        return 0;
//    }
//    return 1;
//}

// ����Ԥ�ⲿ�֣���������һЩGPU�ϵľ�������
extern "C" void ball_model_calc_with_gpu(float* vel_data_cpu, float *predict_results, float* a_1_matrix_cpu, float* bias_1_matrix_cpu, float* a_2_matrix_cpu, float* bias_2_matrix_cpu) {
    //clock_t begin, end;
    //begin = clock();
    float* results_gpu, * vel_data_gpu, * hidden_layer_data_gpu;
    float* a_1_matrix_gpu, * bias_1_matrix_gpu, * a_2_matrix_gpu, * bias_2_matrix_gpu;

    int result_size = OUTPUT_DIM * sizeof(float);
    int vel_data_size = INPUT_DIM * sizeof(float);
    int hidden_layer_size = HIDDEN_LAYER_DIM * sizeof(float);

    // ��ȡ�ռ�
    cudaError_t status1 = cudaMalloc((void**)&results_gpu, result_size); // ���ڴ洢�����Ľ��
    cudaError_t status2 = cudaMalloc((void**)&vel_data_gpu, vel_data_size);
    cudaError_t status3 = cudaMalloc((void**)&hidden_layer_data_gpu, hidden_layer_size);

    cudaError_t status4 = cudaMalloc((void**)&a_1_matrix_gpu, INPUT_DIM * HIDDEN_LAYER_DIM * sizeof(float));
    cudaError_t status5 = cudaMalloc((void**)&bias_1_matrix_gpu, HIDDEN_LAYER_DIM * sizeof(float));
    cudaError_t status6 = cudaMalloc((void**)&a_2_matrix_gpu, HIDDEN_LAYER_DIM * OUTPUT_DIM * sizeof(float));
    cudaError_t status7 = cudaMalloc((void**)&bias_2_matrix_gpu, OUTPUT_DIM * sizeof(float));
    

    if (status1 != cudaSuccess || status2 != cudaSuccess || status3 != cudaSuccess)
    {
        size_t available, total;
        cudaMemGetInfo(&available, &total);
        std::cout << "free memory: " << available << " | total memory: " << total << std::endl;
        printf("    Eror text:  %s\n", cudaGetErrorString(status1));
        printf("    Eror text:  %s\n", cudaGetErrorString(status2));
        printf("    Eror text:  %s\n", cudaGetErrorString(status3));
        printf("**************** ball model cuda malloc dbuf error ******************* \r\n");
        return;
    }
    if (status4 != cudaSuccess || status5 != cudaSuccess || status6 != cudaSuccess || status7 != cudaSuccess)
    {
        size_t available, total;
        cudaMemGetInfo(&available, &total);
        std::cout << "free memory: " << available << " | total memory: " << total << std::endl;
        printf("    Eror text:  %s\n", cudaGetErrorString(status4));
        printf("    Eror text:  %s\n", cudaGetErrorString(status5));
        printf("    Eror text:  %s\n", cudaGetErrorString(status6));
        printf("    Eror text:  %s\n", cudaGetErrorString(status7));
        printf("**************** ball model cuda param malloc dbuf error ******************* \r\n");
        return;
    }

    // ��������
    cudaError_t status_cpy1 = cudaMemcpy(vel_data_gpu, vel_data_cpu, vel_data_size, cudaMemcpyHostToDevice);
    cudaError_t status_cpy2 = cudaMemcpy(a_1_matrix_gpu, a_1_matrix_cpu, INPUT_DIM * HIDDEN_LAYER_DIM * sizeof(float), cudaMemcpyHostToDevice);
    cudaError_t status_cpy3 = cudaMemcpy(bias_1_matrix_gpu, bias_1_matrix_cpu, HIDDEN_LAYER_DIM * sizeof(float), cudaMemcpyHostToDevice);
    cudaError_t status_cpy4 = cudaMemcpy(a_2_matrix_gpu, a_2_matrix_cpu, HIDDEN_LAYER_DIM * OUTPUT_DIM * sizeof(float), cudaMemcpyHostToDevice);
    cudaError_t status_cpy5 = cudaMemcpy(bias_2_matrix_gpu, bias_2_matrix_cpu, OUTPUT_DIM * sizeof(float), cudaMemcpyHostToDevice);
    if (status_cpy1 != cudaSuccess || status_cpy2 != cudaSuccess || status_cpy3 != cudaSuccess || status_cpy4 != cudaSuccess || status_cpy5 != cudaSuccess)
    {
        printf("**************** ball model cuda cpy error ******************* \r\n");
        return;
    }

    // y=xA
    //dim3 dimGrid1(1);
    //dim3 dimBlock1(HIDDEN_LAYER_DIM);
    //matrix_multi << <dimGrid1, dimBlock1 >> > (vel_data_gpu, a_1_matrix_gpu, hidden_layer_data_gpu, 1, INPUT_DIM, HIDDEN_LAYER_DIM);
    matrix_multi(vel_data_gpu, a_1_matrix_gpu, hidden_layer_data_gpu, 1, INPUT_DIM, HIDDEN_LAYER_DIM);

    // relu(y+=b)���ӷ�����ʹ��ͬһƬ��ַ�����ǳ˷�����
    dim3 dimGrid2(1);
    dim3 dimBlock2(HIDDEN_LAYER_DIM);
    matrix_add_with_relu << <dimGrid2, dimBlock2 >> > (hidden_layer_data_gpu, bias_1_matrix_gpu, hidden_layer_data_gpu, 1, HIDDEN_LAYER_DIM);

    // y=xA
    //dim3 dimGrid3(1);
    //dim3 dimBlock3(OUTPUT_DIM);
    //matrix_multi << <dimGrid3, dimBlock3 >> > (hidden_layer_data_gpu, a_2_matrix_gpu, results_gpu, 1, HIDDEN_LAYER_DIM, OUTPUT_DIM);
    matrix_multi(hidden_layer_data_gpu, a_2_matrix_gpu, results_gpu, 1, HIDDEN_LAYER_DIM, OUTPUT_DIM);

    // y+=b���ӷ�����ʹ��ͬһƬ��ַ�����ǳ˷�����
    dim3 dimGrid4(1);
    dim3 dimBlock4(OUTPUT_DIM);
    matrix_add << <dimGrid4, dimBlock4 >> > (results_gpu, bias_2_matrix_gpu, results_gpu, 1, OUTPUT_DIM);

    // ����GPU�˼���õĽ��������CPU��
    cudaMemcpy(predict_results, results_gpu, result_size, cudaMemcpyDeviceToHost);

    // �ͷ�GPU�˵��ڴ�
    cudaFree(results_gpu);
    cudaFree(vel_data_gpu);
    cudaFree(hidden_layer_data_gpu);
    cudaFree(a_1_matrix_gpu);
    cudaFree(bias_1_matrix_gpu);
    cudaFree(a_2_matrix_gpu);
    cudaFree(bias_2_matrix_gpu);
    //end = clock();
    //std::cout << "best break point calc time (GPU): " << double(end - begin) / CLOCKS_PER_SEC * 1000 << "ms" << std::endl;
}