#include "device_launch_parameters.h"
#include <iostream>
#include "cuda_runtime.h"
#include "time.h"
#include "param.h"

//namespace {
//    const float PI = 3.1415926;
//    const float M_2PI = PI * 2;
//    const float halfLength = 600;
//    float goal_pos[2] = { halfLength, 0 };
//    const float POINT_PASS_OFF = 200;
//}

__constant__ float PI = 3.1415926;
__constant__ float M_2PI = 6.2831852;
__constant__ float PITCH_WIDTH = 900;
__constant__ float PITCH_LENGTH = 1200;
__constant__ float PENALTY_WIDTH = 240;
__constant__ float PENALTY_DEPTH = 120;
__constant__ float POINT_PASS_OFF = 80.0;
__constant__ int ENEMY_NUM = 8;

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
        if (their_player_ptr[i * 6]) {
            // �򵽵з��ľ���
            float* their_palyer_pos = their_player_ptr + i * 6 + 1;
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
        if (their_player_ptr[i * 6]) {
            // ���ŵ��з��ľ���
            float* their_palyer_pos = their_player_ptr + i * 6 + 1;
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
        float* their_player_ptr = startPos + (7 + 6 * 8);
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
    float* map_gpu, * start_pos_gpu;

    int map_size = height * width * sizeof(float);
    int pos_size = pos_num * sizeof(float);

    cudaError_t  status1 = cudaMalloc((void**)&map_gpu, map_size); // ���ڴ洢�����Ľ��
    cudaError_t  status2 = cudaMalloc((void**)&start_pos_gpu, pos_size);
    if (status1 != cudaSuccess || status2 != cudaSuccess)
    {
        printf("****************cuda malloc dbuf error ******************* \r\n");
        return;
    }

    // ��������
    cudaError_t status_cpy = cudaMemcpy(start_pos_gpu, start_pos_cpu, pos_size, cudaMemcpyHostToDevice);
    if (status_cpy != cudaSuccess)
    {
        printf("****************cuda cpy error ******************* \r\n");
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
}
