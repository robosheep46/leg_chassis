/**
 ******************************************************************************
 * @file	 user_lib.c
 * @author  Wang Hongxi
 * @author  modified by neozng
 * @version 0.2 beta
 * @date    2021/2/18
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "stdlib.h"
#include "memory.h"
#include "user_lib.h"
#include "math.h"
#include "main.h"


#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif

void *zmalloc(size_t size)
{
    void *ptr = malloc(size);
    memset(ptr, 0, size);
    return ptr;
}

// 快速开方
float Sqrt(float x)
{
    float y;
    float delta;
    float maxError;

    if (x <= 0)
    {
        return 0;
    }

    // initial guess
    y = x / 2;

    // refine
    maxError = x * 0.001f;

    do
    {
        delta = (y * y) - x;
        y -= delta / (2 * y);
    } while (delta > maxError || delta < -maxError);

    return y;
}

// 绝对值限制
float abs_limit(float num, float Limit)
{
    if (num > Limit)
    {
        num = Limit;
    }
    else if (num < -Limit)
    {
        num = -Limit;
    }
    return num;
}

// 判断符号位
float sign(float value)
{
    if (value >= 0.0f)
    {
        return 1.0f;
    }
    else
    {
        return -1.0f;
    }
}

// 浮点死区
float float_deadband(float Value, float minValue, float maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0.0f;
    }
    return Value;
}

// 限幅函数
float float_constrain(float Value, float minValue, float maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

// 限幅函数
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

// 循环限幅函数
float loop_float_constrain(float Input, float minValue, float maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        float len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        float len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}

// 弧度格式化为-PI~PI

// 角度格式化为-180~180
float theta_format(float Ang)
{
    return loop_float_constrain(Ang, -180.0f, 180.0f);
}

int float_rounding(float raw)
{
    static int integer;
    static float decimal;
    integer = (int)raw;
    decimal = raw - integer;
    if (decimal > 0.5f)
        integer++;
    return integer;
}

// 三维向量归一化
float *Norm3d(float *v)
{
    float len = Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    v[0] /= len;
    v[1] /= len;
    v[2] /= len;
    return v;
}

// 计算模长
float NormOf3d(float *v)
{
    return Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

// 三维向量叉乘v1 x v2
void Cross3d(float *v1, float *v2, float *res)
{
    res[0] = v1[1] * v2[2] - v1[2] * v2[1];
    res[1] = v1[2] * v2[0] - v1[0] * v2[2];
    res[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

// 三维向量点乘
float Dot3d(float *v1, float *v2)
{
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

// 均值滤波,删除buffer中的最后一个元素,填入新的元素并求平均值
float AverageFilter(float new_data, float *buf, uint8_t len)
{
    float sum = 0;
    for (uint8_t i = 0; i < len - 1; i++)
    {
        buf[i] = buf[i + 1];
        sum += buf[i];
    }
    buf[len - 1] = new_data;
    sum += new_data;
    return sum / len;
}

void MatInit(mat *m, uint8_t row, uint8_t col)
{
    m->numCols = col;
    m->numRows = row;
    m->pData = (float *)zmalloc(row * col * sizeof(float));
}

/* General type conversion for MATLAB generated C-code  */

const double MATLAB_B[44] = {
   -0.00230858256273,-0.009325171223315, -0.01513487448342,-0.008400928532814,
   0.006833363855991, 0.008519032993693,-0.006865771039861,  -0.0102679770405,
   0.008664362393225,  0.01337506656253, -0.01154425798558,      -0.017835896,
    0.01588276158906,   0.0241479874974, -0.02263140090351, -0.03371810569548,
    0.03420653587077,  0.05059305692987, -0.05847486308356, -0.09208888134333,
     0.1453154498845,   0.4536214846252,   0.4536214846252,   0.1453154498845,
   -0.09208888134333, -0.05847486308356,  0.05059305692987,  0.03420653587077,
   -0.03371810569548, -0.02263140090351,   0.0241479874974,  0.01588276158906,
        -0.017835896, -0.01154425798558,  0.01337506656253, 0.008664362393225,
    -0.0102679770405,-0.006865771039861, 0.008519032993693, 0.006833363855991,
  -0.008400928532814, -0.01513487448342,-0.009325171223315, -0.00230858256273
};
// 滤波器结构体

// 初始化滤波器状态
void initializeFilter(Filter* filter) {
    for (int i = 0; i < MATLAB_BL; i++) {
        filter->x[i] = 0.0;
    }
    filter->y = 0.0;
}

// 应用滤波器
double applyFilter(double input,  Filter* filter) {
    // 更新输入缓冲区
    for (int i = MATLAB_BL - 1; i > 0; i--) {
        filter->x[i] = filter->x[i - 1];
    }
    filter->x[0] = input;

    // 计算输出
    filter->y = 0.0;
    for (int i = 0; i < MATLAB_BL; i++) {
        filter->y += MATLAB_B[i] * filter->x[i];
    }

    return filter->y;
}

void dot_product(int m, int n, double A[m][n], double B[m][n], double C[m][n]) {
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            C[i][j] = A[i][j] * B[i][j];
        }
    }
}