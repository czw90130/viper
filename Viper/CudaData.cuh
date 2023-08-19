// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     https://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once
#include "Common.h"
#include "CudaUtils.cuh"
#include <thrust/device_vector.h>

/// 主要的命名空间，包含所有的CUDA相关数据结构和工具
namespace viper {

/**
 * @brief CUDA上下文中的模拟状态数据结构
 */
struct CudaState {
    thrust::device_vector<Vec3> x;         ///< 位置向量
    thrust::device_vector<float> r;        ///< 半径向量
    thrust::device_vector<Quaternion> q;   ///< 四元数（用于旋转）
};
/**
 * @brief 用于CUDA模拟的主数据结构
 */
struct CudaSimData {
    CudaState X;  ///< 当前状态current: 存储在当前时间步的所有粒子的位置、半径和旋转。
    CudaState Xp; ///< 上一个状态previous: 存储在上一时间步的所有粒子的位置、半径和旋转。用于时间积分和决定粒子的动力学行为。
    CudaState Xi; ///< 初始状态initial: 存储在模拟开始时的所有粒子的位置、半径和旋转。它可以用于恢复到某个状态或计算从初始位置到当前位置的变化。

    thrust::device_vector<float> w;      ///< 位置权重向量
    thrust::device_vector<float> wq;     ///< 四元数权重向量
    thrust::device_vector<float> wr;     ///< 半径权重向量

    thrust::device_vector<Matrix4> b;    ///< 当前矩阵向量
    thrust::device_vector<Matrix4> bp;   ///< 上一个矩阵向量
    thrust::device_vector<Matrix4> bi;   ///< 初始矩阵向量

    thrust::device_vector<uint8_t> xa;   ///< 扩展位置数据
    thrust::device_vector<uint8_t> qa;   ///< 扩展四元数数据
};

/**
 * @brief CUDA中的投影数据结构
 */
struct CudaProjections {
    thrust::device_vector<int> id;        ///< 粒子ID数组
    thrust::device_vector<Vec6> dx;      ///< 变化向量数组

    /**
     * @brief 调整数据大小
     * @param n 新的数据大小
     */
    void resize(int n) {
        id.resize(n);
        dx.resize(n);
    }

    /// 将所有数据置零
    void setZero() {
        thrust::fill(id.begin(), id.end(), 0);
        thrust::fill(dx.begin(), dx.end(), Vec6::Zero());
    }
};

/**
 * @brief 指向CudaSimData的指针结构
 */
struct CudaStatePtr {
    /**
     * @brief 构造函数
     * @param S 模拟数据的引用
     */
    CudaStatePtr(CudaSimData &S) {
        x = ptr(S.X.x);
        q = ptr(S.X.q);
        r = ptr(S.X.r);

        xp = ptr(S.Xp.x);
        qp = ptr(S.Xp.q);
        rp = ptr(S.Xp.r);

        xi = ptr(S.Xi.x);
        qi = ptr(S.Xi.q);
        ri = ptr(S.Xi.r);

        b = ptr(S.b);
        bp = ptr(S.bp);
        bi = ptr(S.bi);

        w = ptr(S.w);
        wq = ptr(S.wq);
        wr = ptr(S.wr);

        xa = ptr(S.xa);
        qa = ptr(S.qa);
    }

    Vec3 *x;
    Quaternion *q;
    float *r;

    Vec3 *xp;
    Quaternion *qp;
    float *rp;

    Vec3 *xi;
    Quaternion *qi;
    float *ri;

    Matrix4 *b;
    Matrix4 *bp;
    Matrix4 *bi;

    float *w;
    float *wr;
    float *wq;

    uint8_t *xa;
    uint8_t *qa;
};

/**
 * @brief 指向CudaProjections的指针结构
 */
struct CudaProjectionsPtr {
    /**
     * @brief 构造函数
     * @param P 投影数据的引用
     * @param offset 偏移量，默认为0
     */
    CudaProjectionsPtr(CudaProjections &P, int offset = 0) {
        id = ptr(P.id) + offset;
        dx = ptr(P.dx) + offset;
    }

    int *id;
    Vec6 *dx;
};

} // namespace viper