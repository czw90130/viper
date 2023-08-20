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
#include "State.h"

/**
 * @file CudaConstraints.h
 * @brief 定义各种物理约束结构的文件。
 * 
 * 这个文件包含多种物理模拟中使用的约束结构，包括距离约束、蒙皮约束、体积约束等。
 */

#ifdef __CUDA_ARCH__
#define CUDA_HOST_DEVICE __host__ __device__
#else
#define CUDA_HOST_DEVICE
#endif

namespace viper {

/**
 * @brief 约束的基础结构。
 * 
 * 所有约束都从这个基础结构继承。
 */
struct ConstraintBase {
    CUDA_HOST_DEVICE ConstraintBase() : enabled(true) {} ///< 默认构造函数
    bool enabled; ///< 标记此约束是否启用。
};

/**
 * @brief 距离约束。
 */
struct C_distance : public ConstraintBase {
    CUDA_HOST_DEVICE C_distance() = default;
    CUDA_HOST_DEVICE C_distance(int a, int b, float restDistance, float compliance = 0.f)
        : a(a), b(b), rDist(restDistance), compliance(compliance) {}
    int a;
    int b;
    float rDist;
    float compliance; // inverse stiffness
};

/**
 * @brief 最大距离约束。
 */
struct C_distancemax : public ConstraintBase {
    CUDA_HOST_DEVICE C_distancemax() = default;
    CUDA_HOST_DEVICE C_distancemax(int a, int b, float max_distance)
        : a(a), b(b), max_distance(max_distance) {}
    int a;
    int b;
    float max_distance;
};

/**
 * @brief 皮肤约束
 */
struct C_skinning : public ConstraintBase {
    CUDA_HOST_DEVICE C_skinning() = default;
    CUDA_HOST_DEVICE C_skinning(int i, int t0, int t1, float w0, float w1)
        : i(i), t0(t0), t1(t1), w0(w0), w1(w1) {}
    int i; ///< 皮肤索引
    int t0; ///< 初始时间
    int t1; ///< 结束时间
    float w0; ///< 初始权重
    float w1; ///< 结束权重
};

/**
 * @brief 体积约束
 */
struct C_volume : public ConstraintBase {
    // 默认构造函数
    CUDA_HOST_DEVICE C_volume() = default;
    /**
     * @brief 构造函数用于设置粒子索引、参考体积和顺从度
     * @param a 粒子索引a
     * @param b 粒子索引b
     * @param Vr 参考体积
     * @param compliance 顺从度，默认为0.0
     */
    CUDA_HOST_DEVICE C_volume(int a, int b, float Vr, float compliance = 0.0)
        : a(a), b(b), Vr(Vr), compliance(compliance) {}

    /**
     * @brief 构造函数用于根据模拟状态设置粒子索引和顺从度，并计算参考体积
     * @param a 粒子索引a
     * @param b 粒子索引b
     * @param state 当前的模拟状态，其中包含粒子的位置和半径
     * @param compliance 顺从度，默认为0.0
     */
    CUDA_HOST_DEVICE C_volume(int a, int b, const SimulationState &state, float compliance = 0.0)
        : a(a), b(b), compliance(compliance) {
        // 获取粒子a和b的半径
        float ra = state.r[a];
        float rb = state.r[b];
        
        // 计算粒子a和b之间的距离
        float d = (state.x[a] - state.x[b]).norm();
        
        // 如果距离大于一个非常小的阈值
        if (d > 1e-7f) {
            // 计算两个粒子的距离和半径之间的关系
            float e = (rb - ra) / d;
            
            // 计算L，一个与距离和半径有关的参数
            float L = d + (ra - rb) * e;
            
            // 根据上述参数，使用公式计算参考体积
            Vr = M_PIf / 3.0f *
                 ((ra * ra * ra - rb * rb * rb) * (e * e * e - 3.f * e) +
                  L * (1.f - e * e) * (ra * ra + ra * rb + rb * rb));
        } else {
            // 若粒子距离非常接近，则参考体积设置为0
            Vr = 0.f;
        }
    }

    int a; ///< 粒子索引a
    int b; ///< 粒子索引b
    float Vr; ///< 参考体积
    float compliance; ///< 顺从度
};

/**
 * @brief 体积约束2
 * 
 * 该结构体表示一个体积约束，它用于确保模拟中某些三维结构的体积保持恒定。
 * 此版本的体积约束涉及三个实体：两个粒子和一个条索，可能用于模拟柔性管状结构或其他类似的物体。
 */
struct C_volume2 : public ConstraintBase {
    
    // 默认构造函数
    CUDA_HOST_DEVICE C_volume2() = default;

    /**
     * @brief 构造函数，用于根据模拟状态和粒子索引初始化体积约束2
     * 
     * @param a 第一个粒子的索引
     * @param b 第二个粒子的索引
     * @param c 条索引，可能代表一个连接粒子a和粒子b的柔性管或线条
     * @param state 当前的模拟状态，其中包含粒子的位置和半径信息
     * @param compliance 顺从度，默认为0.0
     */
    CUDA_HOST_DEVICE C_volume2(int a, int b, int c, const SimulationState &state,
              float compliance = 0.0)
        : a(a), b(b), c(c), compliance(compliance) {
        // 计算粒子a和粒子b之间的初始距离或长度
        l0 = (state.x[a] - state.x[b]).norm();
        // 计算两个粒子的平均半径作为初始半径
        r0 = 0.5f * (state.r[a] + state.r[b]);
    }

    int a; ///< 粒子索引a
    int b; ///< 粒子索引b
    int c; ///< 条索引。代表一个连接粒子a和粒子b的柔性管或线条的索引。
    float compliance; ///< 顺从度，表示结构的柔性或硬度，值越大越柔软。
    float l0; ///< 初始长度。表示粒子a和粒子b在没有受到外部影响时的距离。
    float r0; ///< 初始半径。表示粒子a和粒子b的平均半径。
};

/**
 * @brief 弯曲约束
 * 
 * 该结构体表示一个弯曲约束，主要目的是维持两个粒子之间的旋转关系，从而产生弯曲效果。
 */
struct C_bend : public ConstraintBase {
    
    // 默认构造函数
    CUDA_HOST_DEVICE C_bend() = default;

    /**
     * @brief 构造函数，用于根据模拟状态和粒子索引初始化弯曲约束
     * 
     * @param a 第一个粒子的索引
     * @param b 第二个粒子的索引
     * @param state 当前的模拟状态，其中包含粒子的四元数（旋转信息）
     * @param compliance 顺从度，默认为0.0
     */
    CUDA_HOST_DEVICE C_bend(int a, int b, const SimulationState &state, float compliance = 0.f)
        : a(a), b(b), compliance(compliance) {
        // 计算休息位置的Darboux向量，表示两个粒子在没有外部力作用时的旋转关系
        // 这里使用第一个粒子的四元数的共轭与第二个粒子的四元数相乘得到
        darbouxRest = state.q[a].conjugate() * state.q[b];
    }

    int a; ///< 粒子索引a
    int b; ///< 粒子索引b
    Quaternion darbouxRest; ///< 休息位置的Darboux向量。这个向量描述了在没有外部力作用时，两个粒子之间的旋转关系。
    float compliance; ///< 顺从度，表示结构的柔性或硬度，值越大越柔软
};

/**
 * @brief 拉伸约束
 */
struct C_stretch : public ConstraintBase {
    CUDA_HOST_DEVICE C_stretch() = default;
    CUDA_HOST_DEVICE C_stretch(int a, int b, int c, float L, float compliance = 0.f)
        : a(a), b(b), c(c), L(L), compliance(compliance) {}

    int a;   ///< 粒子索引a particle id
    int b;   ///< 粒子索引b partile id
    int c;   ///< 条索引 pill id
    float L; ///< 休息长度 rest length
    float compliance; ///< 顺从度
};

/**
 * @brief 粒子-粒子碰撞约束
 */
struct C_collpp : public ConstraintBase {
    int a; ///< 粒子索引a
    int b; ///< 粒子索引b
};

/**
 * @brief 碰撞约束
 */
struct C_collision : public ConstraintBase {
    Vec2i a; ///< 粒子对a
    Vec2i b; ///< 粒子对b
};

/**
 * @brief 半径约束
 */
struct C_radius : public ConstraintBase {
    CUDA_HOST_DEVICE C_radius() = default;
    CUDA_HOST_DEVICE C_radius(int a, float r, float compliance = 0.0)
        : a(a), r(r), compliance(compliance) {}
    int a; ///< 粒子索引
    float r; ///< 半径
    float compliance; ///< 顺从度
};

/**
 * @brief 双拉普拉斯约束
 *
 * 该结构体描述了双拉普拉斯约束，通常用于模拟物体的表面或体积的弯曲、褶皱等效果。
 * 它使用了一个粒子索引数组以及相应的权重数组来定义约束的影响。
 */
struct C_bilap : public ConstraintBase {
    /// @brief 默认构造函数
    CUDA_HOST_DEVICE C_bilap() = default;

    /**
     * @brief 参数化构造函数
     *
     * @param A           由粒子索引构成的向量
     * @param i           当前粒子的索引
     * @param compliance  顺从度
     *
     * 根据给定的粒子索引向量和当前粒子索引，初始化权重数组和粒子索引数组。
     * 权重数组和粒子索引数组的值根据当前粒子索引i的位置进行不同的初始化。
     */
    CUDA_HOST_DEVICE C_bilap(const std::vector<int> &A, int i, float compliance = 0.0)
        : compliance(compliance) {
        int n = A.size();
        if (i == 0) {
            // 当i为0时，当前粒子是向量的第一个元素
            ids[0] = A[i];
            ids[1] = A[i];
            ids[2] = A[i];
            ids[3] = A[i + 1];
            ids[4] = A[i + 2];
            // 为这种情况分配特定的权重
            w[0] = 0.f;
            w[1] = 0.f;
            w[2] = 2.f;
            w[3] = -3.f;
            w[4] = 1.f;
        } else if (i == 1) {
            // 当i为1时，当前粒子是向量的第二个元素
            ids[0] = A[i];
            ids[1] = A[i - 1];
            ids[2] = A[i];
            ids[3] = A[i + 1];
            ids[4] = A[i + 2];
            // 为这种情况分配特定的权重
            w[0] = 0.f;
            w[1] = -3.f;
            w[2] = 6.f;
            w[3] = -4.f;
            w[4] = 1.f;
        } else if (i > 1 && i < n - 2) {
            // 当i在第三个和倒数第三个元素之间时
            ids[0] = A[i - 2];
            ids[1] = A[i - 1];
            ids[2] = A[i];
            ids[3] = A[i + 1];
            ids[4] = A[i + 2];
            // 为这种情况分配特定的权重
            w[0] = 1.f;
            w[1] = -4.f;
            w[2] = 6.f;
            w[3] = -4.f;
            w[4] = 1.f;
        } else if (i == n - 2) {
            // 当i为倒数第二个元素时
            ids[0] = A[i - 2];
            ids[1] = A[i - 1];
            ids[2] = A[i];
            ids[3] = A[i + 1];
            ids[4] = A[i];
            // 为这种情况分配特定的权重
            w[0] = 1.f;
            w[1] = -4.f;
            w[2] = 6.f;
            w[3] = -3.f;
            w[4] = 0.f;
        } else if (i == n - 1) {
            // 当i为向量的最后一个元素时
            ids[0] = A[i - 2];
            ids[1] = A[i - 1];
            ids[2] = A[i];
            ids[3] = A[i];
            ids[4] = A[i];
            // 为这种情况分配特定的权重
            w[0] = 1.f;
            w[1] = -3.f;
            w[2] = 2.f;
            w[3] = 0.f;
            w[4] = 0.f;
        }
    }

    int ids[5];       ///< 粒子索引数组，用于存储与当前粒子相关的其他粒子的索引
    float w[5];       ///< 权重数组，存储与当前粒子相关的其他粒子的权重，用于约束计算
    float compliance; ///< 顺从度，用于调整约束的强度，较高的顺从度表示约束较为柔软
};

#define SHAPE_MATCHING_MAX 11

/**
 * @brief 形状匹配约束
 */
struct C_shape : public ConstraintBase {
    
    // 默认构造函数
    CUDA_HOST_DEVICE C_shape() = default;

    /**
     * @brief 构造函数用于根据模拟状态初始化形状匹配约束
     * @param ids 由粒子索引组成的向量，这些粒子需要匹配到某一特定形状
     * @param state 当前的模拟状态，其中包含粒子的位置和半径
     * @param compliance 顺从度，默认为0.0
     */
    CUDA_HOST_DEVICE C_shape(const std::vector<int> &ids, const SimulationState &state,
            float compliance = 0.0)
        : q(Quaternion::Identity()), // 设置默认旋转量为单位四元数（无旋转）
          K(1.f),                    // 默认的刚性系数为1
          compliance(compliance) {
        
        // 获取粒子数量
        n = ids.size();
        
        // 断言确保粒子数量不超过预设的最大值
        assert(n <= SHAPE_MATCHING_MAX);
        
        // 断言确保至少有两个粒子
        assert(n > 1);

        // 计算粒子的中心点
        Vec3 center = Vec3::Zero();
        for (int i = 0; i < n; i++)
            center += state.x[ids[i]];
        center /= (float)n;

        // 初始化粒子索引数组和参考位置数组
        for (int i = 0; i < SHAPE_MATCHING_MAX; i++) {
            if (i < n) {
                id[i] = ids[i];                // 设置粒子索引
                xp[i] = state.x[ids[i]] - center; // 设置粒子相对于中心的位置
            } else {
                id[i] = 0;                     // 超出粒子数量的索引默认为0
                xp[i] = Vec3::Zero();          // 超出粒子数量的位置为0向量
            }
        }

        // 计算平均半径
        r = 0.f;
        for (int i = 0; i < n; i++)
            r += state.r[id[i]];
        r /= (float)n;
    }

    int n; ///< 粒子数量
    int id[SHAPE_MATCHING_MAX]; ///< 粒子索引数组
    Vec3 xp[SHAPE_MATCHING_MAX]; ///< 参考位置数组，代表了在没有外部力作用下粒子应该在的位置
    Quaternion q; ///< 旋转量，用于使粒子旋转到匹配形状
    float K; ///< 刚性系数，控制形状匹配的强度
    float r; ///< 半径，代表了参与形状匹配的粒子的平均半径
    float compliance; ///< 顺从度，表示结构的柔性或硬度，值越大越柔软
};


/**
 * @brief 形状匹配约束2
 * 
 * 该结构体用于确保模拟中某些三维结构的形状保持恒定，通过限制它们的旋转和变形。
 * 此版本的形状匹配约束涉及一个粒子数组以及与之对应的两个旋转量数组。
 */
struct C_shape2 : public ConstraintBase {
    
    ///< 默认构造函数
    CUDA_HOST_DEVICE C_shape2() = default;

    /**
     * @brief 构造函数，用于初始化形状匹配约束2
     * 
     * @param ids 粒子索引的数组
     * @param qas 与粒子相对应的旋转量a的数组
     * @param qbs 与粒子相对应的旋转量b的数组
     * @param state 当前的模拟状态
     */
    CUDA_HOST_DEVICE C_shape2(const std::vector<int> &ids, const std::vector<int> &qas,
             const std::vector<int> &qbs, const SimulationState &state) {
        // 获取粒子的数量
        n = ids.size();
        
        // 确保粒子数量不超过设定的最大值
        assert(n <= SHAPE_MATCHING_MAX);
        // 确保至少有两个粒子
        assert(n > 1);
        // 确保粒子数量与旋转量数组的大小匹配
        assert(n == qas.size() && n == qbs.size());

        // 初始化粒子索引以及它们的旋转量
        for (int i = 0; i < n; i++) {
            id[i] = ids[i];
            qa[i] = qas[i];
            qb[i] = qbs[i];
        }

        // 设置整体的初始旋转量为单位四元数
        q = Quaternion::Identity();
    }

    int n; ///< 粒子数量。表示受到此约束影响的粒子总数。
    int id[SHAPE_MATCHING_MAX]; ///< 粒子索引数组。存储受到此约束影响的所有粒子的索引。
    int qa[SHAPE_MATCHING_MAX]; ///< 旋转量a数组。存储每个粒子的旋转量a。
    int qb[SHAPE_MATCHING_MAX]; ///< 旋转量b数组。存储每个粒子的旋转量b。
    Quaternion q; ///< 整体旋转量。表示整体受到的旋转，用于调整粒子的相对位置。
};

/**
 * @brief 触摸约束
 */
struct C_touch : public ConstraintBase {
    CUDA_HOST_DEVICE C_touch() = default; ///< 默认构造函数
    CUDA_HOST_DEVICE C_touch(int a, int b) : a(a), b(b) {} ///< 带参数构造函数
    int a; ///< 粒子索引a
    int b; ///< 粒子索引b
};

/**
 * @brief 包含所有约束类型的集合。
 */
struct ConstraintsCPU {
    std::vector<C_distance> distance;
    std::vector<C_distancemax> distancemax;
    std::vector<C_skinning> skinning;
    std::vector<C_volume> volume;
    std::vector<C_bend> bend;
    std::vector<C_stretch> stretch;
    std::vector<C_radius> radius;
    std::vector<C_bilap> bilap;
    std::vector<C_shape> shape;
    std::vector<C_touch> touch;

    std::vector<C_volume2> volume2;
    std::vector<C_shape2> shape2;
};

} // namespace viper