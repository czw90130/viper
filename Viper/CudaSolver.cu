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

#include "CollisionGrid.cuh"
#include "ConstraintsInfo.h"
#include "CudaConstraints.cuh"
#include "CudaData.cuh"
#include "CudaSolver.h"
#include "CudaUtils.cuh"
#include <fstream>
#include <thrust/binary_search.h>
#include <thrust/gather.h>

/// \brief 最小的半径值
#define MIN_RADIUS 0.001f
/// \brief 碰撞边界
#define COLLISION_MARGIN 0.002f

namespace viper {

/**
 * @struct CudaConstraints
 * @brief GPU中的约束数据结构.
 *
 * 存储各种约束的设备向量.
 */
struct CudaConstraints {
    thrust::device_vector<C_skinning> skinning; ///< 蒙皮约束
    thrust::device_vector<C_distance> dist; ///< 距离约束
    thrust::device_vector<C_distancemax> distmax; ///< 最大距离约束
    thrust::device_vector<C_volume> vol; ///< 体积约束
    thrust::device_vector<C_volume2> vol2; ///< 第二种体积约束
    thrust::device_vector<C_bend> bend; ///< 第二种体积约束
    thrust::device_vector<C_stretch> stretch; ///< 拉伸约束
    thrust::device_vector<C_radius> radius; ///< 半径约束
    thrust::device_vector<C_shape> shape; ///< 形状约束
    thrust::device_vector<C_shape2> shape2; ///< 第二种形状约束
    thrust::device_vector<C_touch> touch; ///< 触摸约束
    thrust::device_vector<C_bilap> bilap; ///< Bilap 约束
    thrust::device_vector<C_collpp> collpp; ///< 碰撞约束
    thrust::device_vector<C_collision> collision; ///< 碰撞约束集合
};

/**
 * @struct CudaSolverData
 * @brief 用于存储 CUDA 解算数据的结构体
 */
struct CudaSolverData {
    CudaSimData S;      ///< 状态 state
    CudaProjections Pc; ///< 每个约束的投影 projections per constraint
    CudaProjections Pp; ///< 每个粒子的投影 projections per particle
    CudaProjections Pt; ///< 临时的每个粒子的投影 projections per particle temp
    CudaConstraints C; ///< 约束 constraints
    CollisionGrid cgrid; ///< 碰撞网格

    thrust::device_vector<int> c_perm; ///< 约束的排序
};

/**
 * @struct floor_friction
 * @brief 地面摩擦结构体
 * 
 * 当粒子与地面接触时，应用摩擦力
 */
struct floor_friction {
    CudaStatePtr state; /**< CUDA状态指针 */

    /**
     * @brief 构造函数
     * @param S CUDA状态指针
     */
    floor_friction(CudaStatePtr S) : state(S) {}

    /**
     * @brief 重载操作符()
     * @param i 粒子索引
     */
    __device__ void operator()(int i) const {
        if (state.xa[i] == 0)
            return;

        bool isTouching = state.x[i][1] - state.r[i] < 1e-6f;
        if (state.w[i] > 1e-6f && isTouching) {
            float pen = min(0.f, state.x[i][1] - state.r[i]);
            Vec3 dx = state.x[i] - state.xp[i];
            Vec3 tandx = Vec3(dx[0], 0.f, dx[2]);
            float tan_norm = tandx.norm();
            float mu_s = 0.01;
            float mu_k = 3.0;
            float factor = 0.99f;

            float d = abs(pen);
            if (tan_norm > mu_s * d)
                factor = min(0.99, mu_k * d / tan_norm);

            state.x[i][1] -= pen; // normal y
            state.x[i][0] -= tandx[0] * factor; // tangential x
            state.x[i][2] -= tandx[2] * factor; // tangential z
        }
        float wall = 20.0f;
        float newx = max(-wall, min(wall, state.x[i][0]));
        float newz = max(-wall, min(wall, state.x[i][2]));

    }
};

/**
 * @struct V_integration
 * @brief 速度积分结构体
 * 
 * 使用 Verlet 积分方法计算粒子的新位置
 */
struct V_integration {
    CudaStatePtr state; /**< CUDA状态指针 */
    float dt; /**< 时间步长 */
    Vec3 gravity; /**< 重力向量 */
    float damping; /**< 阻尼系数 */

    /**
     * @brief 构造函数
     * @param S CUDA状态指针
     * @param dt 时间步长
     * @param g 重力向量
     * @param damping 阻尼系数
     */
    V_integration(CudaStatePtr S, float dt, Vec3 g, float damping)
        : state(S), dt(dt), gravity(g), damping(damping) {}

    /**
     * @brief 重载操作符()
     * @param i 粒子索引
     */
    __device__ void operator()(int i) const {
        if (state.xa[i] == 0)
            return;

        Vec3 v = Vec3::Zero();
        if (state.w[i] > 1e-6f)
            v = ((state.x[i] - state.xp[i]) / dt + dt * gravity) * damping;
        state.xp[i] = state.x[i];
        state.x[i] += dt * v;

        float dr = 0.f;
        if (state.wr[i] > 1e-6f)
            dr = (state.r[i] - state.rp[i]) * damping;
        state.rp[i] = state.r[i];
        state.r[i] += dr;
    }
};

/**
 * @brief 用于旋转速度积分的结构体。
 */
struct Vq_integration {
    CudaStatePtr state; /**< CUDA状态指针。 */
    float dt; /**< 时间间隔。 */
    float damping; /**< 阻尼系数。 */

    /**
     * @brief 构造函数。
     * @param S CUDA状态指针。
     * @param dt 时间间隔。
     * @param damping 阻尼系数。
     */
    Vq_integration(CudaStatePtr S, float dt, float damping)
        : state(S), dt(dt), damping(damping) {}

    /**
     * @brief 操作符重载函数。
     * @param i 索引。
     */
    __device__ void operator()(int i) const {
        if (state.qa[i] == 0)
            return;

        Vec3 vq = 2.f / dt * (state.qp[i].conjugate() * state.q[i]).vec();
        vq *= damping;
        Quaternion vqq;
        vqq.w() = 0.f;
        vqq.vec() = vq;
        state.qp[i] = state.q[i];
        state.q[i] =
            state.qp[i].coeffs() + 0.5f * dt * (state.qp[i] * vqq).coeffs();
        state.q[i].normalize();
    }
};

/**
 * @brief 弯曲阻尼结构体。
 */
struct bend_damping {
    CudaStatePtr state; /**< CUDA状态指针。 */
    C_bend *C; /**< 弯曲约束指针。 */
    float dt; /**< 时间间隔。 */
    float damping; /**< 阻尼系数。 */

    __device__ void operator()(int i) const {
        Quaternion &qa = state.q[C[i].a];
        Quaternion &qb = state.q[C[i].b];
        Quaternion &qap = state.qp[C[i].a];
        Quaternion &qbp = state.qp[C[i].b];

        Vec3 vqa = 2.f / dt * (qap.conjugate() * qa).vec();
        Vec3 vqb = 2.f / dt * (qbp.conjugate() * qb).vec();

        Vec3 dv = (vqb - vqa) * (1.0f - damping);

        vqa += dv;
        vqb -= dv;

        Quaternion vqaq, vqbq;
        vqaq.w() = 0.f;
        vqbq.w() = 0.f;

        vqaq.vec() = vqa;
        vqbq.vec() = vqb;

        qa = qap.coeffs() + 0.5f * dt * (qap * vqaq).coeffs();
        qb = qbp.coeffs() + 0.5f * dt * (qbp * vqbq).coeffs();

        qa.normalize();
        qb.normalize();
    }
};

/**
 * @brief 对于粒子应用投影的结构体。
 */
struct apply_projection_particles {
    Vec3 *x; /**< 位置向量指针。 */
    float *r; /**< 半径数组指针。 */
    Vec6 *dx; /**< 变化量指针。 */
    int *id; /**< ID数组指针。 */
    uint8_t *a; /**< 状态数组指针。 */

    /**
     * @brief 构造函数。
     */
    apply_projection_particles(Vec3 *x, float *r, Vec6 *dx, int *id, uint8_t *a)
        : x(x), r(r), dx(dx), id(id), a(a) {}

    /**
     * @brief 设备上的操作符重载函数。
     * @param i 索引。
     */
    __device__ void operator()(int i) const {
        int k = id[i];
        if (a[k] == 0)
            return;

        if (dx[i][4] > 1e-6f)
            x[k] += dx[i].head<3>() / dx[i][4];

        if (dx[i][5] > 1e-6f)
            r[k] = fmaxf(MIN_RADIUS, r[k] + dx[i][3] / dx[i][5]);
    }
};

/**
 * @brief 对于帧应用投影的结构体。
 */
struct apply_projection_frames {
    Quaternion *x; /**< 四元数指针。 */
    Vec6 *dx; /**< 变化量指针。 */
    int *id; /**< ID数组指针。 */
    int N; /**< 数量。 */
    uint8_t *a; /**< 状态数组指针。 */

    /**
     * @brief 构造函数。
     */
    apply_projection_frames(Quaternion *x, Vec6 *dx, int *id, uint8_t *a, int N)
        : x(x), dx(dx), id(id), N(N), a(a) {}

    /**
     * @brief 设备上的操作符重载函数。
     * @param i 索引。
     */
    __device__ void operator()(int i) const {
        int k = id[i] - N;
        if (a[k] == 0)
            return;

        if (dx[i][4] > 1e-6f)
            x[k].coeffs() += dx[i].head<4>() / dx[i][4];
        x[k].normalize();
    }
};

/**
 * @brief 产生pill代理的结构体。
 */
struct generate_pills_proxys {
    Vec3 *x;      /**< 位置向量指针。 */
    float *r;     /**< 半径数组指针。 */
    Vec2i *pills; /**< pill数组指针。 */
    Vec3 *sx;     /**< 代理位置向量指针。 */
    float *sr;    /**< 代理半径数组指针。 */

    /**
     * @brief 构造函数。
     */
    generate_pills_proxys(Vec3 *x, float *r, Vec2i *pills, Vec3 *sx, float *sr)
        : x(x), r(r), pills(pills), sx(sx), sr(sr) {}

    /**
     * @brief 设备上的操作符重载函数。
     * @param i 索引。
     */
    __device__ void operator()(int i) const {
        int a = pills[i][0];
        int b = pills[i][1];
        Vec3 s0 = x[a];
        Vec3 s1 = x[b];
        float r0 = r[a];
        float r1 = r[b];
        Vec3 d = s1 - s0;
        float l = d.norm();
        Vec3 dl = d / (l + FLT_EPSILON);
        sx[i] = (s1 + s0 + dl * (r1 - r0)) / 2;
        sr[i] = (l + r0 + r1) / 2;
    }
};

/**
 * @brief 产生碰撞的结构体。
 */
struct generate_collisions {
     Vec2i *pills;        /**< pill数组指针。 */
    const Vec2i *coll_pairs; /**< 碰撞对数组指针。 */
    C_collision *C;          /**< 碰撞指针。 */

    /**
     * @brief 构造函数。
     */
    generate_collisions(Vec2i *pills, const Vec2i *coll_pairs, C_collision *C)
        : pills(pills), coll_pairs(coll_pairs), C(C) {}

    /**
     * @brief 设备上的操作符重载函数。
     * @param i 索引。
     */
    __device__ void operator()(int i) const {
        int a = coll_pairs[i][0];
        int b = coll_pairs[i][1];
        C[i].a = pills[a];
        C[i].b = pills[b];
        C[i].enabled = true;
    }
};

/**
 * @brief 碰撞过滤结构体。
 */
struct collision_filter {
    Vec2i *pills;  /**< pill数组指针。 */
    int *group;    /**< 组数组指针。 */
    CudaStatePtr S; /**< CUDA状态指针。 */

    /**
     * @brief 构造函数。
     */
    collision_filter(Vec2i *pills, int *group, CudaStatePtr S)
        : pills(pills), group(group), S(S) {}

    /**
     * @brief 设备上的操作符重载函数。
     * @param c 碰撞对。
     * @return 是否满足条件。
     */
    __device__ bool operator()(const Vec2i &c) {
        int a0 = pills[c[0]][0];
        int a1 = pills[c[0]][1];
        int b0 = pills[c[1]][0];
        int b1 = pills[c[1]][1];
        int zeroa = S.w[a0] < 1e-6f || S.w[a1] < 1e-6f;
        int zerob = S.w[b0] < 1e-6f || S.w[b1] < 1e-6f;

        return group[c[0]] != group[c[1]] && (zeroa + zerob < 2);
    }
};

CudaSolver::CudaSolver() { gpu = new CudaSolverData(); }

CudaSolver::~CudaSolver() {
    // delete gpu;
}

/**
 * @brief 用于判断给定约束是否被禁用的谓词结构体。
 * 
 * 该结构体定义了一个函数对象，用于判断给定类型 T 的约束是否被禁用。
 * 
 * @tparam T 约束的数据类型
 */
template <typename T> struct DisabledPredicate {
    /**
     * @brief 判断约束是否被禁用。
     * 
     * @param constraint 要进行判断的约束。
     * @return 返回 true 如果约束被禁用，否则返回 false。
     */
    __host__ __device__ bool operator()(const T &constraint) { return !constraint.enabled; }
};

/**
 * @brief 将数据从 CPU 端向 GPU 端上传并过滤禁用的约束。
 * 
 * 该函数首先将数据从 CPU 端向 GPU 端上传，然后过滤掉被禁用的约束。
 * 
 * @tparam CPUVec CPU 端约束容器的类型。
 * @tparam GPUVec GPU 端约束容器的类型。
 * @param gpu_vec GPU 端约束容器的引用。
 * @param cpu_vec CPU 端约束容器的引用。
 */
template <typename CPUVec, typename GPUVec>
void upload_and_filter(GPUVec &gpu_vec, const CPUVec &cpu_vec) {
    using T = typename GPUVec::value_type;
    gpu_vec = cpu_vec;
    if (gpu_vec.size() > 0) {
        gpu_vec.erase(thrust::remove_if(thrust::device, gpu_vec.begin(),
                                        gpu_vec.end(), DisabledPredicate<T>()),
                      gpu_vec.end());
    }
}

/**
 * @brief 使用 CUDA 在 GPU 上求解物理约束。
 *
 * 步骤:
 * 1. 定义和初始化在 GPU 上使用的数据结构，这些数据结构是专门为 CUDA 设计的。
 * 2. 从 CPU 传输模拟状态到 GPU。
 * 3. 定义粒子和药丸的数量。
 * 4. 上传并筛选约束数据。
 * 5. 开始计时。
 * 6. 进行时间积分，使用 thrust 进行各种 GPU 运算：
 *    - 对每个粒子进行速度积分。
 *    - 对每个药丸进行旋转速度积分。
 *    - 对 "skinning" 相关元素进行某种求解。
 * 7. 创建设备向量，用于存储药丸的代理数据。
 * 8. 处理物体之间的碰撞，如果存在足够的药丸（至少2个），则检查碰撞对，过滤无效碰撞对，并生成碰撞数据。
 * 9. 停止计时并开始计算约束信息。
 * 10. 初始化约束的投影值，XPBD（扩展的位置基础动力学）约束列表，并进行弯曲阻尼计算。
 * 11. 进行指定次数的迭代，以满足所有约束：
 *     - 对各种约束（如距离、体积、弯曲等）进行求解。
 *     - 处理物体之间的碰撞。
 *     - 使用预先构建的排列，对投影数据进行排序并收集。
 *     - 使用排序后的数据，将相同的投影信息合并。
 *     - 使用计算出的投影信息来更新物体的位置和速度。
 *     - 如果启用了地面处理，那么进行地面摩擦处理。
 * 12. 将结果返回到CPU。
 * 13. 返回该函数的运行时间。
 * 
 * @param[in] constraints     CPU上的约束信息。
 * @param[in,out] state       模拟状态，输入为当前状态，输出为模拟后的状态。
 * @param[in] pills           碰撞检测用的药丸向量。
 * @param[in] group           碰撞检测分组。
 * @param[in] dt              时间步长。
 * @param[in] g               重力向量。
 * @param[in] iterations      求解迭代次数。
 * @param[in] floor           是否包括地面碰撞。
 * @param[in] damping         阻尼系数。
 * 
 * @return 返回求解过程的运行时间。
 */
double CudaSolver::solve(ConstraintsCPU &constraints, SimulationState &state,
                         const std::vector<Vec2i> &pills,
                         const std::vector<int> &group, float dt, const Vec3 &g,
                         int iterations, bool floor, float damping) {
    // 定义并初始化 GPU 数据结构
    CudaConstraints &C = gpu->C;
    CudaSimData &S = gpu->S;
    CudaProjections &Pc = gpu->Pc;
    CudaProjections &Pt = gpu->Pt;
    CudaProjections &Pp = gpu->Pp;

    // 从 CPU 传输数据到 GPU
    // CPU -> GPU
    S.X.x = state.x;
    S.X.q = state.q;
    S.X.r = state.r;

    S.Xp.x = state.xp;
    S.Xp.q = state.qp;
    S.Xp.r = state.rp;

    S.Xi.x = state.xi;
    S.Xi.q = state.qi;
    S.Xi.r = state.ri;

    S.b = state.b;
    S.bp = state.bp;
    S.bi = state.bi;

    S.w = state.w;
    S.wq = state.wq;
    S.wr = state.wr;

    S.xa = state.xa;
    S.qa = state.qa;

    thrust::device_vector<Vec2i> gpu_pills = pills;
    thrust::device_vector<int> pill_groups = group;

    int N = state.x.size(); // particles count 粒子数量
    int M = state.q.size(); // pills count 药丸数量

    // 上传并筛选约束数据
    upload_and_filter(C.dist, constraints.distance);
    upload_and_filter(C.distmax, constraints.distancemax);
    upload_and_filter(C.skinning, constraints.skinning);
    upload_and_filter(C.vol, constraints.volume);
    upload_and_filter(C.vol2, constraints.volume2);
    upload_and_filter(C.bend, constraints.bend);
    upload_and_filter(C.stretch, constraints.stretch);
    upload_and_filter(C.bilap, constraints.bilap);
    upload_and_filter(C.shape, constraints.shape);
    upload_and_filter(C.shape2, constraints.shape2);
    upload_and_filter(C.radius, constraints.radius);
    upload_and_filter(C.touch, constraints.touch);

    tic();

    // time integration 时间积分部分
    // 使用 thrust 进行各种 GPU 运算

    // 对于0到N的每个数字，调用 速度积分(V_integration)函数
    thrust::for_each(thrust::device, thrust::make_counting_iterator(0),
                     thrust::make_counting_iterator(N),
                     V_integration(CudaStatePtr(S), dt, g, damping));

    // 对于0到M的每个数字，调用旋转速度积分(Vq_integration)函数
    thrust::for_each(thrust::device, thrust::make_counting_iterator(0),
                     thrust::make_counting_iterator(M),
                     Vq_integration(CudaStatePtr(S), dt, damping));

    // 对于C.skinning中每个元素，调用C_skinning_solve函数
    // 此函数似乎解决了某种与"skinning"相关的问题
    thrust::for_each(thrust::device, thrust::make_counting_iterator(0),
                     thrust::make_counting_iterator((int)C.skinning.size()),
                     C_skinning_solve(ptr(C.skinning), CudaStatePtr(S)));

    float t_velocity = toc();

    // 为每个元素创建设备向量（GPU上的向量）
    thrust::device_vector<Vec3> sp(M);
    thrust::device_vector<float> sr(M);

    // 对于0到M的每个数字，调用 产生pill代理(generate_pills_proxys)函数
    thrust::for_each(thrust::device, thrust::make_counting_iterator(0),
                     thrust::make_counting_iterator(M),
                     generate_pills_proxys(ptr(S.X.x), ptr(S.X.r),
                                           ptr(gpu_pills), ptr(sp), ptr(sr)));
    // 处理碰撞
    // 如果M小于2，说明不存在可能的碰撞对
    if (M < 2) {
        C.collision.resize(0);
    } else {
        // 使用gpu->cgrid.test_particles测试可能的碰撞对
        const thrust::device_vector<Vec2i> &coll_pairs =
            gpu->cgrid.test_particles(sp, sr, COLLISION_MARGIN);
        
        // 创建一个新的设备向量，用于存储过滤后的碰撞对
        thrust::device_vector<Vec2i> coll_pairs_filtered(coll_pairs.size());
        
        // 从原始碰撞对列表中复制有效的碰撞对
        auto valid_coll_end =
            thrust::copy_if(thrust::device, coll_pairs.begin(),
                            coll_pairs.end(), coll_pairs_filtered.begin(),
                            collision_filter(ptr(gpu_pills), ptr(pill_groups),
                                             CudaStatePtr(S)));
        
        // 删除无效的碰撞对
        coll_pairs_filtered.erase(valid_coll_end, coll_pairs_filtered.end());
        
        int K = coll_pairs_filtered.size();
        
        // 调整C.collision的大小以匹配过滤后的碰撞对数量
        C.collision.resize(K);
        
        // 对于每一个有效的碰撞对，调用generate_collisions生成相应的碰撞数据
        thrust::for_each(thrust::device, thrust::make_counting_iterator(0),
                         thrust::make_counting_iterator(K),
                         generate_collisions(ptr(gpu_pills),
                                             ptr(coll_pairs_filtered),
                                             ptr(C.collision)));
    }

    // 结束上一个操作的计时器并开始一个新的计时器来衡量下一段操作的执行时间。
    float t_collision = toc();

    // 定义一个新的结构用于存储约束信息。
    ConstraintsInfo cInfo;

    // 添加不同的约束信息到cInfo对象中。每一种约束都有其自己的数量、操作数和处理方式。
    // 这些参数定义了各种约束的特性。
    cInfo.add("distance", C.dist.size(), 2, 1);
    cInfo.add("volume", C.vol.size(), 2, 1);
    cInfo.add("volume2", C.vol2.size(), 3, 1);
    cInfo.add("bend", C.bend.size(), 2, 3);
    cInfo.add("stretch", C.stretch.size(), 3, 3);
    cInfo.add("bilap", C.bilap.size(), 1, 1);
    cInfo.add("shape", C.shape.size(), SHAPE_MATCHING_MAX, 1);
    cInfo.add("shape2", C.shape2.size(), 3 * SHAPE_MATCHING_MAX, 1);
    cInfo.add("radius", C.radius.size(), 1, 1);
    cInfo.add("collision", C.collision.size(), 4, 0);
    cInfo.add("touch", C.touch.size(), 2, 0);

    // 获取总的约束点数量和处理数量。
    int np = cInfo.get_np();
    int nl = cInfo.get_nl();
    // 获取约束的排序（offsets）。
    std::map<std::string, int> o = cInfo.get_o();
    std::map<std::string, int> ol = cInfo.get_ol();

    // 计算总的约束数量。
    int n_cst = C.dist.size() + C.vol.size() + C.bend.size() +
                C.stretch.size() + C.shape.size() + C.radius.size() +
                C.collision.size() + C.vol2.size() + C.shape2.size();

    // 标记是否构建了排列，初始化为false。
    bool permutation_built = false;

    // 初始化约束的投影值。
    Pc.resize(np);
    Pt.resize(np);
    Pp.resize(N + M);
    // 初始化XPBD（扩展的位置基础动力学）约束列表。
    thrust::device_vector<float> L(nl); // XPBD
    thrust::fill(L.begin(), L.end(), 0.f);
    // 弯曲阻尼计算。
    thrust::for_each(thrust::device, thrust::make_counting_iterator(0),
                     thrust::make_counting_iterator((int)C.bend.size()),
                     bend_damping{CudaStatePtr(S), ptr(C.bend), dt, 0.98f});

    // 进行指定次数的迭代，以满足所有约束。
    for (int i = 0; i < iterations; i++) {
        // 标记某些迭代只处理碰撞。
        bool collisions_only = (i % 2 == 1) || i == -1;
        
        // 重置投影的累计值。
        Pc.setZero();
        Pp.setZero();
        
        // 如果不仅仅处理碰撞，则计算和应用其他的约束。
        if (!collisions_only) {

            // 处理各种约束（如距离、体积、弯曲等）。
            thrust::for_each(
                thrust::device, thrust::make_counting_iterator(0),
                thrust::make_counting_iterator((int)C.dist.size()),
                C_distance_solve(ptr(C.dist), CudaStatePtr(S),
                                 CudaProjectionsPtr(Pc, o["distance"]),
                                 ptr(L, ol["distance"]), dt));
            thrust::for_each(thrust::device, thrust::make_counting_iterator(0),
                             thrust::make_counting_iterator((int)C.vol.size()),
                             C_volume_solve(ptr(C.vol), CudaStatePtr(S),
                                            CudaProjectionsPtr(Pc, o["volume"]),
                                            ptr(L, ol["volume"]), dt));
            thrust::for_each(
                thrust::device, thrust::make_counting_iterator(0),
                thrust::make_counting_iterator((int)C.vol2.size()),
                C_volume2_solve(ptr(C.vol2), CudaStatePtr(S),
                                CudaProjectionsPtr(Pc, o["volume2"]),
                                ptr(L, ol["volume2"]), dt));
            thrust::for_each(thrust::device, thrust::make_counting_iterator(0),
                             thrust::make_counting_iterator((int)C.bend.size()),
                             C_bend_solve(ptr(C.bend), CudaStatePtr(S),
                                          CudaProjectionsPtr(Pc, o["bend"]), N,
                                          ptr(L, ol["bend"]), dt));
            thrust::for_each(
                thrust::device, thrust::make_counting_iterator(0),
                thrust::make_counting_iterator((int)C.stretch.size()),
                C_stretch_solve(ptr(C.stretch), CudaStatePtr(S),
                                CudaProjectionsPtr(Pc, o["stretch"]), N,
                                ptr(L, ol["stretch"]), dt));
            thrust::for_each(
                thrust::device, thrust::make_counting_iterator(0),
                thrust::make_counting_iterator((int)C.bilap.size()),
                C_bilap_solve(ptr(C.bilap), CudaStatePtr(S),
                              CudaProjectionsPtr(Pc, o["bilap"]),
                              ptr(L, ol["bilap"]), dt));
            thrust::for_each(
                thrust::device, thrust::make_counting_iterator(0),
                thrust::make_counting_iterator((int)C.shape.size()),
                C_shape_solve(ptr(C.shape), CudaStatePtr(S),
                              CudaProjectionsPtr(Pc, o["shape"]),
                              ptr(L, ol["shape"]), dt));
            thrust::for_each(
                thrust::device, thrust::make_counting_iterator(0),
                thrust::make_counting_iterator((int)C.shape2.size()),
                C_shape2_solve(ptr(C.shape2), CudaStatePtr(S),
                               CudaProjectionsPtr(Pc, o["shape2"]), N,
                               ptr(L, ol["shape2"]), dt));
            thrust::for_each(
                thrust::device, thrust::make_counting_iterator(0),
                thrust::make_counting_iterator((int)C.radius.size()),
                C_radius_solve(ptr(C.radius), CudaStatePtr(S),
                               CudaProjectionsPtr(Pc, o["radius"]),
                               ptr(L, ol["radius"]), dt));
            thrust::for_each(
                thrust::device, thrust::make_counting_iterator(0),
                thrust::make_counting_iterator((int)C.touch.size()),
                C_touch_solve(ptr(C.touch), CudaStatePtr(S),
                              CudaProjectionsPtr(Pc, o["touch"]),
                              ptr(L, ol["touch"]), dt));
        }

        // 处理物体之间的碰撞
        thrust::for_each(
            thrust::device,
            thrust::make_counting_iterator(0),
            thrust::make_counting_iterator((int)C.collision.size()),
            C_collision_solve(ptr(C.collision), CudaStatePtr(S),
                              CudaProjectionsPtr(Pc, o["collision"])));
        
        // 判断是否已经构建排列，这个排列在处理多个约束时很有用
        if (!permutation_built) {
            // 如果没有构建，那么开始构建
            gpu->c_perm.resize(np); // 重新设置存储排列的大小
            // 创建一个序列
            thrust::sequence(thrust::device, gpu->c_perm.begin(),
                             gpu->c_perm.end()); 
            auto vals_begin = thrust::make_zip_iterator(
                thrust::make_tuple(Pc.dx.begin(), gpu->c_perm.begin()));
            // 根据id对投影数据进行排序
            thrust::sort_by_key(Pc.id.begin(), Pc.id.end(), vals_begin);
            Pt = Pc; // 复制当前投影信息
            permutation_built = true; // 标记排列已构建
        } else {
            // 如果排列已经构建，那么我们使用这个排列来收集数据
            auto src_begin = thrust::make_zip_iterator(
                thrust::make_tuple(Pc.dx.begin(), Pc.id.begin()));
            auto dst_begin = thrust::make_zip_iterator(
                thrust::make_tuple(Pt.dx.begin(), Pt.id.begin()));
            thrust::gather(thrust::device, gpu->c_perm.begin(),
                           gpu->c_perm.end(), src_begin, dst_begin);
        }

        // 根据排序后的id集合和投影数据，将相同的id合并到一起
        auto new_end =
            thrust::reduce_by_key(thrust::device, Pt.id.begin(), Pt.id.end(),
                                  Pt.dx.begin(), Pp.id.begin(), Pp.dx.begin())
                .first;
        auto f_start = thrust::lower_bound(Pp.id.begin(), new_end, N);
        int proj_count = new_end - Pp.id.begin();
        int p_count = f_start - Pp.id.begin();

        // 对于每个粒子，使用之前计算的投影信息来更新它们的位置和速度
        thrust::for_each(thrust::device, thrust::make_counting_iterator(0),
                         thrust::make_counting_iterator(p_count),
                         apply_projection_particles(ptr(S.X.x), ptr(S.X.r),
                                                    ptr(Pp.dx), ptr(Pp.id),
                                                    ptr(S.xa)));
        thrust::for_each(thrust::device,
                         thrust::make_counting_iterator(p_count),
                         thrust::make_counting_iterator(proj_count),
                         apply_projection_frames(ptr(S.X.q), ptr(Pp.dx),
                                                 ptr(Pp.id), ptr(S.qa), N));

        // 如果存在地板，将对物体进行地面摩擦处理，这样物体不会无限制地滑动
        if (floor)
            thrust::for_each(thrust::device, thrust::make_counting_iterator(0),
                             thrust::make_counting_iterator(N),
                             floor_friction(CudaStatePtr(S)));
    }

    // 结束此段操作的计时器，并返回执行时间。
    float t_solve = toc();

    // 从 GPU 传输数据回 CPU
    // GPU -> CPU
    thrust::copy(S.X.x.begin(), S.X.x.end(), state.x.begin());
    thrust::copy(S.Xp.x.begin(), S.Xp.x.end(), state.xp.begin());
    thrust::copy(S.X.q.begin(), S.X.q.end(), state.q.begin());
    thrust::copy(S.Xp.q.begin(), S.Xp.q.end(), state.qp.begin());
    thrust::copy(S.X.r.begin(), S.X.r.end(), state.r.begin());
    thrust::copy(S.Xp.r.begin(), S.Xp.r.end(), state.rp.begin());
    thrust::copy(C.shape.begin(), C.shape.end(), constraints.shape.begin());
    thrust::copy(C.shape2.begin(), C.shape2.end(), constraints.shape2.begin());

    return t_solve;
}

} // namespace viper