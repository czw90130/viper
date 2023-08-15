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

#include <unordered_map>

#include "Common.h"
#include "CudaConstraints.h"
#include "CudaSolver.h"
#include "Mesh.h"
#include "Particle.h"
#include "Rod.h"

// 推断关于particle、pill和rod之间的关系：

// Particle（粒子）:
// 在物理模拟中，粒子是基本的模拟单位。它有一定的位置、速度、半径、权重等属性。
// 通过addParticle函数，我们可以在场景中添加一个新的粒子。这个粒子可以是动力学的或者是静态的（取决于kinematic参数）。

// Pill（药丸形状）:
// Pill在这里似乎代表一个连接两个粒子的形状，可能是某种形式的弹簧或约束。
// 通过addPill函数，我们可以在两个粒子之间添加一个Pill。Pill的方向可能是由两个粒子的相对位置确定的。
// 每一个Pill由两个粒子a和b定义，而且这两个粒子的信息会被更新为属于“纤维”(isFiber)。

// Rod（杆）:
// Rod可能是由一系列的粒子组成，这些粒子通过Pills连接在一起，形成了一个刚性或柔性的杆状结构。
// 通过addRod函数，我们可以添加一个由多个粒子组成的Rod到场景中。这个Rod有一系列的属性，如起始位置、每个粒子之间的距离、杆的半径等。
// 在Rod中，每两个相邻的粒子都通过一个Pill连接。
// Rod也可以有一些物理约束，如拉伸、弯曲和双边约束。

// 总结:
// Particle是模拟的基本单位。
// Pill是连接两个粒子的形状或约束，可能表示某种弹簧或链接。
// Rod是由多个粒子组成的结构，其中每两个相邻的粒子都通过Pill连接。

/**
 * @brief 主要的场景管理命名空间
 */
namespace viper {

/**
 * @brief 场景结构体，用于管理和模拟物理场景
 */
struct Scene {
    /// @brief 默认构造函数
    Scene();

    /// @brief 析构函数
    ~Scene();

    /**
     * @brief 添加一个粒子到场景中
     * @param p 粒子的位置，默认为原点
     * @param r 粒子的半径，默认为 PARTICLE_RADIUS
     * @param w 粒子的权重，默认为 1.0f
     * @param kinematic 粒子是否为动力学的，默认为 false
     * @return 新添加的粒子的ID
     */
    Id addParticle(const Vec3 &p = Vec3::Zero(), float r = PARTICLE_RADIUS,
                   float w = 1.0f, bool kinematic = false);
    
    /**
     * @brief 在两个粒子之间添加一个pill
     * @param a 第一个粒子的ID
     * @param b 第二个粒子的ID
     * @param kinematic pill是否为动力学的，默认为false
     * @return 新添加的pill的ID
     */
    Id addPill(Id a, Id b, bool kinematic = false);

    /**
     * @brief 添加一个杆到场景中
     * @param n 杆中的粒子数量
     * @param start 杆的起始位置
     * @param step 杆中每个粒子之间的间隔
     * @param r 杆的半径
     * @param volume 是否有体积约束
     * @param pIds 返回的粒子ID列表
     * @param rIds 返回的pill ID列表
     * @param stretch_eta 拉伸的参数
     * @param volume_eta 体积的参数
     * @param bilap_eta 双边参数
     */
    void addRod(int n, Vec3 start, Vec3 step, float r, bool volume,
                IntArray &pIds, IntArray &rIds, float stretch_eta = 0.0f,
                float volume_eta = 0.0f, float bilap_eta = 0.0f);

    /**
     * @brief 执行一步物理模拟
     * @param timestep 时间步长
     * @param iterations 迭代次数，默认为20
     * @param floor 是否有地面，默认为false
     * @param damping 阻尼系数，默认为0.9999f
     * @return 模拟所用的时间
     */
    double step(float timestep, int iterations = 20, bool floor = false,
                float damping = 0.9999f);

    /// @brief 清空场景数据
    void clear();

    /// @brief 重置场景到初始状态
    void reset();

    /// @brief 初始化场景，子类可以重写此函数
    virtual void initialize(){};

    /// @brief 对于动力学物体，进行一步模拟，子类可以重写此函数
    virtual void kinematicStep(float time){};

    /// @brief 获取一个新的分组ID
    /// @return 新的分组ID
    Id getNewGroup();

    /**
     * @brief 设置上方向
     * @param dir 指定的上方向
     */
    void setUpDirection(const Vec3 &dir);

    SimulationState state;  ///< 当前的模拟状态

    std::vector<Vec2i> pills;  ///< 所有的pill数据
    std::vector<ParticleInfo> pInfo;  ///< 所有粒子的信息

    Vec3 upDirection = Vec3::UnitY();  ///< 上方向
    float gravity_strength = 1.0;  ///< 重力强度
    CudaSolver solver;  ///< 用于物理模拟的求解器

    ConstraintsCPU constraints;  ///< 所有的约束
};

} // namespace viper