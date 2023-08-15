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

#include "Scene.h"
#include "Utils.h"
#include "nnsearch.h"
#include <algorithm>
#include <iostream>
#include <numeric>
#include <thrust/iterator/transform_iterator.h>

int current_frame = 0;  ///< 当前帧数

namespace viper {

/**
 * @brief 执行模拟的一步操作。
 * @param dt 时间步长。
 * @param iterations 迭代次数。
 * @param floor 是否存在地面。
 * @param damping 阻尼系数。
 * @return 返回模拟所需的时间。
 */
double Scene::step(float dt, int iterations, bool floor, float damping) {
    int M = pills.size(); // 获取pill的数量
    std::vector<int> group(M); // 初始化分组向量
    for (int i = 0; i < M; i++)
        group[i] = pInfo[pills[i][0]].group; // 为每个pill分配分组

    current_frame++; // 更新当前帧数
    // 调用求解器进行物理模拟，并返回模拟时间
    double t = solver.solve(constraints, state, pills, group, dt,
                            -10.f * gravity_strength * upDirection, iterations,
                            floor, damping);

    return t;
}

/**
 * @brief 场景的构造函数，进行初始化操作。
 */
Scene::Scene() { clear(); }

/**
 * @brief 重置场景到初始状态。
 */
void Scene::reset() {
    for (int i = 0; i < state.x.size(); i++) {
        state.x[i] = state.xi[i];
        state.xp[i] = state.xi[i];
    }

    for (int i = 0; i < state.r.size(); i++) {
        state.r[i] = state.ri[i];
    }

    for (int i = 0; i < state.q.size(); i++) {
        state.q[i] = state.qi[i];
        state.qp[i] = state.qi[i];
    }

    for (int i = 0; i < constraints.shape.size(); i++)
        constraints.shape[i].q = Quaternion::Identity(); // 为约束设置恒等四元数

    current_frame = 0; // 帧数重置为0
}

/**
 * @brief 向场景中添加一个新的粒子。
 * 
 * 这个方法会将新的粒子状态信息添加到模拟状态中，并返回新粒子的ID。
 *
 * @param p 粒子的位置。
 * @param r 粒子的半径。
 * @param w 粒子的重量（其倒数可视为质量）。
 * @param kinematic 定义粒子是否为运动的。True表示粒子是静态的，反之为动态的。
 * @return 返回新添加的粒子的ID。
 */
int Scene::addParticle(const Vec3 &p, float r, float w, bool kinematic) {
    // 更新当前状态以存储新粒子的位置、半径和速度信息
    state.x.push_back(p);      ///< 存储粒子当前位置
    state.xp.push_back(p);     ///< 存储粒子上一帧的位置
    state.v.push_back(Vec3::Zero()); ///< 初始化粒子的速度为0
    state.w.push_back(w);      ///< 设置粒子的位置权重
    state.wr.push_back(1.0);   ///< 设置粒子的半径权重为1.0
    state.r.push_back(r);      ///< 存储粒子的半径
    state.rp.push_back(r);     ///< 存储粒子上一帧的半径

    // 存储粒子的初始状态，可用于某些仿真重置操作
    state.xi.push_back(p);     ///< 存储粒子的初始位置
    state.ri.push_back(r);     ///< 存储粒子的初始半径

    // 根据是否为kinematic（动态）来更新状态
    state.xa.push_back(!kinematic); ///< 根据kinematic参数设置粒子的动态状态
    state.xai.push_back(!kinematic);///< 存储粒子动态状态的初始值

    ParticleInfo info; ///< 创建一个新的粒子信息对象
    pInfo.push_back(info); ///< 添加新的粒子信息到pInfo向量中

    return state.x.size() - 1; ///< 返回新添加的粒子的ID
}

/**
 * @brief 在两个指定的粒子之间添加一个pill。
 * 
 * 这个方法将在两个指定的粒子之间创建一个pill，更新相关的状态信息，并返回这个新pill的ID。
 *
 * @param a 第一个粒子的ID。
 * @param b 第二个粒子的ID。
 * @param kinematic 定义pill是否为运动的。True表示pill是静态的，反之为动态的。
 * @return 返回新添加的pill的ID。
 */
Id Scene::addPill(Id a, Id b, bool kinematic) {
    pills.push_back(Vec2i(a, b));  ///< 在pills向量中添加新的条目，代表两个粒子a和b之间的连接
    Id id = pills.size() - 1;      ///< 获取新添加pill的索引，作为它的ID
    
    // 标记这两个粒子为fiber类型
    pInfo[a].isFiber = true;
    pInfo[b].isFiber = true;

    // 计算从粒子a到粒子b的方向向量，并将其标准化
    Vec3 ab = (state.x[b] - state.x[a]).normalized();
    // 计算出的方向向量与Z轴的向量间的旋转四元数
    Quaternion q = Quaternion::FromTwoVectors(Vec3::UnitZ(), ab);

    // 更新pill的状态信息
    state.q.push_back(q);           ///< 存储pill的当前旋转四元数
    state.qp.push_back(q);          ///< 存储pill上一帧的旋转四元数
    state.qi.push_back(q);          ///< 存储pill的初始旋转四元数
    state.wq.push_back(1.0f);       ///< 设置pill的旋转权重为1.0
    state.vq.push_back(Vec3::Zero()); ///< 初始化pill的角速度为0

    // 根据是否为kinematic（动态）来更新状态
    state.qa.push_back(!kinematic); ///< 设置pill的动态状态
    state.qai.push_back(!kinematic);///< 存储pill动态状态的初始值

    return id; ///< 返回新添加的pill的ID
}

/**
 * @brief 场景的析构函数，进行清理操作。
 */
Scene::~Scene() { clear(); }

/**
 * @brief 清除场景的所有内容。
 */
void Scene::clear() {
    // 清除所有状态和约束信息
    state.clear();
    pInfo.clear();
    pills.clear();

    constraints.distancemax.clear();
    constraints.distance.clear();
    constraints.skinning.clear();
    constraints.volume.clear();
    constraints.volume2.clear();
    constraints.bend.clear();
    constraints.stretch.clear();
    constraints.radius.clear();
    constraints.bilap.clear();
    constraints.shape.clear();
    constraints.touch.clear();
    constraints.shape2.clear();
}

/**
 * @brief 获取新的粒子组ID。
 * @return 返回新的粒子组ID。
 */
Id Scene::getNewGroup() {
    int maxGroup = -1;
    for (ParticleInfo &i : pInfo) {
        if (i.group > maxGroup)
            maxGroup = i.group; // 寻找当前的最大组ID
    }
    return maxGroup + 1; // 返回新的组ID
}

/**
 * @brief 在场景中添加一个杆。
 * @param n 杆中的粒子数量。
 * @param start 起始位置。
 * @param step 步长。
 * @param r 半径。
 * @param volume 是否考虑体积。
 * @param pIds 粒子的ID数组。
 * @param rIds pill的ID数组。
 * @param stretch_eta 拉伸系数。
 * @param volume_eta 体积系数。
 * @param bilap_eta 双曲线系数。
 */
void Scene::addRod(int n, Vec3 start, Vec3 step, float r, bool volume,
                   IntArray &pIds, IntArray &rIds, float stretch_eta,
                   float volume_eta, float bilap_eta) {
    
    // 清除传入的粒子和pill ID数组
    pIds.clear();
    rIds.clear();
    
    // 在指定的起始位置，以给定的步长，添加n个粒子到杆中
    for (int i = 0; i < n; i++) {
        Vec3 p = start + i * step; // 计算粒子的位置
        Id id = addParticle(p, r); // 添加粒子并获取其ID
        pIds.push_back(id); // 将粒子ID保存到数组
    }

    // 在连续的粒子之间添加pill并存储其ID
    for (int i = 1; i < n; i++) {
        Id a = pIds[i - 1];
        Id b = pIds[i];
        Id id = addPill(a, b); // 在连续的粒子之间添加pill并存储其ID
        rIds.push_back(id);
        float d = step.norm();  // 计算步长的长度
        // 添加拉伸约束
        constraints.stretch.push_back(C_stretch(a, b, id, d, stretch_eta));
        // 如果考虑体积，则添加体积约束
        if (volume) {
            constraints.volume2.push_back(
                C_volume2(a, b, id, state, volume_eta));
        }
    }

    // 为连续的pills添加弯曲约束
    for (int i = 1; i < n - 1; i++) {
        constraints.bend.push_back(C_bend(rIds[i - 1], rIds[i], state, 0.0f));
    }

    // 为杆中的粒子添加双曲线约束
    for (int i = 1; i < n - 1; i++) {
        constraints.bilap.push_back(C_bilap(pIds, i, bilap_eta));
    }
}

/**
 * @brief 设置场景中的上方向。
 * @param dir 上方向。
 */
void Scene::setUpDirection(const Vec3 &dir) { upDirection = dir; }

} // namespace viper