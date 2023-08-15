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

namespace viper {

/**
 * @brief 仿真状态类，用于存储与仿真相关的所有状态数据。
 * 
 * 这个类主要用于保存当前、上一帧和初始状态的位置、方向和半径等信息。
 */    

class SimulationState {
  public:
    /**
     * @brief 交换当前状态和上一帧的状态。
     */
    void swap() {
        x.swap(xp); ///< 交换当前和上一帧的位置。
        q.swap(qp); ///< 交换当前和上一帧的方向。
        r.swap(rp); ///< 交换当前和上一帧的半径。
    }
    
    /**
     * @brief 清除所有状态数据。
     */
    void clear() {
        x.clear();
        q.clear();
        r.clear();

        xp.clear();
        qp.clear();
        rp.clear();

        xi.clear();
        qi.clear();
        ri.clear();

        w.clear();
        wr.clear();

        v.clear();
        vq.clear();
        vr.clear();
    }

    // Current state
    Vec3Array x;       ///< 当前位置 (Current position)
    QuaternionArray q; ///< 当前方向 (Current orientation)
    FloatArray r;      ///< 当前半径 (Current radii)

    // Previous state
    Vec3Array xp;       ///< 上一帧的位置 (Previous position)
    QuaternionArray qp; ///< 上一帧的方向 (Previous orientation)
    FloatArray rp;      ///< 上一帧的半径 (Previous radii)

    // Initial state
    Vec3Array xi;       ///< 初始位置 (Initial position)
    QuaternionArray qi; ///< 初始方向 (Initial orientation)
    FloatArray ri;      ///< 初始半径 (Initial radii)

    std::vector<uint8_t> xa;
    std::vector<uint8_t> xai;
    std::vector<uint8_t> qa;
    std::vector<uint8_t> qai;

    std::vector<Matrix4> b;
    std::vector<Matrix4> bp;
    std::vector<Matrix4> bi;

    // Weights
    FloatArray w;  ///< 位置权重（质量的倒数）(Position weight - inverse mass)
    FloatArray wq; ///< 方向权重 (Orientation weight)
    FloatArray wr; ///< 半径权重 (Radius weight)

    // Velocities
    Vec3Array v;   ///< 位置速度 (Positional velocity)
    Vec3Array vq;  ///< 角速度 (Angular velocity)
    FloatArray vr; ///< 半径速度 (Radii velocity)
};

} // namespace viper