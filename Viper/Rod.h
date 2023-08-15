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

/**
 * @file rod.h
 * @brief 定义Rod相关的数据结构
 */

#pragma once

#include "Common.h"

namespace viper {

/**
 * @struct RodBundle
 * @brief 描述一束杆的数据结构
 * 
 * 该结构体包含了一组相关联的杆及其相关信息，例如与之关联的粒子和pill。
 */
struct RodBundle {
    /**
     * @brief 默认构造函数
     */
    RodBundle() {}

    /**
 * @brief 基于提供的粒子和pill切片ID集合构造一个RodBundle。
 * 
 * 此构造函数初始化RodBundle的粒子和pill切片ID集合，并将所有的pill IDs聚集到一个单独的数组中。
 *
 * @param pIds 与束相关的粒子ID集合。每个条目是一个数组，代表与特定杆相关的粒子ID。
 * @param rIds 与束相关的pill ID的切片集合。每个条目是一个数组，代表与特定杆相关的pill ID。
 */
    RodBundle(const std::vector<IntArray> &pIds,
              const std::vector<IntArray> &rIds)
        : particles(pIds), pillsSlices(rIds) {
        // 遍历提供的pill ID切片集合，并将所有pill IDs聚集到pills数组中
        for (const IntArray &p : rIds) {
            for (int idx : p)
                pills.push_back(idx);
        }
    }

    std::vector<IntArray> particles;          /**< 与束相关的粒子ID集合 */
    std::vector<IntArray> pillsSlices;        /**< 与束相关的pill ID的切片集合 */
    IntArray pills;                           /**< 与束相关的所有pill ID */
    std::vector<std::vector<float>> skin_weights; /**< 杆上的皮肤权重集合 */
    std::vector<std::vector<int>> skin_bone_ids;  /**< 杆上的皮肤骨骼ID集合 */
};

} // namespace viper