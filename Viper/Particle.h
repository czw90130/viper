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
 * @struct ParticleInfo
 * @brief 描述粒子的详细信息
 * 
 * 该结构体为粒子提供了详细的描述，例如其所属的组、其是否是纤维等。
 */
struct ParticleInfo {
    Id group = -1;     /**< 粒子所属的组ID，默认为-1 */
    bool isFiber = false; /**< 标志表示此粒子是否为纤维，默认为false */
    bool isFake = false;  /**< 标志表示此粒子是否为伪造的，默认为false */
    float alpha;         /**< 粒子的alpha值，通常用于渲染或透明度等用途 */
    bool isBone = false;  /**< 标志表示此粒子是否为骨骼，默认为false */
};

} // namespace viper