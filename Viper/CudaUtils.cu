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

#include "CudaUtils.cuh"
#include <chrono>

namespace viper {

/// 使用chrono库来测量时间的起始点
decltype(std::chrono::system_clock::now()) start_time;

/**
 * @brief 开始计时。此函数首先同步CUDA设备，然后记录当前时间为开始时间。
 */
void tic() {
    cudaDeviceSynchronize(); ///< 确保CUDA设备完成所有操作
    start_time = std::chrono::system_clock::now(); ///< 记录当前时间为开始时间
}

/**
 * @brief 停止计时，并返回从调用tic()函数到现在所消耗的时间（毫秒）。
 * 
 * @return 从调用tic()函数到现在所消耗的时间，单位为毫秒。
 */
double toc() {
    cudaDeviceSynchronize(); ///< 确保CUDA设备完成所有操作
    auto end = std::chrono::system_clock::now(); ///< 记录当前时间为结束时间

    // 计算开始时间到结束时间的持续时间
    auto dur = end - start_time;
    
    // 将持续时间转换为微秒
    auto us =
        std::chrono::duration_cast<std::chrono::microseconds>(dur).count();

    return (double)us / 1000.0; ///< 返回毫秒
}

} // namespace viper