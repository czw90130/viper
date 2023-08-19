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
#include <thrust/device_vector.h>
#include <thrust/execution_policy.h>
#include <thrust/host_vector.h>

namespace viper {

/**
 * @brief 获取device_vector的原始指针，指向某一特定位置。
 * @tparam T device_vector的元素类型。
 * @param v 输入的device_vector。
 * @param i 偏移量，默认为0。
 * @return 返回原始指针，指向v的第i个元素。
 */
template <class T> T *ptr(thrust::device_vector<T> &v, int i = 0) {
    return thrust::raw_pointer_cast(v.data()) + i;
}

/**
 * @brief 获取const device_vector的原始指针，指向某一特定位置。
 * @tparam T device_vector的元素类型。
 * @param v 输入的const device_vector。
 * @param i 偏移量，默认为0。
 * @return 返回原始指针，指向v的第i个元素。
 */
template <class T> const T *ptr(const thrust::device_vector<T> &v, int i = 0) {
    return thrust::raw_pointer_cast(v.data()) + i;
}

/**
 * @brief 打印device_vector的内容。
 * @tparam T device_vector的元素类型。
 * @param gpu_v 输入的device_vector。
 */
template <class T>
__host__ inline void printVector(const thrust::device_vector<T> &gpu_v) {
    thrust::host_vector<T> v = gpu_v;
    for (auto item : v)
        std::cout << " | " << item;
    std::cout << " |" << std::endl;
}

/**
 * @brief 打印device_vector<Vec3>的内容。
 * @param gpu_v 输入的device_vector<Vec3>。
 */
template <>
inline __host__ void
printVector<Vec3>(const thrust::device_vector<Vec3> &gpu_v) {
    thrust::host_vector<Vec3> v = gpu_v;
    for (auto item : v)
        std::cout << " | " << item.transpose();
    std::cout << " |" << std::endl;
}

/**
 * @brief 打印device_vector<Vec6>的内容。
 * @param gpu_v 输入的device_vector<Vec6>。
 */
template <>
inline __host__ void
printVector<Vec6>(const thrust::device_vector<Vec6> &gpu_v) {
    thrust::host_vector<Vec6> v = gpu_v;
    for (auto item : v)
        std::cout << " | " << item.transpose() << std::endl;
    std::cout << " |" << std::endl;
}

/**
 * @brief 开始计时。
 */
void tic();

/**
 * @brief 停止计时，并返回所消耗的时间（秒）。
 * @return 消耗的时间，单位为秒。
 */
double toc();

} // namespace viper