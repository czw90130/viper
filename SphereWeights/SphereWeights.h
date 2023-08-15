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

#include <OpenGP/SphereMesh/SphereMesh.h>
#include <OpenGP/SurfaceMesh/SurfaceMesh.h>

/**
 * @brief 计算球形网格上的权重并应用到表面网格上。
 * 
 * 此函数取球形网格作为输入，并计算其权重，
 * 然后将这些权重应用到给定的表面网格上。
 *
 * @param smesh 输入的球形网格。
 * @param mesh 输出的表面网格，用于存储权重。
 */
void calc_weights(const OpenGP::SphereMesh &smesh, OpenGP::SurfaceMesh &mesh);