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
#include "Scene.h"

namespace viper {

/// 工具类集合，提供了一系列实用功能。
namespace Utils {
/**
 * @brief 根据给定的向量a生成一个正交帧（基）。
 * 
 * @param a 一个向量，通常表示法向量。
 * @return 生成的正交帧，由3x3矩阵表示。
 */
Transform getOrthogonalFrame(const Vec3 &a);

/**
 * @brief 生成[min, max]范围内的随机浮点数。
 * 
 * @param min 最小值。
 * @param max 最大值。
 * @return 在[min, max]范围内的随机浮点数。
 */
float randomFloat(float min = 0.0f, float max = 1.0f);

/**
 * @brief 在两个3D向量范围内生成一个随机向量。
 * 
 * @param min 表示范围的最小3D点。
 * @param max 表示范围的最大3D点。
 * @return 在给定范围内的随机3D向量。
 */
Vec3 randomVector(const Vec3 &min, const Vec3 &max);

/**
 * @brief 获取随机四元数
 * @return 随机四元数
 */
Quaternion randomQuaternion();

/**
 * @brief 生成一个随机方向向量
 * @return 随机方向向量
 */
Vec3 randomDirection();

/**
 * @brief 生成一个围绕法线n的随机方向圆上的向量
 * @param n 法线
 * @return 圆上的随机方向向量
 */
Vec3 randomDirectionCircle(const Vec3 &n);

/**
 * @brief 生成半球上的一个随机方向向量，其中n为半球的方向
 * @param n 半球的方向
 * @return 半球上的随机方向向量
 */
Vec3 randomDirectionHalfSphere(const Vec3 &n);

/**
 * @brief 生成一个整数数组，范围从start到end
 * @param start 开始值
 * @param end 结束值
 * @return 整数数组
 */
IntArray range(int start, int end);

/**
 * @brief 将数值n限制在[min, max]范围内。
 * 
 * @param n 需要限制的数值。
 * @param min 最小值。
 * @param max 最大值。
 * @return 在[min, max]范围内的数值。
 */
float clamp(float n, float min, float max);

/**
 * @brief 计算点到线段最近的点。
 * 
 * @param c 目标点。
 * @param a 线段的起点。
 * @param b 线段的终点。
 * @param t 输出参数，表示线段上最近点的参数化坐标。
 * @param d 输出参数，表示线段上的最近点。
 */
void closestPtPointSegment(Vec3 c, Vec3 a, Vec3 b, float &t, Vec3 &d);

/**
 * @brief 计算两线段之间的最近的两点。
 * 
 * 该函数使用线性代数和向量运算来计算两个线段之间的最近的两点。
 * 
 * 假设有两线段：线段1由点p1到q1定义，线段2由点p2到q2定义。
 * 让d1 = q1 - p1和d2 = q2 - p2，这是两线段的方向向量。
 * 通过设置线段1上的任何点为p1 + s * d1，线段2上的任何点为p2 + t * d2，
 * 其中s和t是从0到1的标量，我们可以寻找这样的s和t，使得这两点之间的距离最小。
 * 
 * @param p1 第一条线段的起点。
 * @param q1 第一条线段的终点。
 * @param p2 第二条线段的起点。
 * @param q2 第二条线段的终点。
 * @param s 输出参数，表示第一条线段上最近点的参数化坐标。
 * @param t 输出参数，表示第二条线段上最近点的参数化坐标。
 * @param c1 输出参数，表示第一条线段上的最近点。
 * @param c2 输出参数，表示第二条线段上的最近点。
 * @return 两个最近点之间的距离。
 */
float closestPtSegmentSegment(Vec p1, Vec q1, Vec p2, Vec q2, float &s,
                              float &t, Vec &c1, Vec &c2);

/**
 * @brief 计算一个点到胶囊体的最近距离。
 * 
 * 胶囊体由两个端点a和b、以及这两个端点处的半径ra和rb定义。
 * 这个函数计算点p到胶囊体的最近点c，以及该点处的半径r，并返回两点之间的距离。
 * 
 * @param p 输入点
 * @param a 胶囊体的一个端点
 * @param b 胶囊体的另一个端点
 * @param ra 端点a处的半径
 * @param rb 端点b处的半径
 * @param t 输出参数，表示最近点在线段a-b上的参数化位置。
 * @param c 输出参数，胶囊体上的最近点
 * @param r 输出参数，最近点处的半径
 * @return 点p到胶囊体的最近距离
 */
float closestPtPointPill(const Vec3 &p, const Vec3 &a, const Vec3 &b, float ra,
                         float rb, float &t, Vec &c, float &r);
/**
 * @brief 寻找两胶囊之间的最近点
 * @param x 点的集合
 * @param r 半径的集合
 * @param a 第一个胶囊的索引
 * @param b 第二个胶囊的索引
 * @param uv 输出参数，插值向量
 * @param pa 输出参数，第一个胶囊的点
 * @param pb 输出参数，第二个胶囊的点
 * @return 返回两胶囊之间的最近距离
 */
float closestPtPills(const Vec3Array &x, const FloatArray &r, const Vec2i &a,
                     const Vec2i &b, Vec2 &uv, Vec &pa, Vec &pb);

/**
 * @brief 数值方法寻找两胶囊之间的最近点
 * @details 通过数值方法计算两胶囊之间的最近点，可能比其他方法更准确，但可能也更慢。
 * @param x 点的集合
 * @param r 半径的集合
 * @param a 第一个胶囊的索引
 * @param b 第二个胶囊的索引
 * @param uv 输出参数，插值向量
 * @param pa 输出参数，第一个胶囊的点
 * @param pb 输出参数，第二个胶囊的点
 * @return 返回两胶囊之间的最近距离
 */
float closestPtPillsNumerical(const Vec3Array &x, const FloatArray &r,
                              const Vec2i &a, const Vec2i &b, Vec2 &uv, Vec &pa,
                              Vec &pb);

/**
 * @brief 根据给定的场景和偏移量生成网格
 * 
 * 此函数使用CGAL库来生成一个场景的网格。首先，它创建了一个带符号距离函数（SDF），然后使用这个SDF和CGAL的
 * make_surface_mesh来生成一个表面网格。网格的每个顶点位置和每个三角形的顶点索引都会被存储。
 * 
 * @param scene 指向场景的指针。场景包含物体的位置和大小等信息。
 * @param mesh 用于输出生成的网格数据。此参数将被填充顶点和三角形数据。
 * @param offset 用于SDF的偏移量。此偏移量会被应用到计算的距离上。
 */
void generateMesh(const Scene *scene, Mesh &mesh, float offset);

/**
 * @brief 使用数值方法计算胶囊体的体积。
 * 
 * @param a 胶囊体的一个端点。
 * @param b 胶囊体的另一个端点。
 * @param rA a点的半径。
 * @param rB b点的半径。
 * @param n 网格大小。
 * @return 胶囊体的体积。
 */
float pillVolumeNumerical(const Vec3 &a, const Vec3 &b, float rA, float rB,
                          int n);
/**
 * @brief 使用解析方法计算胶囊体的体积。
 * 
 * @param a 胶囊体的一个端点。
 * @param b 胶囊体的另一个端点。
 * @param rA a点的半径。
 * @param rB b点的半径。
 * @return 胶囊体的体积。
 */
float pillVolumeAnalytical(const Vec3 &a, const Vec3 &b, float rA, float rB);

/**
 * @brief 安全地计算两点之间的法线
 * @param a 第一个点
 * @param b 第二个点
 * @return 计算的法线
 */
Vec3 safeNormal(const Vec3 &a, const Vec3 &b);
Vec3 safeNormalCapsules(const Vec3 &a, const Vec3 &b, const Vec3 &a1,
                        const Vec3 &a2, const Vec3 &b1, const Vec3 &b2);

/**
 * @brief 插值两个点之间的位置
 * @param a 第一个点
 * @param b 第二个点
 * @param t 插值参数
 * @return 插值后的点
 */
Vec3 pointInterp(const Vec3 &a, const Vec3 &b, float t);

/**
 * @brief 计算给定点的重心坐标
 * @param a 三角形的第一个顶点
 * @param b 三角形的第二个顶点
 * @param c 三角形的第三个顶点
 * @param p 要计算重心坐标的点
 * @return 重心坐标
 */
Vec3 barycentricCoordinates(const Vec3 &a, const Vec3 &b, const Vec3 &c,
                            const Vec3 &p);
float pillVolume(const Vec3 &xa, const Vec3 &xb, float ra, float rb,
                 bool capA = false, bool capB = false);
} // namespace Utils

} // namespace viper