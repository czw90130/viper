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

#include "Utils.h"
#include <CGAL/Complex_2_in_triangulation_3.h>
#include <CGAL/Implicit_surface_3.h>
#include <CGAL/Surface_mesh_default_triangulation_3.h>
#include <CGAL/make_surface_mesh.h>
#include <iostream>
#include <map>
#include <random>

namespace viper {

// 定义一些CGAL相关的类型别名
typedef CGAL::Surface_mesh_default_triangulation_3 Tr;
typedef CGAL::Complex_2_in_triangulation_3<Tr> C2t3;
typedef Tr::Geom_traits GT;
typedef GT::Sphere_3 Sphere_3;
typedef GT::Point_3 Point_3;
typedef GT::FT FT;

template <typename function>
using Surface_3 = CGAL::Implicit_surface_3<GT, function>;

/**
 * @brief 表示场景的带符号距离函数 (SceneSDF) 结构体
 * @details 计算一个点到场景中所有物体的最近距离
 */
struct SceneSDF {
    SceneSDF(const Scene *scene, float offset) : scene(scene), offset(offset) {}
    /**
     * @brief 重载的函数调用操作符，计算点到场景的最近距离
     * @param p3 要测试的点
     * @return 到场景的最近距离
     */
    FT operator()(Point_3 p3) const {
        Vec3 p = Vec3(p3.x(), p3.y(), p3.z());
        float minDist = 1e10f;
        for (int i = 0; i < scene->state.x.size(); i++) {
            float d = (p - scene->state.x[i]).norm() - scene->state.r[i];
            minDist = std::min(minDist, d);
        }
        for (const Vec2i &f : scene->pills) {

            Vec3 c;
            float r;
            float t;
            float d = Utils::closestPtPointPill(
                p, scene->state.x[f(0)], scene->state.x[f(1)],
                scene->state.r[f(0)], scene->state.r[f(1)], t, c, r);
            minDist = std::min(minDist, d - r);
        }

        return minDist - offset;
    }

    float offset;      ///< 场景的偏移量
    const Scene *scene;///< 指向场景的指针
};

namespace Utils {
/**
 * @brief 生成一个随机四元数
 * @return 随机生成的四元数
 */
Quaternion randomQuaternion() {
    return Quaternion(
               Rotation(randomFloat(0.0f, 2.0 * M_PIf), randomDirection()))
        .normalized();
}

/**
 * @brief 安全地计算两点之间的法线
 * @param a 第一个点
 * @param b 第二个点
 * @return 计算的法线
 */
Vec3 safeNormal(const Vec3 &a, const Vec3 &b) {
    Vec3 n = b - a;
    float d = n.norm();
    const float eps = 1e-8f;
    if (d < eps)
        n = Utils::randomDirection();
    else
        n = n / d;

    return n;
}

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
void generateMesh(const Scene *scene, Mesh &mesh, float offset) {
    // 清空之前的网格数据
    mesh.vertices.clear();
    mesh.triangles.clear();

    // 定义三维Delaunay三角化和其相关的复杂2-in-3结构
    Tr tr3d;
    C2t3 c2t3(tr3d);

    // 初始化带符号距离函数
    SceneSDF sdf = SceneSDF(scene, offset);

    // 定义隐式表面并设置其边界球
    Surface_3<SceneSDF> surface(sdf, Sphere_3(Point_3(0.0, 1.0, 0.0), 4.));
    
    // 设置表面网格生成的标准
    CGAL::Surface_mesh_default_criteria_3<Tr> criteria(20.f, 0.03f, 0.03f);
    
    // 使用CGAL的make_surface_mesh生成表面网格
    CGAL::make_surface_mesh(c2t3, surface, criteria, CGAL::Manifold_tag());

    // 从c2t3结构中获取三角化
    Tr tr = c2t3.triangulation();

    // 使用map存储三角化中的顶点和其对应的索引
    std::map<Tr::Vertex_handle, int> V;
    int i = 0;
    for (auto it = tr.finite_vertices_begin(); it != tr.finite_vertices_end();
         it++) {
        Tr::Vertex_handle vh = it;
        V[vh] = i++;
        Point_3 p = it->point();
        mesh.vertices.push_back(Vec3(p.x(), p.y(), p.z()));
    }

    // 遍历三角化中的面，并将其存储到网格的三角形数组中
    for (auto it = tr.finite_facets_begin(); it != tr.finite_facets_end();
         it++) {
        Tr::Cell_handle cell = it->first;
        int index = it->second;
        if (!c2t3.is_in_complex(cell, index))
            continue;
        Vec3i t = Vec3i(-1, -1, -1);
        for (int i = 0; i < 3; i++) {
            Tr::Vertex_handle vh =
                cell->vertex(tr.vertex_triple_index(index, i));
            auto vIt = V.find(vh);
            if (vIt == V.end())
                std::cout << "WOOOOOOOOOOO" << std::endl;
            else
                t[i] = vIt->second;
        }
        mesh.triangles.push_back(t);
    }

    // 根据SDF对三角形的朝向进行校正，以确保其与SDF的正负值一致
    for (int i = 0; i < mesh.triangles.size(); i++) {
        Vec3i &T = mesh.triangles[i];
        Vec3 ab = mesh.vertices[T[1]] - mesh.vertices[T[0]];
        Vec3 ac = mesh.vertices[T[2]] - mesh.vertices[T[0]];
        Vec3 n = ab.cross(ac).normalized();
        Vec3 p0 =
            (mesh.vertices[T[0]] + mesh.vertices[T[0]] + mesh.vertices[T[0]]) /
            3.0f;
        Vec3 p1 = p0 + 0.00001f * n;
        float f0 = sdf(Point_3(p0[0], p0[1], p0[2]));
        float f1 = sdf(Point_3(p1[0], p1[1], p1[2]));
        if (f0 > f1) {
            int tmp = T[1];
            T[1] = T[2];
            T[2] = tmp;
        }
    }

    // 输出生成的顶点和面的数量
    std::cout << "nverts: " << mesh.vertices.size() << std::endl;
    std::cout << "nFaces: " << mesh.triangles.size() << std::endl;
}

/**
 * @brief 根据给定的向量a生成一个正交帧（基）。
 * 
 * @param a 一个向量，通常表示法向量。
 * @return 生成的正交帧，由3x3矩阵表示。
 */
Transform getOrthogonalFrame(const Vec3 &a) {
    Vec3 helper = Vec3(1.f, 0.f, 0.f);
    if (abs(a.dot(helper)) > 0.9f)
        helper = Vec3(0.f, 1.f, 0.f);
    Vec3 u = a.cross(helper).normalized();
    Vec3 v = a.cross(u);

    Matrix3 M;
    M.col(0) = a;
    M.col(1) = u;
    M.col(2) = v;

    return Transform(M);
}

/**
 * @brief 计算点到线段最近的点。
 * 
 * @param c 目标点。
 * @param a 线段的起点。
 * @param b 线段的终点。
 * @param t 输出参数，表示线段上最近点的参数化坐标。
 * @param d 输出参数，表示线段上的最近点。
 */
void closestPtPointSegment(Vec3 c, Vec3 a, Vec3 b, float &t, Vec3 &d) {
    Vec3 ab = b - a;
    t = (c - a).dot(ab);
    if (t < 0.f) {
        t = 0.f;
        d = a;
    } else {
        float denom = ab.dot(ab);
        if (t >= denom) {
            t = 1.0f;
            d = b;
        } else {
            t = t / denom;
            d = a + t * ab;
        }
    }
}

/**
 * @brief 生成[min, max]范围内的随机浮点数。
 * 
 * @param min 最小值。
 * @param max 最大值。
 * @return 在[min, max]范围内的随机浮点数。
 */
float randomFloat(float min, float max) {
    std::random_device rd;
    std::mt19937 e2(rd());
    std::uniform_real_distribution<float> dist(min, max);
    float r = dist(e2);
    return r;
}

/**
 * @brief 在两个3D向量范围内生成一个随机向量。
 * 
 * @param min 表示范围的最小3D点。
 * @param max 表示范围的最大3D点。
 * @return 在给定范围内的随机3D向量。
 */
Vec3 randomVector(const Vec3 &min, const Vec3 &max) {
    float x = randomFloat(min.x(), max.x());
    float y = randomFloat(min.y(), max.y());
    float z = randomFloat(min.z(), max.z());
    return Vec3(x, y, z);
}

/**
 * @brief 将数值n限制在[min, max]范围内。
 * 
 * @param n 需要限制的数值。
 * @param min 最小值。
 * @param max 最大值。
 * @return 在[min, max]范围内的数值。
 */
float clamp(float n, float min, float max) {
    if (n < min)
        return min;
    if (n > max)
        return max;
    return n;
}

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
                              float &t, Vec &c1, Vec &c2) {
    const float eps = 1e-10f;  // 一个非常小的数，用于避免除以0的错误和处理浮点数不精确的问题
    Vec d1 = q1 - p1;  // 第一条线段的方向
    Vec d2 = q2 - p2;  // 第二条线段的方向
    Vec r = p1 - p2;   // 从p2到p1的向量
    float a = d1.squaredNorm();  // d1的长度的平方
    float e = d2.squaredNorm();  // d2的长度的平方
    float f = d2.dot(r);  // d2与r的点积

    // 如果两线段都退化为点
    if (a <= eps && e <= eps) {
        s = t = 0.0f;
        c1 = p1;
        c2 = p2;
        return (c1 - c2).squaredNorm();
    }
    // 如果第一条线段退化为点
    if (a <= eps) {
        s = 0.0f;
        t = f / e;  // 使用点与线段的最近点方法
        t = clamp(t, 0.0f, 1.0f);  // 确保t在[0,1]范围内
    } else {
        float c = d1.dot(r);  // d1与r的点积
        // 如果第二条线段退化为点
        if (e <= eps) {
            t = 0.0f;
            s = clamp(-c / a, 0.0f, 1.0f);  // 使用点与线段的最近点方法
        } else {
            float b = d1.dot(d2);  // d1与d2的点积
            float denom = a * e - b * b;  // 确定性因子，判断两线段是否平行

            // 如果线段不平行
            if (denom != 0.0f)
                s = clamp((b * f - c * e) / denom, 0.0f, 1.0f);
            else
                s = 0.5f;  // 如果线段平行，随意选择s的值，因为任何s值都会产生相同的结果

            t = (b * s + f) / e;

            // 下面的逻辑确保s和t都在[0,1]范围内，这样c1和c2都在给定的线段上
            if (t < 0.0f) {
                t = 0.0f;
                s = clamp(-c / a, 0.0f, 1.0f);
            } else if (t > 1.0f) {
                t = 1.0f;
                s = clamp((b - c) / a, 0.0f, 1.0f);
            }
        }
    }

    c1 = p1 + d1 * s;  // 计算第一条线段上的最近点
    c2 = p2 + d2 * t;  // 计算第二条线段上的最近点
    return (c1 - c2).norm();  // 返回两点之间的距离
}

/**
 * @brief 生成一个整数数组，表示从start到end的范围。
 * 
 * @param start 起始整数。
 * @param end 结束整数。
 * @return 从start到end的整数数组。
 */
IntArray range(int start, int end) {
    IntArray ids;
    ids.reserve(end - start + 1);
    for (int i = start; i <= end; i++)
        ids.push_back(i);
    return ids;
}

/**
 * @brief 生成一个随机方向的3D向量。
 * 
 * @return 一个随机方向的3D向量。
 */
Vec3 randomDirection() {
    // https://math.stackexchange.com/questions/44689/how-to-find-a-random-axis-or-unit-vector-in-3d
    float t = randomFloat(0.0f, 2.0f * M_PIf);
    float z = randomFloat(-1.0f, 1.0f);
    float a = sqrt(1.f - z * z);
    return Vec(a * cos(t), a * sin(t), z);
}

/**
 * @brief 根据法向量生成随机的圆形方向。
 * 
 * @param n 一个表示法向量的3D向量。
 * @return 在平面上的随机方向。
 */
Vec3 randomDirectionCircle(const Vec3 &n) {
    float t = randomFloat(0.f, 2 * M_PIf);
    Transform T = getOrthogonalFrame(n);
    return T * Vec3(0.f, cos(t), sin(t));
}

/**
 * @brief 生成沿给定法向量的半球方向的随机3D向量。
 * 
 * @param n 表示半球方向的法向量。
 * @return 沿给定法向量的半球方向的随机3D向量。
 */
Vec3 randomDirectionHalfSphere(const Vec3 &n) {
    Vec3 v = randomDirection();
    float p = v.dot(n);
    if (p < 0.f)
        v -= 2.f * p * n;
    return v;
}

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
                         float rb, float &t, Vec &c, float &r) {
    Vec3 ab = b - a;  // 计算胶囊体的轴向量
    float L = ab.norm();  // 获得胶囊体轴的长度
    Vec3 abn = ab / L;  // 获得单位方向向量
    Vec3 ap = p - a;  // 计算从a到p的向量
    // 计算p在胶囊体轴线上的投影
    Vec3 p_proj = ap - abn * ap.dot(abn);  
    
    // 计算端点半径差对应的角度的正切值
    float sigma = tan(asin((rb - ra) / L));
    
    // 如果sigma是NaN，说明胶囊体的端点半径是一样的，或者轴太短
    if (std::isnan(sigma)) {
        // 直接返回点p到胶囊体两端点的最小距离
        return std::min(ap.norm() - ra, (p - b).norm() - rb);
    }
    
    // 使用sigma调整p的位置，使其更接近胶囊体的曲面
    Vec3 offset = abn * p_proj.norm() * sigma;
    
    // 计算调整后的点p在胶囊体轴线上的投影的参数化位置
    t = std::min(1.f, std::max(0.f, abn.dot(ap + offset) / L));
    
    // 计算最近点c的位置
    c = (1.f - t) * a + t * b;
    
    // 计算最近点处的半径
    r = (1.f - t) * ra + t * rb;
    
    // 返回点p到胶囊体的最近距离
    return (c - p).norm() - r;
}

/**
 * @brief 计算两圆锥体的距离
 * @param a1 圆锥A的开始点
 * @param a2 圆锥A的结束点
 * @param b1 圆锥B的开始点
 * @param b2 圆锥B的结束点
 * @param ra1 圆锥A开始点的半径
 * @param ra2 圆锥A结束点的半径
 * @param rb1 圆锥B开始点的半径
 * @param rb2 圆锥B结束点的半径
 * @param uv 插值向量
 * @return 返回两圆锥体的距离
 */
float coneSphereDistance(const Vec3 &a1, const Vec3 &a2, const Vec3 &b1,
                         const Vec3 &b2, float ra1, float ra2, float rb1,
                         float rb2, Vec2 uv) {
    Vec3 a = a1 + uv(0) * (a2 - a1);
    Vec3 b = b1 + uv(1) * (b2 - b1);
    float ra = ra1 + uv(0) * (ra2 - ra1);
    float rb = rb1 + uv(1) * (rb2 - rb1);
    return (a - b).norm() - ra - rb;
}

/**
 * @brief 计算点与胶囊之间的距离U
 * @param x 点的集合
 * @param r 半径的集合
 * @param a 第一个胶囊的索引
 * @param b 第二个胶囊的索引
 * @param u 插值参数
 * @param v 输出参数，胶囊之间的距离
 * @param pa 输出参数，第一个胶囊的点
 * @param pb 输出参数，第二个胶囊的点
 * @return 返回点与胶囊之间的距离U
 */
float pillDistanceU(const Vec3Array &x, const FloatArray &r, const Vec2i &a,
                    const Vec2i &b, float u, float &v, Vec3 &pa, Vec3 &pb) {
    pa = (1.f - u) * x[a(0)] + u * x[a(1)];
    float ra = (1.f - u) * r[a(0)] + u * r[a(1)];
    float rc;
    return closestPtPointPill(pa, x[b(0)], x[b(1)], r[b(0)], r[b(1)], v, pb,
                              rc) -
           ra;
}

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
                     const Vec2i &b, Vec2 &uv, Vec &pa, Vec &pb) {
    const int iter = 14;
    Vec2 range = Vec2(0.f, 1.f);
    for (int j = 0; j < iter; j++) {
        float eps = M_PIf * 1e-7f;
        float mid = 0.5f * range.sum();
        float ua = mid - eps;
        float ub = mid + eps;
        if (ua > ub)
            break;
        float v;
        Vec3 pa, pb;
        float fa = pillDistanceU(x, r, a, b, ua, v, pa, pb);
        float fb = pillDistanceU(x, r, a, b, ub, v, pa, pb);
        if (fa == fb)
            range = Vec2(ua, ub);
        else if (fa > fb)
            range(0) = ua;
        else
            range(1) = ub;
    }
    uv(0) = 0.5f * range.sum();
    return pillDistanceU(x, r, a, b, uv(0), uv(1), pa, pb);
}

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
                              Vec &pb) {
    int n = 1000;
    float min_dist = 1e20f;
    for (int i = 0; i < n; i++) {
        float u = ((float)i) / (n - 1);
        Vec3 pa = (1.f - u) * x[a(0)] + u * x[a(1)];
        float ra = (1.f - u) * r[a(0)] + u * r[a(1)];
        for (int j = 0; j < n; j++) {
            float v = ((float)j) / (n - 1);
            Vec3 pb = (1.f - v) * x[b(0)] + v * x[b(1)];
            float rb = (1.f - v) * r[b(0)] + v * r[b(1)];

            float d = (pa - pb).norm() - ra - rb;
            if (d < min_dist) {
                min_dist = d;
                uv = Vec2(u, v);
            }
        }
    }

    pa = (1.f - uv[0]) * x[a(0)] + uv[0] * x[a(1)];
    pb = (1.f - uv[1]) * x[b(0)] + uv[1] * x[b(1)];
    return min_dist;
}

/**
 * @brief 计算点到胶囊体的最近距离（数值方法）。
 * 
 * @param p 要计算的点。
 * @param a 胶囊体的一个端点。
 * @param b 胶囊体的另一个端点。
 * @param rA a点的半径。
 * @param rB b点的半径。
 * @param n 数值方法的采样点数量。
 * @return 点到胶囊体的最近距离。
 */
float pillDistanceNumerical(const Vec3 &p, const Vec3 &a, const Vec3 &b,
                            float rA, float rB, int n) {
    float min_dist = 1e20f;
    for (int i = 0; i < n; i++) {
        float t = ((float)i) / (n - 1);
        Vec3 c = (1.f - t) * a + t * b;
        float rc = (1.f - t) * rA + t * rB;
        float d = (c - p).norm() - rc;
        min_dist = std::min(d, min_dist);
    }

    return min_dist;
}

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
                          int n) {
    Vec3 gMin = (a - Vec3(rA, rA, rA)).cwiseMin(b - Vec3(rB, rB, rB));
    Vec3 gMax = (a + Vec3(rA, rA, rA)).cwiseMax(b + Vec3(rB, rB, rB));
    Vec3 gDiff = gMax - gMin;
    float maxDiff = gDiff.maxCoeff();
    float s = maxDiff / (float)n;
    Vec3i gDims = (gDiff / s).array().ceil().cast<int>();

    Vec3 gSize = gDims.cast<float>() * s;
    float Vgrid = gSize.prod();
    int cell_count = gDims.prod();

    int count = 0;
    for (int i = 0; i < gDims[0]; i++) {
        for (int j = 0; j < gDims[1]; j++) {
            for (int k = 0; k < gDims[2]; k++) {
                Vec3 p = gMin + s * Vec3(i, j, k);
                float t, r;
                Vec3 c;
                // float d = closestPtPointPill(p, a, b, rA, rB, t, c, r);
                float d = pillDistanceNumerical(p, a, b, rA, rB, 100);
                // closestPtPointSegment(p, a, b, t, c);
                bool isInside = d < 0.0f;
                if (isInside)
                    count++;
            }
        }
    }
    float r = (float)count / (float)cell_count;
    return Vgrid * r;
}

/**
 * @brief 使用解析方法计算胶囊体的体积。
 * 
 * @param a 胶囊体的一个端点。
 * @param b 胶囊体的另一个端点。
 * @param rA a点的半径。
 * @param rB b点的半径。
 * @return 胶囊体的体积。
 */
float pillVolumeAnalytical(const Vec3 &a, const Vec3 &b, float rA, float rB) {
    float d = (a - b).norm();
    float beta = asin((rB - rA) / d);
    float sinBeta = sin(beta);
    float cosBeta = cos(beta);
    float L = d + (rA - rB) * sinBeta;
    float ha = rA * cosBeta;
    float hb = rB * cosBeta;
    float da = rA * (1.f - sinBeta);
    float db = rB * (1.f + sinBeta);

    float v_cyl = M_PIf / 3.f * L * (ha * ha + ha * hb + hb * hb);
    float v_capA = M_PIf / 3.f * da * da * (3.f * rA - da);
    float v_capB = M_PIf / 3.f * db * db * (3.f * rB - db);
    float v = v_cyl + v_capA + v_capB;
    return v;
}

/**
 * @brief 在两点之间插值来获得一个新的点。
 * 
 * @param a 第一个点。
 * @param b 第二个点。
 * @param t 插值参数（0 <= t <= 1）。
 * @return 插值得到的点。
 */
Vec3 pointInterp(const Vec3 &a, const Vec3 &b, float t) {
    return (1.f - t) * a + t * b;
}

/**
 * @brief 安全地计算两个胶囊体间的法向量。
 * 
 * @param a 第一个胶囊体的一个端点。
 * @param b 第一个胶囊体的另一个端点。
 * @param a1 第二个胶囊体的一个端点。
 * @param a2 第二个胶囊体的另一个端点。
 * @param b1 第三个胶囊体的一个端点。
 * @param b2 第三个胶囊体的另一个端点。
 * @return 胶囊体之间的法向量。
 */
Vec3 safeNormalCapsules(const Vec3 &a, const Vec3 &b, const Vec3 &a1,
                        const Vec3 &a2, const Vec3 &b1, const Vec3 &b2) {
    Vec3 n = b - a;
    float d = n.norm();
    const float eps = 1e-8f;
    if (d < eps)
        n = (a2 - a1).cross(b2 - b1).normalized();
    if (n.norm() < eps)
        n = Utils::randomDirection();
    else
        n = n / d;

    return n;
}

/**
 * @brief 计算2D三角形的面积。
 * 
 * @param x1, y1, x2, y2, x3, y3 三角形的三个顶点坐标。
 * @return 2D三角形的面积。
 */
float triArea2D(float x1, float y1, float x2, float y2, float x3, float y3) {
    return (x1 - x2) * (y2 - y3) - (x2 - x3) * (y1 - y2);
}

/**
 * @brief 计算3D点在三角形中的重心坐标。
 * 
 * @param a, b, c 三角形的三个顶点。
 * @param p 需要计算重心坐标的点。
 * @return 该点的重心坐标。
 */
Vec3 barycentricCoordinates(const Vec3 &a, const Vec3 &b, const Vec3 &c,
                            const Vec3 &p) {
    Vec3 m = (b - a).cross(c - a);
    float nu, nv, ood;
    float x = std::abs(m.x());
    float y = std::abs(m.y());
    float z = std::abs(m.z());

    if (x >= y && x >= z) {
        nu = triArea2D(p.y(), p.z(), b.y(), b.z(), c.y(), c.z());
        nv = triArea2D(p.y(), p.z(), c.y(), c.z(), a.y(), a.z());
        ood = 1.f / m.x();
    } else if (y >= x && y >= z) {
        nu = triArea2D(p.x(), p.z(), b.x(), b.z(), c.x(), c.z());
        nv = triArea2D(p.x(), p.z(), c.x(), c.z(), a.x(), a.z());
        ood = 1.f / -m.y();
    } else {
        nu = triArea2D(p.x(), p.y(), b.x(), b.y(), c.x(), c.y());
        nv = triArea2D(p.x(), p.y(), c.x(), c.y(), a.x(), a.y());
        ood = 1.f / m.z();
    }

    float u = nu * ood;
    float v = nv * ood;
    float w = 1.f - u - v;
    return Vec3(u, v, w);
}

/**
 * @brief 计算胶囊体的体积。
 * 
 * @param xa 胶囊体的一个端点。
 * @param xb 胶囊体的另一个端点。
 * @param ra xa点的半径。
 * @param rb xb点的半径。
 * @param capA 是否考虑xa端的半球体。
 * @param capB 是否考虑xb端的半球体。
 * @return 胶囊体的体积。
 */
float pillVolume(const Vec3 &xa, const Vec3 &xb, float ra, float rb, bool capA,
                 bool capB) {
    float V = 0.0f;
    float d = (xa - xb).norm();
    if (d > 1e-7f) {
        float e = (rb - ra) / d;
        float L = d + (ra - rb) * e;
        V += M_PIf / 3.0f *
             ((ra * ra * ra - rb * rb * rb) * (e * e * e - 3.f * e) +
              L * (1.f - e * e) * (ra * ra + ra * rb + rb * rb));
    }
    if (capA)
        V += 2.0f * M_PIf / 3.0f * ra * ra * ra;
    if (capB)
        V += 2.0f * M_PIf / 3.0f * rb * rb * rb;

    return V;
}
} // namespace Utils

} // namespace viper