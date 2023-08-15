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

#include "SphereWeights.h"

#include <algorithm>
#include <cfloat>
#include <numeric>

#include <Eigen/Sparse>
#include <Eigen/SparseQR>

#include <igl/harmonic.h>

#include <OpenGP/Image/Image.h>
#include <OpenGP/SphereMesh/helpers.h>

using namespace OpenGP;
using Vec2i = Eigen::Vector2i;
using Vec3i = Eigen::Vector3i;
using Vec4i = Eigen::Vector4i;

/**
 * @brief 计算球形网格上的权重并应用到表面网格上
 * 
 * @param smesh 输入的球形网格
 * @param mesh 输出的表面网格，用于存储权重
 */
void calc_weights(const SphereMesh &smesh, SurfaceMesh &mesh) {

    // 获取球形网格的顶点属性（4D坐标点）
    auto vpoint = smesh.get_vertex_property<Vec4>("v:point");

    /**
     * @brief 分配函数，用于计算权重
     * 
     * @param p 输入的3D点
     * @param edge 输入的球形网格的边
     * @return Vec2 返回的2D向量权重
     */
    auto distribute = [&](Vec3 p, SphereMesh::Edge edge) -> Vec2 {
        Vec4 s0 = vpoint[smesh.vertex(edge, 0)];
        Vec4 s1 = vpoint[smesh.vertex(edge, 1)];

        Vec3 c0 = s0.head<3>();
        Vec4 a = s1 - s0;
        Vec3 d = a.head<3>();
        float l = d.norm();
        Vec3 an = d / l;
        Vec3 d2 = p - c0;
        Vec3 p_proj = d2 - an * d2.dot(an);
        float beta = asin(a(3) / l);
        Vec3 offset = an * p_proj.norm() * tan(beta);
        float t = an.dot(p + p_proj + offset - c0) / l;
        t = fmax(fmin(t, 1), 0);
        return Vec2(1 - t, t);
    };

    // 计算球形网格的边数和表面网格的顶点数
    int n_pills = smesh.n_edges();
    int n_verts = mesh.n_vertices();

    // 初始化权重矩阵P
    Eigen::MatrixXf P(n_verts, n_pills);
    P.setZero();

    float lambda = 0.1;

    // 遍历每个表面网格的顶点
    for (int i = 0; i < n_verts; ++i) {

        auto vert_i = SurfaceMesh::Vertex(i);

        // 计算权重
        for (int j = 0; j < smesh.n_edges(); ++j) {

            auto edge_j = SphereMesh::Edge(j);

            Vec4 s0 = vpoint[smesh.vertex(edge_j, 0)];
            Vec4 s1 = vpoint[smesh.vertex(edge_j, 1)];

            float sdf;
            pill_project(mesh.position(vert_i), s0, s1, &sdf);

            P(i, j) = 1.f / (1e-6 + std::pow(std::max(sdf, 0.f), 2.f));
        }

        // 归一化权重
        float sum = P.row(i).sum();
        P.row(i) /= sum;
    }

    P *= lambda;

    // 构建顶点和面的矩阵V和F
    Eigen::Matrix<double, -1, -1> V(n_verts, 3);
    Eigen::Matrix<int, -1, -1> F(mesh.n_faces(), 3);

    for (int i = 0; i < n_verts; ++i) {
        V.row(i) =
            mesh.position(SurfaceMesh::Vertex(i)).transpose().cast<double>();
    }

    for (int i = 0; i < mesh.n_faces(); ++i) {
        int j = 0;
        for (auto vert : mesh.vertices(SurfaceMesh::Face(i)))
            F(i, j++) = vert.idx();
    }

    // 使用IGL库计算谐波坐标
    Eigen::SparseMatrix<double> Ld(n_verts, n_verts);

    igl::harmonic(V, F, 1, Ld);

    for (int i = 0; i < n_verts; ++i) {
        Ld.coeffRef(i, i) += lambda;
    }

    // 转换数据类型并进行权重求解
    Eigen::SparseMatrix<float> L = Ld.cast<float>();

    Eigen::SparseLU<Eigen::SparseMatrix<float>> solver(L);

    Eigen::MatrixXf weights(n_verts, n_pills);
    for (int i = 0; i < n_pills; ++i) {
        weights.col(i) = solver.solve(P.col(i));
    }

    // 存储权重和骨骼ID
    auto weights_prop =
        mesh.add_vertex_property<std::vector<float>>("v:skinweight");
    auto bone_ids_prop = mesh.add_vertex_property<std::vector<int>>("v:boneid");

    // 遍历每个顶点并分配权重和骨骼ID
    for (int i = 0; i < n_verts; ++i) {
        Vec3 p = mesh.position(SurfaceMesh::Vertex(i));
        Eigen::VectorXf row = weights.row(i).transpose();
        std::vector<float> per_cap_weights;
        for (int j = 0; j < n_pills; ++j) {
            Vec2 cap_weights = distribute(p, SphereMesh::Edge(j)) * row[j];
            per_cap_weights.push_back(cap_weights[0]);
            per_cap_weights.push_back(cap_weights[1]);
        }
        std::vector<int> inds(2 * n_pills);
        std::iota(inds.begin(), inds.end(), 0);

        // 根据权重排序骨骼ID
        std::sort(inds.begin(), inds.end(), [&](int i, int j) {
            return per_cap_weights[i] > per_cap_weights[j];
        });

        std::vector<float> w;
        std::vector<int> wi;
        for (int j = 0; j < inds.size(); ++j) {
            w.push_back(per_cap_weights[inds[j]]);
            wi.push_back(inds[j]);
            if (w.back() / w.front() < 0.01)
                break;
        }

        // 归一化权重
        float sum = std::accumulate(w.begin(), w.end(), 0.f);
        std::transform(w.begin(), w.end(), w.begin(),
                       [sum](float x) { return x / sum; });

        SurfaceMesh::Vertex vert(i);
        weights_prop[vert] = w;
        bone_ids_prop[vert] = wi;

    }

}