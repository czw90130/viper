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

#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_triangle_primitive.h>
#include <CGAL/Simple_cartesian.h>
#include <Eigen/Dense>
#include <vector>

namespace Eigen {
/// @brief 
/**
 * @brief 用于在网格上计算最近点的加速数据结构。
 * acceleration data structure for closest point computation on a mesh
 *
 * @tparam VertexMatrix 顶点的矩阵类型。
 * @tparam FaceMatrix 面的矩阵类型。
 */
template <typename VertexMatrix, typename FaceMatrix> class TrimeshSearcher {
    /// CGAL TYPES CGAL类型定义
    typedef CGAL::Simple_cartesian<typename VertexMatrix::Scalar> K;
    typedef typename K::FT FT;
    typedef typename K::Point_3 Point_3;
    typedef typename K::Triangle_3 Triangle;
    typedef typename std::vector<Triangle>::iterator Iterator;
    typedef typename CGAL::AABB_triangle_primitive<K, Iterator> Primitive;
    typedef typename CGAL::AABB_traits<K, Primitive> AABB_triangle_traits;
    typedef typename CGAL::AABB_tree<AABB_triangle_traits> Tree;
    typedef typename Tree::Point_and_primitive_id Point_and_primitive_id;
    /// Eigen Types Eigen类型定义
    typedef Matrix<typename FaceMatrix::Scalar, 3, 1> Face;
    typedef Matrix<typename VertexMatrix::Scalar, 3, 1> Vertex;
    typedef typename FaceMatrix::Scalar FaceIndex;

  public:
    /**
     * @brief 将Eigen类型转换为CGAL类型。 Conversion Eigen-CGAL
     */
    static inline Point_3 tr(Vertex v) { return Point_3(v.x(), v.y(), v.z()); }
    /**
     * @brief 将CGAL类型转换为Eigen类型。
     */
    static inline Vertex tr(Point_3 v) { return Vertex(v.x(), v.y(), v.z()); }

  private:
    Tree tree; ///< AABB树结构
    std::vector<Triangle> triangles; ///< 三角形集合

  public:
    /**
     * @brief 构建用于查找三角形网格上最近点的加速结构。
     * Builds the acceleration structure to look for closest
     * points on a triangular mesh
     *
     * @param vertices 网格的顶点（每列一个顶点
     * vertices of the surface (one vertex per column)
     * @param faces faces.col(i) 包含面上的顶点索引
     * faces.col(i) contains the indexes of vertices on the face
     */
    template <typename Derived1, typename Derived2>
    void build(MatrixBase<Derived1> &vertices, MatrixBase<Derived2> &faces) {
        /// Bake triangle set
        for (int fi = 0; fi < faces.cols(); fi++) {
            Face f = faces.col(fi);
            Point_3 v0 = tr(vertices.col(f[0]));
            Point_3 v1 = tr(vertices.col(f[1]));
            Point_3 v2 = tr(vertices.col(f[2]));
            triangles.push_back(Triangle(v0, v1, v2));
        }
        // constructs AABB tree
        tree.rebuild(triangles.begin(), triangles.end());
    }

    /**
     * @brief 查找表面上的最近点。
     * Find closest points on the surface
     *
     * 每一列一个查询。One query per column
     * 仅返回足迹点。Only returns footpoints
     * @param queries 查询点云（每列一个点）
     * query point cloud (one point per column)
     * @param footpoints 输入三角形集上找到的最近点（每列一个点）
     * fetched closest point on the input triangle set
     * (one point per column)
     */
    template <typename Derived1, typename Derived2>
    void closest_point(const MatrixBase<Derived1> &queries,
                       MatrixBase<Derived2> &footpoints) {
        for (int iq = 0; iq < queries.cols(); iq++) {
            Point_3 query = tr(queries.col(iq));
            Point_3 pp = tree.closest_point(query);
            footpoints.col(iq) = tr(pp);
        }
    }

    /**
     * @brief 查找表面上的最近点。
     * Find closest points on the surface
     *
     * @param queries 查询点云（每列一个点）
     * query point cloud (one point per column)
     * @param footpoints 输入三角形集上找到的最近点（每列一个点）
     * fetched closest point on the input triangle set (one
     * point per column)
     * @param indexes 找到足迹点的三角形的索引（每个查询一个索引）
     * index of triangle on which the footpoint was found
     * (one index per query)
     * @note 对退化的三角形没有保证。矩阵求逆将失败！
     * NO INSURANCE on degenerate triangles. Matrix inversion will fail!!
     */
    template <typename Derived1, typename Derived2, typename Derived3>
    void closest_point(const MatrixBase<Derived1> &queries,
                       MatrixBase<Derived2> &footpoints,
                       MatrixBase<Derived3> &indexes) {
        for (int iq = 0; iq < queries.cols(); iq++) {
            Point_3 query = tr(queries.col(iq));
            Point_and_primitive_id pp = tree.closest_point_and_primitive(query);
            Iterator id = pp.second;
            std::size_t index_in_vector = std::distance(triangles.begin(), id);
            assert(triangles[index_in_vector] == *id);
            footpoints.col(iq) = tr(pp.first);
            indexes(iq) = index_in_vector;
        }
    }

    /**
     * @brief 将{3D足迹点,面索引}对转换为其重心坐标。
     * Converts the pair of {3D footpoint,face index} into its
     * barycentric coordinate coordinates
     *
     * @param footpoints 每列一个3D足迹点
     * one 3D footpoint per column
     * @param indexes 足迹点找到的面的索引
     * index of the face where footpoint was found
     * @param coordinates 对应面中足迹点的重心坐标（对退化的三角形不进行检查！）
     * barycentric coordinates of the footpoint in the corresponding face
     * (not checked for degenerate triangles!!!)
     */
    template <typename Derived1, typename Derived2, typename Derived3>
    void barycentric(const MatrixBase<Derived1> &footpoints,
                     const MatrixBase<Derived2> &indexes,
                     MatrixBase<Derived3> &coordinates) {
        Matrix<FT, 3, 3> A;
        for (int iq = 0; iq < footpoints.cols(); iq++) {
            const Vertex &footpoint = footpoints.col(iq);
            FaceIndex fi = indexes(iq);
            Triangle &triangle = triangles[fi];
            A.col(0) = tr(triangle[0]);
            A.col(1) = tr(triangle[1]);
            A.col(2) = tr(triangle[2]);
            coordinates.col(iq) = A.inverse() * footpoint;
        }
    }
};
} // namespace Eigen
