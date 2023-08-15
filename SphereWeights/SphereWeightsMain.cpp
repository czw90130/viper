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

#include <fstream>
#include <iostream>
#include <thread>

#include "Viper_json.h"

// 定义需要使用的命名空间和数据结构类型
using namespace OpenGP;
using Vec2i = Eigen::Vector2i;
using Vec3i = Eigen::Vector3i;
using Vec4i = Eigen::Vector4i;


// 从标准输入或指定的文件中读取JSON格式的输入数据。这些数据主要包含顶点、三角形、球体和药丸信息。
// 根据输入数据，创建两种不同类型的网格：SurfaceMesh和SphereMesh。
// 调用calc_weights函数为这两种网格计算权重。
// 从计算得到的权重中提取骨骼的权重和骨骼ID。
// 将骨骼权重和骨骼ID转换为JSON格式并输出到标准输出。

// 调用方法示例：
// 使用标准输入运行程序：
// echo '{"vertices": [...], "triangles": [...], ...}' | ./SphereWeights
// 从文件中读取输入数据：
// ./SphereWeights input_filename.json
// 程序将处理输入数据，并将结果输出到标准输出。你也可以将其重定向到文件以保存结果
// ./SphereWeights input_filename.json > output_filename.json
int main(int argc, char **argv) {

    std::clog << "Starting SphereWeights:" << std::endl;

    // 默认从标准输入读取数据
    std::istream *stream_ptr = &std::cin;

    // 检查命令行参数，如果存在，则从文件读取数据
    std::ifstream file_stream;
    if (argc > 1) {
        file_stream = std::ifstream(argv[1]);
        stream_ptr = &file_stream;
    }

    std::istream &in_stream = *stream_ptr;

    std::string input, line;

    // 读取整个输入到字符串变量
    while (std::getline(in_stream, line)) {
        input += line + "\n";
    }

    // 使用rapidjson解析输入数据
    rapidjson::Document j;
    j.Parse(input.c_str());

    // 解析顶点、三角形、球体和药丸数据
    auto verts = viper::from_json<std::vector<Vec3>>(j["vertices"]);
    auto tris = viper::from_json<std::vector<Vec3i>>(j["triangles"]);
    auto spheres = viper::from_json<std::vector<Vec4>>(j["spheres"]);
    auto pills = viper::from_json<std::vector<Vec2i>>(j["pills"]);

    // 初始化SurfaceMesh和SphereMesh
    SurfaceMesh mesh;
    SphereMesh smesh;

    // 为SurfaceMesh添加顶点和三角形
    for (auto vert : verts) {
        mesh.add_vertex(vert);
    }
    for (auto tri : tris) {
        using V = SurfaceMesh::Vertex;
        mesh.add_triangle(V(tri[0]), V(tri[1]), V(tri[2]));
    }

    // 为SphereMesh添加顶点和边
    for (auto sphere : spheres) {
        smesh.add_vertex(sphere);
    }
    for (auto pill : pills) {
        using V = SphereMesh::Vertex;
        smesh.add_edge(V(pill[0]), V(pill[1]));
    }

    // 调用先前定义的函数，计算权重
    calc_weights(smesh, mesh);

    // 从mesh中获取权重和骨骼ID数据
    auto &weights =
        mesh.get_vertex_property<std::vector<float>>("v:skinweight").vector();
    auto &bone_ids =
        mesh.get_vertex_property<std::vector<int>>("v:boneid").vector();

    {
        rapidjson::MemoryPoolAllocator<> alloc;
        rapidjson::Document j(&alloc);
        j.SetObject();
        rapidjson::Document::AllocatorType &allocator = j.GetAllocator();

        // 将权重数据转换为JSON格式
        rapidjson::Value weights_array(rapidjson::kArrayType);
        for (auto &w : weights) {
            weights_array.PushBack(viper::to_json(w, allocator), allocator);
        }
        j.AddMember("weights", weights_array, allocator);

        // 将骨骼ID数据转换为JSON格式
        rapidjson::Value ids_array(rapidjson::kArrayType);
        for (auto &i : bone_ids) {
            ids_array.PushBack(viper::to_json(i, allocator), allocator);
        }
        j.AddMember("bone_ids", ids_array, allocator);

        // 将JSON数据写入字符串缓冲区
        rapidjson::StringBuffer strbuf;
        strbuf.Clear();
        rapidjson::Writer<rapidjson::StringBuffer> writer(strbuf);
        j.Accept(writer);

        // 输出结果
        std::cout << strbuf.GetString() << std::endl;
    }

    return 0;
}