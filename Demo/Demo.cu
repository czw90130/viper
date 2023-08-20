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

#include <cfloat>
#include <fstream>
#include <iostream>
#include <thread>

// 定义tiny object loader的实现
#define TINYOBJLOADER_IMPLEMENTATION
// 使用PNG格式
#define USE_PNG

// 引入OpenGP的库头文件
#include <OpenGP/GL/Application.h>
#include <OpenGP/GL/Components/GUICanvasComponent.h>
#include <OpenGP/GL/ImguiRenderer.h>
#include <OpenGP/Image/Image.h>

// 引入本地的头文件
#include "OctopusComponent.h"
#include "Scene.h"

// 引入CUDA头文件来处理碰撞
#include "CollisionGrid.cuh"

// 定义所有OpenGP的实现在这个文件
#define OPENGP_IMPLEMENT_ALL_IN_THIS_FILE
#include <OpenGP/util/implementations.h>

// 使用OpenGP的命名空间
using namespace OpenGP;

int main(int argc, char **argv) {

    // 定义阴影的大小
    int shadow_size = 2048;

    // 初始化应用
    Application app;

    // 初始化场景
    Scene scene;

    // 创建一个实体作为灯源，并附带一个摄像机组件
    auto &light_entity = scene.create_entity_with<CameraComponent>();
    // 设置灯源的方向
    light_entity.get<TransformComponent>().set_forward(
        Vec3(-1, -2, 0).normalized());
    // 设置灯源的位置
    light_entity.get<TransformComponent>().position = Vec3(50, 100, 0);

    // 计算投影和视图矩阵，用于计算阴影
    Mat4x4 shadow_matrix =
        (light_entity.get_projection(shadow_size, shadow_size) *
         light_entity.get_view());

    // 创建一个用于渲染的地板实体
    auto &floor_entity = scene.create_entity_with<WorldRenderComponent>();
    // 设置地板的渲染器
    auto &floor_renderer = floor_entity.set_renderer<SurfaceMeshRenderer>();
    floor_renderer.get_gpu_mesh().set_vpoint(
        {Vec3(-10000, 0, -10000), Vec3(10000, 0, -10000),
         Vec3(-10000, 0, 10000), Vec3(10000, 0, 10000)});
    floor_renderer.get_gpu_mesh().set_vnormal(
        {Vec3(0, 1, 0), Vec3(0, 1, 0), Vec3(0, 1, 0), Vec3(0, 1, 0)});
    floor_renderer.get_gpu_mesh().set_triangles({0, 1, 2, 1, 2, 3});

    Material floormat(R"GLSL(

        uniform sampler2D shadow_map;

        uniform vec3 light_pos;
        uniform mat4 shadow_matrix;
        uniform float shadow_near;
        uniform float shadow_far;

        vec3 world2uvdepth(vec3 pos, mat4 mat) {
            vec4 a = mat * vec4(pos, 1);
            vec3 b = a.xyz / a.w;
            return (b + vec3(1)) / 2;
        }

        float get_shadow_mask(vec2 uv) {
            return 1 - smoothstep(0.3, 0.5, length(uv - vec2(0.5, 0.5)));
        }

        vec3 get_ambient(vec3 pos) {
            vec3 ambient = vec3(0.14, 0.14, 0.18);

            vec3 uvd = world2uvdepth(pos, shadow_matrix);

            return ambient + vec3(0.2) * get_shadow_mask(uvd.xy);
        }

        float linear_shadow_depth(float d) {
            return shadow_near * shadow_far / (shadow_far + d * (shadow_near - shadow_far));
        }

        float get_shadow(vec3 pos) {
            ivec2 dim = textureSize(shadow_map, 0);
            vec3 uvd = world2uvdepth(pos, shadow_matrix);

            vec2 base_coord = uvd.xy * dim;
            ivec2 base_coord_i = ivec2(floor(base_coord));
            vec2 inter = fract(base_coord);

            mat4 shadow_depths;
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    shadow_depths[i][j] = linear_shadow_depth(texelFetch(shadow_map, base_coord_i + ivec2(i-1, j-1), 0).r);
                }
            }

            float threshold = linear_shadow_depth(uvd.z) - 0.1;

            mat2 pcf_vals = mat2(0);
            for (int i = 0; i < 2; ++i) {
                for (int j = 0; j < 2; ++j) {
                    for (int x = 0; x < 3; ++x) {
                        for (int y = 0; y < 3; ++y) {
                            pcf_vals[i][j] += (shadow_depths[x + i][y + j] < threshold) ? 0 : (1.0 / 9.0);
                        }
                    }
                }
            }

            float a = mix(pcf_vals[0][0], pcf_vals[1][0], inter.x);
            float b = mix(pcf_vals[0][1], pcf_vals[1][1], inter.x);

            return mix(a, b, inter.y) * get_shadow_mask(uvd.xy);
        }

        vec4 fragment_shade() {
            vec3 pos = get_position();

            vec3 lightdir = normalize(light_pos - pos);

            vec3 white_color = vec3(1, 1, 1);
            vec3 black_color = vec3(0.6, 0.6, 0.6);

            vec3 background = (white_color + black_color) / 2;

            vec3 diffuse_color = white_color;

            vec3 modpos = mod(pos / 5, 1);

            if ((modpos.x < 0.5) ^^ (modpos.z < 0.5)) {
                diffuse_color = black_color;
            }

            float blur = exp(-2 * max(length(dFdx(pos)), length(dFdy(pos))));
            blur = clamp(2 * blur, 0, 1);

            diffuse_color = mix(background, diffuse_color, blur);

            vec3 ambient = get_ambient(pos);

            float shadow = get_shadow(pos);

            vec3 out_color = shadow * 0.85 * clamp(dot(get_normal(), normalize(lightdir)), 0, 1) * diffuse_color;

            out_color += ambient * diffuse_color;

            return vec4(out_color, 1);
        }

    )GLSL");
    floormat.set_property("ao_map", 6);
    floormat.set_property("shadow_map", 7);
    floormat.set_property("shadow_matrix", shadow_matrix);
    floormat.set_property("light_pos",
                          light_entity.get<TransformComponent>().position);
    floormat.set_property("shadow_near", light_entity.near_plane);
    floormat.set_property("shadow_far", light_entity.far_plane);
    floor_renderer.set_material(floormat);
    floor_renderer.rebuild();

    // 创建一个模拟场景  
    viper::Scene sim_scene;

    // 为OctopusComponent设置模拟场景
    OctopusComponent::v_scene = &sim_scene;
    // 在场景中创建一个实体并附加一个OctopusComponent组件
    auto &octoswarm = scene.create_entity_with<OctopusComponent>();

    // 为octoswarm的渲染器设置阴影矩阵属性
    octoswarm.renderer->get_material().set_property("shadow_matrix",
                                                    shadow_matrix);
    // 为octoswarm的渲染器设置光源位置属性
    octoswarm.renderer->get_material().set_property(
        "light_pos", light_entity.get<TransformComponent>().position);
    // 为octoswarm的渲染器设置阴影近裁剪面属性
    octoswarm.renderer->get_material().set_property("shadow_near",
                                                    light_entity.near_plane);
    // 为octoswarm的渲染器设置阴影远裁剪面属性
    octoswarm.renderer->get_material().set_property("shadow_far",
                                                    light_entity.far_plane);

    
    // 为octoswarm的sphere渲染器设置相同的属性
    octoswarm.sphere_renderer->get_material().set_property("shadow_matrix",
                                                           shadow_matrix);
    octoswarm.sphere_renderer->get_material().set_property(
        "light_pos", light_entity.get<TransformComponent>().position);
    octoswarm.sphere_renderer->get_material().set_property(
        "shadow_near", light_entity.near_plane);
    octoswarm.sphere_renderer->get_material().set_property(
        "shadow_far", light_entity.far_plane);

    // 为octoswarm的tsphere渲染器设置相同的属性
    octoswarm.tsphere_renderer->get_material().set_property("shadow_matrix",
                                                            shadow_matrix);
    octoswarm.tsphere_renderer->get_material().set_property(
        "light_pos", light_entity.get<TransformComponent>().position);
    octoswarm.tsphere_renderer->get_material().set_property(
        "shadow_near", light_entity.near_plane);
    octoswarm.tsphere_renderer->get_material().set_property(
        "shadow_far", light_entity.far_plane);

    // 为octoswarm的cannonball渲染器设置相同的属性
    octoswarm.cannonball_renderer->get_material().set_property("shadow_matrix",
                                                               shadow_matrix);
    octoswarm.cannonball_renderer->get_material().set_property(
        "light_pos", light_entity.get<TransformComponent>().position);
    octoswarm.cannonball_renderer->get_material().set_property(
        "shadow_near", light_entity.near_plane);
    octoswarm.cannonball_renderer->get_material().set_property(
        "shadow_far", light_entity.far_plane);

    // 为octoswarm的pillar渲染器设置相同的属性
    octoswarm.pillar_renderer->get_material().set_property("shadow_matrix",
                                                           shadow_matrix);
    octoswarm.pillar_renderer->get_material().set_property(
        "light_pos", light_entity.get<TransformComponent>().position);
    octoswarm.pillar_renderer->get_material().set_property(
        "shadow_near", light_entity.near_plane);
    octoswarm.pillar_renderer->get_material().set_property(
        "shadow_far", light_entity.far_plane);

    // 创建一个带有Trackball组件的实体
    auto &c_entity = scene.create_entity_with<TrackballComponent>();
    c_entity.oriented = true; // 设置实体的方向属性为true


    // 设置窗口的宽和高
    int ww = 1024, wh = 768;

    // 以下是创建和管理Framebuffer，纹理等的代码

    // 定义Framebuffer对象
    Framebuffer fb, fb_shadow;
    // 定义颜色纹理对象
    RGB8Texture color_map, color_map_shadow;
    // 定义深度纹理对象
    D32FTexture depth_map, depth_map_shadow;

    // 定义一个lambda函数，用于重新分配纹理空间大小
    auto realloc = [&](int w, int h) {
        color_map.allocate(w, h); // 重新分配颜色纹理的空间
        depth_map.allocate(w, h); // 重新分配深度纹理的空间
    };

    realloc(ww, wh); // 使用上面定义的窗口宽高来重新分配纹理空间大小

    // 为阴影纹理重新分配指定大小的空间
    depth_map_shadow.allocate(shadow_size, shadow_size);
    color_map_shadow.allocate(shadow_size, shadow_size);

    // 将纹理对象附加到Framebuffer上
    fb.attach_color_texture(color_map);
    fb.attach_depth_texture(depth_map);

    fb_shadow.attach_color_texture(color_map_shadow);
    fb_shadow.attach_depth_texture(depth_map_shadow);

    // 创建一个RGB颜色纹理对象
    RGB8Texture colmap;
    // 创建一个CPU端的RGB颜色图像对象，大小为2048x2048
    Image<Eigen::Matrix<uint8_t, 3, 1>> colmap_cpu(2048, 2048);
    // 从"texture.bin"文件中读取数据到colmap_cpu对象
    std::ifstream("texture.bin", std::ios::binary).read(
        reinterpret_cast<char*>(&colmap_cpu(0, 0)), 12582912);
    colmap.upload(colmap_cpu); // 将CPU端的图像数据上传到GPU纹理对象中

    // 创建一个全屏的四边形对象
    FullscreenQuad fsquad;

    // 定义两个布尔变量，分别用于表示是否显示pill和是否分屏显示
    bool show_pills = false;
    bool splitscreen = false;

    // 定义一个lambda函数，用于设置pill的可见性
    auto set_pill_visibility = [&](bool visible) {
        show_pills = visible; // 设置是否显示pill
        octoswarm.render_comp->visible = !visible;  // 设置octoswarm的渲染组件是否可见
        octoswarm.sphere_render_comp->visible = visible; // 设置octoswarm的球形渲染组件是否可见
        octoswarm.vis_update(); // 更新octoswarm的可见性
    };

    // 定义一个Lambda函数来绘制场景
    auto draw_scene = [&](int width, int height, int x, int y) {
        //======================================================================
        // Draw shadow map 绘制阴影映射

        fb_shadow.bind(); // 绑定阴影缓冲

        // 使用光源实体绘制阴影贴图
        light_entity.draw(shadow_size, shadow_size);

        fb_shadow.unbind(); // 解绑阴影缓冲

        //======================================================================
        // Draw scene with shadows 使用阴影绘制场景

        fb.bind(); // 绑定帧缓冲对象

        // 设置活跃纹理单元并绑定相关纹理
        glActiveTexture(GL_TEXTURE5);
        colmap.bind();
        glActiveTexture(GL_TEXTURE7);
        depth_map_shadow.bind();

        glActiveTexture(GL_TEXTURE0); // 设定默认纹理单元为纹理0

        auto &cam = c_entity.get<CameraComponent>(); // 获取相机组件

        // 使用相机绘制场景
        cam.draw(color_map.get_width(), color_map.get_height(), 0, 0, false);

        if (octoswarm.sphere_render_comp->visible) {
            RenderContext context; // 创建渲染上下文

            glDepthMask(GL_FALSE); // 禁用深度缓冲写入

            // 设置渲染上下文的参数
            context.aspect =
                (float)color_map.get_width() / (float)color_map.get_height();
            context.vfov = cam.vfov;
            context.near = cam.near_plane;
            context.far = cam.far_plane;
            context.eye = cam.get<TransformComponent>().position;
            context.forward = cam.get<TransformComponent>().forward();
            context.up = cam.get<TransformComponent>().up();

            // 更新视图和投影矩阵
            context.update_view();
            context.update_projection();

            // 获取渲染组件和变换组件
            auto &renderable = *octoswarm.tsphere_render_comp;
            auto &transform = renderable.get<TransformComponent>();

            // 设置模型变换参数
            context.translation = transform.position;
            context.scale = transform.scale;
            context.rotation = transform.rotation;

            // 更新模型矩阵
            context.update_model();

            glEnable(GL_DEPTH_TEST); // 启用深度测试
            renderable.get_renderer().render(context); // 使用指定的渲染器渲染场景

            glDepthMask(GL_TRUE); // 启用深度缓冲写入
        }

        // 绘制相机的图形用户界面（GUI）
        cam.draw_gui();

        fb.unbind(); // 解绑帧缓冲对象

        //======================================================================
        // Draw color map to window 将颜色映射绘制到窗口

        // 设置视口大小和位置
        glViewport(x, y, width, height);
        // 使用全屏四边形绘制纹理
        fsquad.draw_texture(color_map);
    };

    // 创建一个窗口
    auto &window = app.create_window([&](Window &window) {
        // 获取窗口的大小
        std::tie(ww, wh) = window.get_size();

        // 如果开启了分屏功能，那么新的帧缓冲宽度为窗口宽度的一半，否则为窗口的完整宽度
        int fbw_new = splitscreen ? ww / 2 : ww;
        int fbh_new = wh;

        // 获取当前的帧缓冲宽度和高度
        int fbw = color_map.get_width();
        int fbh = color_map.get_height();

        // 如果新的帧缓冲宽度或高度与当前的不匹配，重新分配内存
        if (fbw_new != fbw || fbh_new != fbh) {
            realloc(fbw_new, fbh_new);
        }

        // 如果开启了分屏功能
        if (splitscreen) {
            // 关闭pill的可见性，并在窗口的左半部分绘制场景
            set_pill_visibility(false);
            draw_scene(ww / 2, wh, 0, 0);
            // 开启pill的可见性，并在窗口的右半部分绘制场景
            set_pill_visibility(true);
            draw_scene(ww / 2, wh, ww / 2, 0);
        } else {
            // 更新octoswarm的视觉
            octoswarm.vis_update();
            // 在整个窗口中绘制场景
            draw_scene(ww, wh, 0, 0);
        }
    });

    // 设置窗口大小和标题
    window.set_size(ww, wh);
    window.set_title("VIPER Demo");

    // 获取窗口的输入对象，用于处理输入事件和UI交互
    auto &input = window.get_input();

    // 设置摄像头关联的窗口，并初始化摄像头的位置和焦点位置
    c_entity.get<CameraComponent>().set_window(window);
    c_entity.center = Vec3(0, 1, 0);
    c_entity.get<TransformComponent>().position = Vec3(-12, 1, 0);

    // 在场景中创建一个实体并为其添加WorldRenderComponent组件
    auto &bsphere_entity = scene.create_entity_with<WorldRenderComponent>();
    // 设置实体的渲染器为SphereMeshRenderer
    auto &bsphere_renderer = bsphere_entity.set_renderer<SphereMeshRenderer>();

    // 定义一个函数，用于获取鼠标当前位置对应的射线
    auto get_mouse_ray = [&](Vec3 &eye, Vec3 &dir) {
        // 获取鼠标的屏幕位置
        Vec2 pos = input.mouse_position;
        // 对鼠标的y坐标进行反转（原点从左上角转到左下角）
        pos[1] = wh - pos[1];

        // 根据是否分屏来确定宽度
        int w = splitscreen ? ww / 2 : ww;
        // 将鼠标的屏幕坐标映射到[-1, 1]的范围内
        pos = 2 * pos.cwiseQuotient(Vec2(w, wh)) - Vec2(1, 1);

        // 创建一个裁剪空间坐标
        Vec4 cs(pos[0], pos[1], 0.1, 1);

        // 获取摄像头组件并计算逆投影矩阵
        auto &cam = c_entity.get<CameraComponent>();
        Mat4x4 inv_mat = (cam.get_projection(w, wh) * cam.get_view()).inverse();

        // 将裁剪空间坐标转换到世界空间坐标
        Vec4 world = inv_mat * cs;
        Vec3 p = world.head<3>() / world[3];

        // 获取摄像头的位置
        eye = c_entity.get<TransformComponent>().position;
        // 计算鼠标射线的方向
        dir = (p - eye).normalized();
    };

    int framerate = 0; // 帧率
    double frametime = 0; // 帧时间
    double sim_frametime = 0; // 模拟帧时间

    float playback = 1.0; // 播放速率

    int it_count = 10; // 迭代次数

    bool hide_gui = false; // 是否隐藏图形用户界面
    bool simulating = true; // 是否正在模拟
    bool single_step = false; // 是否进行单步模拟
    bool bsphere_vis = false; // 是否可见边界球

    std::vector<float> framerates(120); // 帧率数组，大小为120

    // 设置默认值的lambda函数
    auto set_defaults = [&]() {
        show_pills = false;
        octoswarm.render_comp->visible = !show_pills; // 默认不显示药丸
        octoswarm.sphere_render_comp->visible = show_pills; // octoswarm的渲染组件是否可见
        it_count = 10; // 默认的迭代次数
        sim_scene.gravity_strength = 1.0; // 默认的重力强度
        playback = 1.0; // 默认的播放速率
    };

    set_defaults(); // 调用上面的设置默认值函数

    // 创建一个带有GUICanvasComponent的实体
    auto &canvas = scene.create_entity_with<GUICanvasComponent>();

    // 设置画布的操作函数
    canvas.set_action([&]() {
        if (hide_gui)
            return;

        ImGui::SetNextWindowSize(ImVec2(400, 500)); // 设置窗口的大小

        // 开始绘制控制窗口
        ImGui::Begin("Controls", nullptr,
                     ImGuiWindowFlags_NoResize |
                         ImGuiWindowFlags_NoSavedSettings);

        char fr_label[256];
        // 格式化帧率和帧时间标签
        sprintf(fr_label,
                "Framerate %i fps\n Total:   %3.1f ms\n Sim:     %3.1f ms",
                framerate, frametime, sim_frametime);

        ImGui::PlotLines(fr_label, &(framerates[0]), framerates.size(), 0, "",
                         0, 60);

        ImGui::Separator();

        // “重置”按钮
        if (ImGui::Button("Reset")) {
            octoswarm.reset();
        }

        ImGui::SameLine(0, 4);

        // “暂停”或“继续”按钮
        const char *bname = simulating ? "Pause" : "Resume";
        if (ImGui::Button(bname)) {
            simulating = !simulating;
        }

        // 如果没有在模拟中
        if (!simulating) {
            ImGui::SameLine(0, 4);
            // “单步”按钮
            if (ImGui::Button("Step")) {
                single_step = true;
            }
        }

        // “分屏”复选框
        ImGui::Checkbox("Split Screen", &splitscreen);

        // “显示基本形状”复选框
        if (ImGui::Checkbox("Show Primitives", &show_pills)) {
            set_pill_visibility(show_pills);
        }

        // “重力”滑动条
        ImGui::SliderFloat("Gravity", &sim_scene.gravity_strength, -1.0f, 3.0f);
        // “求解器迭代次数”滑动条
        ImGui::SliderInt("Solver Iterations", &it_count, 0, 50);

        // “设置默认值”按钮
        if (ImGui::Button("Set Defaults")) {
            set_defaults();
            octoswarm.vis_update();
        }

        ImGui::Separator();

        const char *const scenes[] = {"Empty", "Pillars", "Cannonballs",
                                      "Explosion"};

        // “场景”列表框
        if (ImGui::ListBox("Scenes", &octoswarm.scene_index, scenes,
                           sizeof(scenes) / sizeof(scenes[0]))) {
            octoswarm.reset();
        }

        ImGui::Separator();

        // 显示各种控制指南
        ImGui::LabelText("Controls", "Look:                   Middle Mouse");
        ImGui::Text("Recenter:                Right Mouse");
        ImGui::Text("Pan:            Shift + Middle Mouse");
        ImGui::Text("Grab:                     Left Mouse");
        ImGui::Text("Shoot:                      Spacebar");
        ImGui::Text("Toggle Primitives:               F10");
        ImGui::Text("Pause/Resume:                    F11");
        ImGui::Text("Show/Hide Window:                F12");

        ImGui::End(); // 结束绘制控制窗口
    });

    // 将canvas的相机设置为c_entity的相机组件
    canvas.set_camera(c_entity.get<CameraComponent>());

    // 初始化装载的牛的数量为0
    int chambered_cow = 0;

    // 初始化总帧数和模拟帧数
    long frame = 0;
    long sim_frame = 0;

    // 获取当前时间作为最后的时间
    double last_time = glfwGetTime();
    // 初始化帧平均值和模拟帧平均值
    double frame_avg = 0;
    double sim_frame_avg = 0;

    // 初始化持有的时间和选定的项目
    int held = 0;
    int selected = -1;

    // 初始化几个标志变量
    bool swapped_pills = false;
    bool swapped_pause = false;
    bool swapped_window = false;
    bool recentered = false;

    // 添加一个应用程序更新事件的监听器
    app.add_listener<ApplicationUpdateEvent>(
        [&](const ApplicationUpdateEvent &) {
            // 创建一个临时的球形网格
            SphereMesh temp_smesh;
            // 在球形网格中添加一个顶点
            auto vs_temp =
                temp_smesh.add_vertex(viper::CollisionGrid::b_sphere);
            // 在球形网格中添加一个球
            temp_smesh.add_sphere(vs_temp);
            // 根据bsphere_vis来设置bsphere_entity的可见性
            bsphere_entity.visible = bsphere_vis;
            // 将球形网格上传到渲染器
            bsphere_renderer.upload_mesh(temp_smesh);

            // 当鼠标左键被按下时
            if (input.get_mouse(0)) {
                Vec3 eye, dir;
                // 获取鼠标的射线
                get_mouse_ray(eye, dir);

                // 如果没有选定的对象
                if (selected == -1) {
                    // 寻找与射线相交的对象并选定它
                    selected = octoswarm.intersect(eye, dir);
                    sim_scene.state.xa[selected] = 0;
                } else {
                    // 否则，将选定的对象移动到新的位置
                    Vec3 p = sim_scene.state.x[selected];

                    Vec3 x = p - eye;
                    Vec3 new_pos = p - (x - dir * dir.dot(x));
                    new_pos[1] =
                        std::max(new_pos[1], sim_scene.state.r[selected]);
                    sim_scene.state.x[selected] = new_pos;
                    sim_scene.state.xp[selected] = sim_scene.state.x[selected];
                }
            } else if (selected != -1) {
                // 如果鼠标左键没有被按下，且之前有选定的对象，则取消选定
                sim_scene.state.xa[selected] = 1;
                selected = -1;
            }

            // 当鼠标右键被按下时
            if (input.get_mouse(1)) {
                // 获取深度图像
                Image<float> depth_im;
                depth_map.download(depth_im);

                // 获取鼠标的位置
                int mxi = int(input.mouse_position[0]);
                int myi = int(wh - input.mouse_position[1]);

                // 获取当前的相机组件
                auto &cam = c_entity.get<CameraComponent>();
                // 计算逆矩阵
                Mat4x4 inv_mat =
                    (cam.get_projection() * cam.get_view()).inverse();

                // 如果没有重新居中，并且鼠标在窗口内
                if (!recentered &&
                    !(mxi < 0 || mxi >= ww || myi < 0 || myi >= wh)) {

                    // 获取深度值
                    Vec3 uvdepth;
                    uvdepth.head<2>() = Vec2(float(mxi) / ww, float(myi) / wh);
                    uvdepth[2] = min(depth_im(myi, mxi), 0.999);

                    // 将深度值转换为世界坐标
                    Vec4 dev(0, 0, 0, 1);
                    dev.head<3>() = 2 * uvdepth - Vec3::Ones();
                    Vec4 world_h = inv_mat * dev;

                    // 更新实体的中心和位置
                    Vec3 new_center = world_h.head<3>() / world_h[3];
                    Vec3 dc = new_center - c_entity.center;
                    c_entity.center += dc;
                    c_entity.get<TransformComponent>().position += dc;

                    recentered = true;
                }
            } else {
                recentered = false;
            }

            // F10键控制药丸的可见性
            if (input.get_key(GLFW_KEY_F10)) {
                if (!swapped_pills) {
                    set_pill_visibility(!show_pills);
                    swapped_pills = true;
                }
            } else {
                swapped_pills = false;
            }
            
            // F11键控制模拟的暂停和继续
            if (input.get_key(GLFW_KEY_F11)) {
                if (!swapped_pause) {
                    simulating = !simulating;
                    swapped_pause = true;
                }
            } else {
                swapped_pause = false;
            }
            
            // F12键控制GUI的显示和隐藏
            if (input.get_key(GLFW_KEY_F12)) {
                if (!swapped_window) {
                    hide_gui = !hide_gui;
                    swapped_window = true;
                }
            } else {
                swapped_window = false;
            }

            // 空格键控制发射牛
            if (input.get_key(GLFW_KEY_SPACE)) {
                std::cout << "Firing cow " << chambered_cow << std::endl;
                // 每当held变量是5的倍数时，执行以下操作
                // 这意味着用户必须连续按下空格键5次才会发射一次牛
                if ((held % 5) == 0) {
                    // 获取c_entity实体的位置
                    Vec3 p = c_entity.get<TransformComponent>().position;
                    // 获取c_entity实体的前进方向
                    Vec3 v = c_entity.get<TransformComponent>().forward();

                    // 设置选定的牛的位置和速度
                    // 新的位置是当前实体位置加上其前进方向的3倍
                    // 这意味着牛将在实体前方3单位的位置被发射
                    octoswarm.set_position(chambered_cow, p + 3 * v, v);

                    // 选择下一头牛进行发射
                    // 通过模运算确保chambered_cow值在0到octoswarm.n_cows之间
                    chambered_cow = (chambered_cow + 1) % octoswarm.n_cows;
                }
                // 增加held变量的值，以记录空格键被连续按下的次数
                held++;
            } else {
                // 如果空格键没有被按下，将held变量重置为0
                held = 0;
            }

            // 计算帧时间，并进行帧率控制
            double frame_time = 0.0;
            double this_time = last_time;
            while (frame_time < 0.016667) {
                this_time = glfwGetTime();
                frame_time = this_time - last_time;
                std::this_thread::yield();
            }
            last_time = this_time;

            // 更新帧率数组
            framerates.erase(framerates.begin());
            framerates.push_back(1.0 / frame_time);

            // 计算帧平均时间和帧率
            frame_avg += frame_time;

            if ((frame % 10) == 0) {
                frametime = 1000 * frame_avg / 10.0;
                framerate = 0.5 + 10.0 / frame_avg;
                frame_avg = 0;
            }

            // 如果正在进行模拟或者是单步模拟
            if (simulating || single_step) {
                double sim_time =
                    sim_scene.step(playback / 60.f, it_count, true);

                sim_frame_avg += sim_time;

                if ((sim_frame % 10) == 0) {
                    sim_frametime = sim_frame_avg / 10.0;
                    sim_frame_avg = 0;
                }

                single_step = false;

                sim_frame++;
            }

            // 更新场景
            scene.update();

            frame++;
        });

    app.run();

    return 0;
}