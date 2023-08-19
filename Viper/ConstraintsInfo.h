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

/**
 * @brief 约束信息结构体
 * 
 * 该结构体用于存储和管理场景中不同类型的约束信息。
 * 约束 (Constraint): 在物理模拟中，约束定义了某些物体或物体部分之间的相对关系。这些关系可以是固定的，也可以是可变的。约束通常用于确保模拟在某些条件下保持稳定，例如确保物体不会穿透其他物体。
 * 投影 (Projection): 在约束求解的上下文中，投影通常指的是将物体从违反约束的位置移动到满足约束的位置的过程。例如，如果两个物体被约束为不得接触，但由于某种原因它们接触了，那么它们的位置会被“投影”回到不接触的位置。
 * Lambda: 在某些物理模拟算法中，尤其是那些使用拉格朗日乘子的方法来解决约束的算法中，lambda代表了约束的强度或权重。实际上，它是约束修正的一个因子，决定了应用于物体以满足约束的力的大小。
 * 投影偏移量 (Projection Offset): 这可能是指用于存储每种约束类型投影数据的数组中的起始位置。在具有多种约束类型的复杂模拟中，每种约束可能需要不同数量的数据来描述。投影偏移量可以帮助我们快速找到某种约束类型的投影数据的开始位置。
 * Lambda偏移量 (Lambda Offset): 与投影偏移量类似，这可能是指用于存储每种约束类型的lambda数据的数组中的起始位置。
 */
struct ConstraintsInfo {
    /**
     * @brief 添加一个新的约束类型的信息。
     * 
     * 该函数允许用户添加关于新约束类型的信息，如其名称、数量、与其关联的投影和lambda数量等。
     * 每种约束类型的信息都会存储在各自的数组中，并用于后续的模拟和计算。
     * 
     * @param name 约束的名称。每种约束都应该有一个独特的名称。
     * @param count 当前约束类型的数量。
     * @param projections_per_constraint 每个约束的投影数量。这表示为了满足这种类型的约束，可能需要进行多少次投影。
     * @param lambda_per_constraint 每个约束的lambda数量。lambda是用于约束修正的因子，表示每种约束的强度或权重。
     */
    void add(const std::string &name, int count, int projections_per_constraint,
            int lambda_per_constraint) {
        // 确保同名的约束还没有被添加过
        assert(index.find(name) == index.end());

        // 获取np的大小作为新约束类型的索引值
        int i = np.size();

        // 将约束名称与其索引值相关联
        index[name] = i;

        // 如果这是第一次添加约束，设置初始的投影和lambda偏移量为0
        if (i == 0) {
            o.push_back(0);
            ol.push_back(0);
        } else {
            // 否则，根据前一个约束的偏移量和数量，计算新的偏移量
            o.push_back(o.back() + np.back());
            ol.push_back(ol.back() + nl.back());
        }

        // 存储当前约束类型的数量
        nc.push_back(count);
        // 存储当前约束类型的总投影数量
        np.push_back(count * projections_per_constraint);
        // 存储当前约束类型的总lambda数量
        nl.push_back(count * lambda_per_constraint);
    }


    /**
     * @brief 获取约束的总数量
     * 
     * @return 约束的总数量
     */
    int get_nc() {
        int n = 0;
        for (int i = 0; i < nc.size(); i++) {
            n += nc[i];
        }
        return n;
    }

    /**
     * @brief 获取投影的总数量
     * 
     * @return 投影的总数量
     */
    int get_np() {
        if (np.size() == 0)
            return 0;
        return o.back() + np.back();
    }

    /**
     * @brief 获取lambda的总数量
     * 
     * @return lambda的总数量
     */
    int get_nl() {
        if (np.size() == 0)
            return 0;
        return ol.back() + nl.back();
    }

    /**
     * @brief 获取每个约束类型的投影偏移量
     * 
     * @return 约束类型与其对应的投影偏移量的映射
     */
    std::map<std::string, int> get_o() {
        std::map<std::string, int> m;
        for (auto const &x : index)
            m[x.first] = o[x.second];
        return m;
    }

    /**
     * @brief 获取每个约束类型的lambda偏移量
     * 
     * @return 约束类型与其对应的lambda偏移量的映射
     */
    std::map<std::string, int> get_ol() {
        std::map<std::string, int> m;
        for (auto const &x : index)
            m[x.first] = ol[x.second];
        return m;
    }

  private:
    std::map<std::string, int> index;
    std::vector<int> nc; ///< 约束名到其在数组中的索引的映射 number of constraints per type
    std::vector<int> np; ///< 每种约束类型的数量 number of projections per type
    std::vector<int> nl; ///< 每种约束类型的投影数量 number of lambdas per type
    std::vector<int> o;  ///< 每种约束类型的lambda数量 projection offset per type
    std::vector<int> ol; ///< 每种约束类型的lambda偏移量 lambda offset per type
};