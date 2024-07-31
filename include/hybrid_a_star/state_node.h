/*******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2022 Zhang Zhimeng
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 * WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#ifndef HYBRID_A_STAR_STATE_NODE_H
#define HYBRID_A_STAR_STATE_NODE_H

#include "type.h"

#include <Eigen/Dense>

struct StateNode {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // 内存对齐，提高速度

    enum NODE_STATUS {  // 节点状态：未被访问/正在操作/已被访问
        NOT_VISITED = 0, IN_OPENSET = 1, IN_CLOSESET = 2
    };

    enum DIRECTION {    // 方向：前进/后退/...
        FORWARD = 0, BACKWARD = 1, NO = 3
    };

    StateNode() = delete; // 默认构造函数不能被调用，否则会报错

    // 构造函数
    explicit StateNode(const Vec3i &grid_index) {  // explicit 阻止隐式类型转换
        node_status_ = NOT_VISITED;
        grid_index_ = grid_index;
        parent_node_ = nullptr;
    }

    // Reset方法
    void Reset() {
        node_status_ = NOT_VISITED;
        parent_node_ = nullptr;
    }

    NODE_STATUS node_status_;
    DIRECTION direction_{};

    Vec3d state_;
    Vec3i grid_index_;

    double g_cost_{}, f_cost_{};
    int steering_grade_{};

    StateNode *parent_node_;   // 父节点
    typedef StateNode *Ptr;

    VectorVec3d intermediate_states_;  // 中间态(OpenSet)
};

#endif //HYBRID_A_STAR_STATE_NODE_H
