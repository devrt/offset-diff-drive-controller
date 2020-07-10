/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, MID Academic Promotions, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the MID Academic Promotions nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Yosuke Matsusaka
 */

/*
 * Dynamics of offset diff drive vehicle
 *  Equations are from the paper written by Masayoshi Wada etal.
 *  https://www.jstage.jst.go.jp/article/jrsj1983/18/8/18_8_1166/_pdf
 */

#include <cmath>
#include <offset_diff_drive_controller/dynamics.h>

namespace offset_diff_drive_controller
{

    void forward_dynamics(joint_param &input, state &state, cartesian_param &output)
    {
        double cos_s = cos(state.steer_angle);
        double sin_s = sin(state.steer_angle);
        output.dot_x = (state.wheel_radius / 2.0 * cos_s - state.wheel_radius * state.wheel_offset / state.wheel_separation * sin_s) * input.vel_wheel_r + (state.wheel_radius / 2.0 * cos_s + state.wheel_radius * state.wheel_offset / state.wheel_separation * sin_s) * input.vel_wheel_l;
        output.dot_y = (state.wheel_radius / 2.0 * sin_s + state.wheel_radius * state.wheel_offset / state.wheel_separation * cos_s) * input.vel_wheel_r + (state.wheel_radius / 2.0 * sin_s - state.wheel_radius * state.wheel_offset / state.wheel_separation * cos_s) * input.vel_wheel_l;
        output.dot_r = state.wheel_radius / state.wheel_separation * input.vel_wheel_r - state.wheel_radius / state.wheel_separation * input.vel_wheel_l - input.vel_steer;
    }

    void inverse_dynamics(cartesian_param &input, state &state, joint_param &output)
    {
        double cos_s = cos(state.steer_angle);
        double sin_s = sin(state.steer_angle);
        output.vel_wheel_r = (cos_s / state.wheel_radius - state.wheel_separation * sin_s / 2.0 / state.wheel_radius / state.wheel_offset) * input.dot_x + (sin_s / state.wheel_radius + state.wheel_separation * cos_s / 2.0 / state.wheel_radius / state.wheel_offset) * input.dot_y;
        output.vel_wheel_l = (cos_s / state.wheel_radius + state.wheel_separation * sin_s / 2.0 / state.wheel_radius / state.wheel_offset) * input.dot_x + (sin_s / state.wheel_radius - state.wheel_separation * cos_s / 2.0 / state.wheel_radius / state.wheel_offset) * input.dot_y;
        output.vel_steer = -sin_s / state.wheel_offset * input.dot_x + cos_s / state.wheel_offset * input.dot_y - input.dot_r;
    }

} // namespace offset_diff_drive_controller