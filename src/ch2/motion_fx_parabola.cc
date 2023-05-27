//
// Created by fx
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "common/eigen_types.h"
#include "common/math_utils.h"
#include "tools/ui/pangolin_window.h"

DEFINE_double(init_velocity, 10.0, "init velocity");
DEFINE_double(init_angle, 0.0, "init angle");
DEFINE_double(gravity, -9.8, "gravity acceleration");
DEFINE_double(angular_velocity, 10.0, "angular velocity");
DEFINE_double(decay_factor, 0.99, "decay factor");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    /// 可视化
    sad::ui::PangolinWindow ui;

    if (ui.Init() == false) {
        return -1;
    }

    double init_angle_rad = FLAGS_init_angle * sad::math::kDEG2RAD;   // 弧度制角速度
    Vec3d omega(0, 0, FLAGS_angular_velocity * sad::math::kDEG2RAD);  // 角速度矢量
    SE3 pose;                                                         // TWB表示的位姿
    Vec3d v_body(FLAGS_init_velocity * cos(init_angle_rad), 0,
                 FLAGS_init_velocity * sin(init_angle_rad));  // 角速度矢量
    const double dt = 0.01;

    LOG(INFO) << "pose: " << pose.translation().transpose();
    ui.UpdateNavState(sad::NavStated(0, pose, v_body));
    usleep(dt * 1e6);  // 每次更新的时间

    while (ui.ShouldQuit() == false) {
        v_body[2] += FLAGS_gravity * dt;
        Vec3d v_world = pose.so3() * v_body;
        pose.translation() += v_world * dt;
        pose.so3() = pose.so3() * SO3::exp(omega * dt);

//        // rebound
//        if (pose.translation()[2] < 0) {
//            v_body = v_body * FLAGS_decay_factor;
//            v_body[2] = -v_body[2];
//            pose.translation()[2] = -pose.translation()[2];
//        }

        LOG(INFO) << "pose: " << pose.translation().transpose();
        ui.UpdateNavState(sad::NavStated(0, pose, v_body));

        usleep(dt * 1e6);
    }

    ui.Quit();
    return 0;
}