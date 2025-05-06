#include "aljnu_path/bezier.h"

Bezier::Bezier() : Node("bezier_path") {
    pub_ = this->create_publisher<TrajectoryMsg>("path", 10);
    sub_ = this->create_subscription<PoseStampedMsg>(
        "goal", 10, std::bind(&Bezier::goal_callback, std::placeholders::_1));
}

void Bezier::goal_callback(const PoseStampedMsg::ConstPtr &msg) {
    msg->pose.position.x;
    msg->pose.position.x;
    msg->pose.position.x;
}
void Bezier::compute_pose(double t) {
    // control points
    double p0[2] = {0.0, 0.0};
    double p1[2] = {1.0, 1.0};
    double p2[2] = {2.0, 1.0};
    double p3[2] = {3.0, 0.0};

    double x = (1 - t) * ((1 - t) * ((1 - t) * p0[0] + t * p1[0]) +
                          t * ((1 - t) * p1[0] + t * p2[0])) +
               t * ((1 - t) * ((1 - t) * p1[0] + t * p2[0]) +
                    t * ((1 - t) * p2[0] + t * p3[0]));
    double y = (1 - t) * ((1 - t) * ((1 - t) * p0[1] + t * p1[1]) +
                          t * ((1 - t) * p1[1] + t * p2[1])) +
               t * ((1 - t) * ((1 - t) * p1[1] + t * p2[1]) +
                    t * ((1 - t) * p2[1] + t * p3[1]));
}

void Bezier::compute_velo(double t) {
}

void Bezier::compute_accl(double t) {
}