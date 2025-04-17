#include <rclcpp/rclcpp.hpp>

class Bezier {
    private:
        /* data */
    public:
        Bezier(/* args */);
        void compute_pose(double t);
        void compute_velo(double t);
        void compute_accl(double t);
};
