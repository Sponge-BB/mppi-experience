#include "mm_config/mm_config.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {
void require(bool ok, const char *message) {
    if (!ok) throw std::runtime_error(message);
}
double number(XmlRpc::XmlRpcValue &value) {
    if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
        return static_cast<int>(value);
    require(value.getType() == XmlRpc::XmlRpcValue::TypeDouble,
            "sphere parameter is not numeric");
    return static_cast<double>(value);
}
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "urdf_spheres_b_model_test");
    ros::NodeHandle nh("~");
    try {
        XmlRpc::XmlRpcValue rows;
        require(nh.getParam("mm/fixed_collision_spheres", rows),
                "missing candidate B sphere parameter");
        require(rows.getType() == XmlRpc::XmlRpcValue::TypeArray && rows.size() == 47,
                "candidate B must contain 47 spheres");
        std::vector<Eigen::Vector4d> expected;
        double expected_max_radius = 0.0;
        for (int i = 0; i < rows.size(); ++i) {
            require(rows[i].getType() == XmlRpc::XmlRpcValue::TypeArray && rows[i].size() == 4,
                    "candidate sphere row must have four values");
            Eigen::Vector4d sphere;
            for (int j = 0; j < 4; ++j) sphere(j) = number(rows[i][j]);
            expected.push_back(sphere);
            expected_max_radius = std::max(expected_max_radius, sphere(3));
        }

        remani_planner::MMConfig model;
        model.setParam(nh);
        require(model.hasCustomBaseSpheres(), "candidate model was not activated");
        require(std::abs(model.getBaseMaxCheckRadius() - expected_max_radius) < 1e-12,
                "maximum radius is inconsistent");
        const std::vector<double> yaws{-2.7, -1.1, -0.2, 0.0, 0.6, 2.5};
        double centre_error = 0.0, radius_error = 0.0;
        for (std::size_t k = 0; k < yaws.size(); ++k) {
            const Eigen::Vector3d state(0.31 * k - 0.7, 0.17 * k - 0.4, yaws[k]);
            Eigen::Matrix2d rotation;
            rotation << std::cos(state.z()), -std::sin(state.z()),
                        std::sin(state.z()), std::cos(state.z());
            std::vector<Eigen::Vector3d> points;
            model.getCarPts(state, points, Eigen::Vector3d::Zero());
            require(points.size() == expected.size(), "candidate sphere count changed");
            for (std::size_t i = 0; i < points.size(); ++i) {
                const Eigen::Vector2d xy = state.head<2>() + rotation * expected[i].head<2>();
                const Eigen::Vector3d target(xy.x(), xy.y(), expected[i].z());
                centre_error = std::max(centre_error, (points[i] - target).norm());
                radius_error = std::max(radius_error,
                    std::abs(model.getBaseSphereRadius(i) - expected[i](3)));
            }
        }
        require(centre_error < 1e-12 && radius_error < 1e-12,
                "sphere transform or radius mismatch");

        const double step = 1e-6;
        double yaw_gradient_error = 0.0, velocity_gradient_error = 0.0;
        double gradient_point_error = 0.0;
        for (int direction : {-1, 1}) {
            for (std::size_t k = 0; k < yaws.size(); ++k) {
                const Eigen::Vector2d pos(0.12 * k - 0.3, 0.27 * k - 0.1);
                const Eigen::Vector2d velocity(0.24 * std::cos(yaws[k]),
                                                0.24 * std::sin(yaws[k]));
                const double yaw = model.calYaw(velocity, direction);
                std::vector<Eigen::Vector3d> points, velocity_points, plus, minus;
                std::vector<Eigen::Vector2d> yaw_gradient;
                std::vector<Eigen::Matrix2d> velocity_gradient;
                model.getCarPtsGradNew(pos, velocity, direction, Eigen::Vector3d::Zero(),
                                       points, yaw_gradient);
                model.getCarPtsGrad(pos, velocity, direction, Eigen::Vector3d::Zero(),
                                    velocity_points, velocity_gradient);
                require(points.size() == 47 && yaw_gradient.size() == 47 &&
                        velocity_points.size() == 47 && velocity_gradient.size() == 47,
                        "gradient arrays are not aligned to the 47 spheres");
                model.getCarPts(Eigen::Vector3d(pos.x(), pos.y(), yaw + step), plus);
                model.getCarPts(Eigen::Vector3d(pos.x(), pos.y(), yaw - step), minus);
                for (std::size_t i = 0; i < points.size(); ++i) {
                    const Eigen::Vector2d numerical =
                        (plus[i] - minus[i]).head<2>() / (2.0 * step);
                    yaw_gradient_error = std::max(yaw_gradient_error,
                        (numerical - yaw_gradient[i]).norm());
                    gradient_point_error = std::max(gradient_point_error,
                        (points[i] - velocity_points[i]).norm());
                }
                for (int axis = 0; axis < 2; ++axis) {
                    Eigen::Vector2d vp = velocity, vm = velocity;
                    vp(axis) += step;
                    vm(axis) -= step;
                    model.getCarPts(Eigen::Vector3d(pos.x(), pos.y(), model.calYaw(vp, direction)), plus);
                    model.getCarPts(Eigen::Vector3d(pos.x(), pos.y(), model.calYaw(vm, direction)), minus);
                    for (std::size_t i = 0; i < points.size(); ++i) {
                        const Eigen::Vector2d numerical =
                            (plus[i] - minus[i]).head<2>() / (2.0 * step);
                        velocity_gradient_error = std::max(velocity_gradient_error,
                            (numerical - velocity_gradient[i].col(axis)).norm());
                    }
                }
            }
        }
        require(yaw_gradient_error < 1e-7 && velocity_gradient_error < 1e-7 &&
                gradient_point_error < 1e-12, "analytical collision centre gradient mismatch");
        bool inflation_rejected = false;
        try {
            std::vector<Eigen::Vector3d> points;
            model.getCarPts(Eigen::Vector3d::Zero(), points, Eigen::Vector3d(0.01, 0, 0));
        } catch (const std::invalid_argument &) { inflation_rejected = true; }
        require(inflation_rejected, "unsupported custom inflation was silently ignored");

        std::vector<double> q_param;
        require(nh.getParam("mm/manipulator_min_pos", q_param) && q_param.size() == 6,
                "missing fixed arm pose");
        Eigen::VectorXd q(6);
        for (int i = 0; i < 6; ++i) q(i) = q_param[i];
        double hard_distance = 0.0, safe_distance = 0.0;
        const bool base_arm_hard_collision = model.checkCarManiCollision(q, false, hard_distance);
        const bool base_arm_safe_collision = model.checkCarManiCollision(q, true, safe_distance);
        // A changed self-collision outcome must be visible to the caller, not
        // hidden by disabling the checker or claiming the candidate is ready.

        bool test_explicit_switch = false;
        nh.param("test_explicit_sphere_switch", test_explicit_switch, false);
        if (test_explicit_switch) {
            // Production can switch OFF without deleting rows left in a
            // long-lived parameter server. This branch is not used by SIL.
            nh.setParam("mm/fixed_collision_spheres_enable", false);
            remani_planner::MMConfig disabled;
            disabled.setParam(nh);
            std::vector<Eigen::Vector3d> disabled_points;
            disabled.getCarPts(Eigen::Vector3d::Zero(), disabled_points);
            require(nh.hasParam("mm/fixed_collision_spheres") &&
                    !disabled.hasCustomBaseSpheres() && disabled_points.size() == 140,
                    "OFF did not restore legacy geometry with stale rows present");
            nh.setParam("mm/fixed_collision_spheres_enable", true);
        }

        require(nh.deleteParam("mm/fixed_collision_spheres"), "could not remove custom parameter in isolated test master");
        if (test_explicit_switch) {
            bool missing_rows_rejected = false;
            try {
                remani_planner::MMConfig missing;
                missing.setParam(nh);
            } catch (const std::invalid_argument &) { missing_rows_rejected = true; }
            require(missing_rows_rejected, "enabled production model silently accepted missing rows");
            nh.setParam("mm/fixed_collision_spheres_enable", false);
        }
        remani_planner::MMConfig legacy;
        legacy.setParam(nh);
        std::vector<Eigen::Vector3d> legacy_points;
        legacy.getCarPts(Eigen::Vector3d::Zero(), legacy_points);
        require(!legacy.hasCustomBaseSpheres() && legacy_points.size() == 140,
                "missing custom parameter did not restore the legacy 140-sphere model");
        for (std::size_t i = 0; i < legacy_points.size(); ++i)
            require(std::abs(legacy.getBaseSphereRadius(i) - 0.08) < 1e-12,
                    "legacy sphere radius changed");

        std::cout << std::setprecision(17)
                  << "URDF_SPHERES_B_RESULT {\"passed\":true,\"sphere_count\":47,"
                  << "\"legacy_sphere_count\":" << legacy_points.size()
                  << ",\"max_radius\":" << expected_max_radius
                  << ",\"centre_max_error\":" << centre_error
                  << ",\"radius_max_error\":" << radius_error
                  << ",\"yaw_gradient_max_error\":" << yaw_gradient_error
                  << ",\"velocity_gradient_max_error\":" << velocity_gradient_error
                  << ",\"gradient_centres_max_error\":" << gradient_point_error
                  << ",\"nonzero_inflation_rejected\":true"
                  << ",\"base_arm_hard_collision\":" << (base_arm_hard_collision ? "true" : "false")
                  << ",\"base_arm_safe_collision\":" << (base_arm_safe_collision ? "true" : "false")
                  << ",\"base_arm_hard_checker_distance\":" << hard_distance
                  << ",\"base_arm_safe_checker_distance\":" << safe_distance
                  << "}" << std::endl;
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "URDF_SPHERES_B_ERROR " << error.what() << std::endl;
        return 1;
    }
}
