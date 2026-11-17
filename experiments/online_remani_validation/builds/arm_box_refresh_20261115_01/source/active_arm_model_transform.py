"""Fail-closed source transforms for the isolated active-arm native build.

No production source is changed by importing or running this module. The builder
passes source text and writes its own isolated build. Self margin semantics are
preserved: use self_safe_margin=0 in the physical-gap experiment, since native
wrist geometry has a valid non-adjacent gap of only 3 mm.
"""

OLD_PIPER_BLOCK = '''    if(!useFastArmer_) {
        min_dist = 0.5;
        return false; // Disable self-collision for Piper arm temporarily to prevent false positives
    }'''

NEW_PIPER_BLOCK = '''    if(!useFastArmer_) {
        // ACTIVE_ARM_NATIVE20_SELF: exact pair policy of active_arm_model.py.
        if (mani_state.size() != 6 || !mani_state.allFinite() || manipulator_dof_ != 6) {
            min_dist = 0.0;
            return true;
        }
        const int expected_counts[6] = {3, 5, 2, 5, 1, 4};
        if (manipulator_link_pts_.size() != 6) { min_dist = 0.0; return true; }
        for (int link = 0; link < 6; ++link) {
            if (manipulator_link_pts_[link].cols() != expected_counts[link]) {
                min_dist = 0.0; return true;
            }
        }
        std::vector<Eigen::Matrix4d> transforms, unused_gradients;
        getJointTrans(mani_state, transforms, unused_gradients);
        if (transforms.size() != 6) { min_dist = 0.0; return true; }
        std::vector<Eigen::Vector3d> points;
        std::vector<int> links;
        Eigen::Matrix4d current = Eigen::Matrix4d::Identity();
        for (int link = 0; link < 6; ++link) {
            current = current * transforms[link];
            for (int sample = 0; sample < expected_counts[link]; ++sample) {
                points.push_back((current * manipulator_link_pts_[link].col(sample)).head<3>());
                links.push_back(link);
            }
        }
        min_dist = 1.0e30;
        for (int i = 0; i < 20; ++i) {
            for (int j = i + 1; j < 20; ++j) {
                // Same/adjacent links and three permanent joint-neighbour overlaps.
                if (links[j] - links[i] <= 1 ||
                    (i == 3 && j == 14) || (i == 10 && j == 19) ||
                    (i == 11 && j == 19)) continue;
                const double distance = (points[i] - points[j]).norm();
                if (!std::isfinite(distance)) { min_dist = 0.0; return true; }
                min_dist = std::min(min_dist, distance);
            }
        }
        const double threshold = 2.0 * manipulator_thickness_ + (safe ? self_safe_margin_ : 0.0);
        return min_dist < threshold;
    }'''


def transform_self_collision(source):
    """Restore Piper hard self checks, retaining unchanged FastArmer code."""
    if source.count(OLD_PIPER_BLOCK) != 1:
        raise ValueError('Expected exactly one disabled Piper self-collision block')
    return source.replace(OLD_PIPER_BLOCK, NEW_PIPER_BLOCK)


def transform_ground_checks(source):
    """Use physical sphere bottoms for the two native arm ground checks."""
    old = 'if(pt_on_link(2) < ground_safe_dis_)'
    if source.count(old) != 2:
        raise ValueError('Expected exactly two native arm ground checks')
    return source.replace(old, 'if(pt_on_link(2) - manipulator_thickness_ < ground_safe_dis_)')


def transform_mm_config(source):
    return transform_ground_checks(transform_self_collision(source))


transform = transform_mm_config
