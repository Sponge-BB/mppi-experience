with open("/home/hai/remani_ws/src/mm_remani_planner/mm_config/src/mm_config.cpp", "r") as f:
    lines = f.readlines()

new_block = """        // Piper collision spheres derived purely from Inverse DH mapping
        for(int i = 0; i < manipulator_dof_; ++i){
            switch(i){
            case 0:{ // a=0, d=0.123, alpha=-pi/2. V_back = (0, 0.123, 0)
                link_pts.resize(4, 3);
                link_pts.col(0) = Eigen::Vector4d(0, 0, 0, 1);
                link_pts.col(1) = Eigen::Vector4d(0, 0.06, 0, 1);
                link_pts.col(2) = Eigen::Vector4d(0, 0.123, 0, 1);
                break;
            }
            case 1:{ // a=0.285, d=0, alpha=0. V_back = (-0.285, 0, 0)
                link_pts.resize(4, 5);
                link_pts.col(0) = Eigen::Vector4d(0, 0, 0, 1);
                link_pts.col(1) = Eigen::Vector4d(-0.07, 0, 0, 1);
                link_pts.col(2) = Eigen::Vector4d(-0.14, 0, 0, 1);
                link_pts.col(3) = Eigen::Vector4d(-0.21, 0, 0, 1);
                link_pts.col(4) = Eigen::Vector4d(-0.285, 0, 0, 1);
                break;
            }
            case 2:{ // a=-0.022, d=0, alpha=pi/2. V_back = (0.022, 0, 0)
                link_pts.resize(4, 2);
                link_pts.col(0) = Eigen::Vector4d(0, 0, 0, 1);
                link_pts.col(1) = Eigen::Vector4d(0.022, 0, 0, 1);
                break;
            }
            case 3:{ // a=0, d=0.25, alpha=-pi/2. V_back = (0, 0.25, 0)
                link_pts.resize(4, 5);
                link_pts.col(0) = Eigen::Vector4d(0, 0, 0, 1);
                link_pts.col(1) = Eigen::Vector4d(0, 0.06, 0, 1);
                link_pts.col(2) = Eigen::Vector4d(0, 0.12, 0, 1);
                link_pts.col(3) = Eigen::Vector4d(0, 0.18, 0, 1);
                link_pts.col(4) = Eigen::Vector4d(0, 0.25, 0, 1);
                break;
            }
            case 4:{ // a=0, d=0, alpha=pi/2. V_back = (0, 0, 0)
                link_pts.resize(4, 1);
                link_pts.col(0) = Eigen::Vector4d(0, 0, 0, 1);
                break;
            }
            case 5:{ // a=0, d=0.211, alpha=0. V_back = (0, 0, -0.211)
                link_pts.resize(4, 4);
                link_pts.col(0) = Eigen::Vector4d(0, 0, 0, 1);
                link_pts.col(1) = Eigen::Vector4d(0, 0, -0.07, 1);
                link_pts.col(2) = Eigen::Vector4d(0, 0, -0.14, 1);
                link_pts.col(3) = Eigen::Vector4d(0, 0, -0.211, 1);
                break;
            }
            default:
                break;
            }
            manipulator_link_pts_.push_back(link_pts);
        }\n"""

start_idx = -1
end_idx = -1
for i, line in enumerate(lines):
    if "        // Piper collision spheres" in line:
        start_idx = i
    if start_idx != -1 and "        }" in line and i > start_idx:
        end_idx = i

if start_idx != -1:
    del lines[start_idx:end_idx+1]
    lines.insert(start_idx, new_block)

with open("/home/hai/remani_ws/src/mm_remani_planner/mm_config/src/mm_config.cpp", "w") as f:
    f.writelines(lines)
