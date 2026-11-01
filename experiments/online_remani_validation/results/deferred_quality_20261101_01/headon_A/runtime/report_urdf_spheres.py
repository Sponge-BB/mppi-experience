#!/usr/bin/env python3
"""Offline fixed-structure geometry comparison. Does not modify ROS parameters."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.font_manager import FontProperties
from matplotlib.patches import Circle
import numpy as np

from urdf_sphere_model import build_model, load_stl


ROOT = Path(__file__).resolve().parents[2]
FONT = FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')


def original_base_spheres():
    """Reproduce current getCarPts, zero pose/inflation, physical radius .08 m."""
    radius = .08
    corners = np.array([[.17, .17], [.17, -.17], [-.17, -.17], [-.17, .17]])
    perimeter = list(corners)
    for start, stop in zip(corners, np.roll(corners, -1, axis=0)):
        length = np.linalg.norm(stop-start)
        for distance in np.arange(radius, length, radius):
            perimeter.append(start+(stop-start)*distance/length)
    return np.array([[*point, height, radius]
                     for height in np.arange(radius, .6, radius) for point in perimeter])


def component_triangles(component):
    """Display collision geometry, not the differently oriented visual meshes."""
    if component['geometry_kind'] != 'mesh':
        return None
    triangles = load_stl(Path(component['source_path']))
    transform = np.array(component['base_link_T_collision'])
    # Mesh scale is part of the collision local coordinates used by bounds.
    scale = np.array(component.get('mesh_scale', [1., 1., 1.]))
    return (triangles*scale) @ transform[:3, :3].T + transform[:3, 3]


def primitive_triangles(component):
    """Cylinder surfaces for drawing only; coverage uses an analytic sphere."""
    bounds = component['local_bounds']
    low, high = np.array(bounds['min']), np.array(bounds['max'])
    radius, half_length = high[0], high[2]
    angles = np.linspace(0., 2.*np.pi, 49)
    ring = np.column_stack([radius*np.cos(angles), radius*np.sin(angles)])
    faces = []
    for a, b in zip(ring[:-1], ring[1:]):
        for z in (-half_length, half_length):
            faces.append([[0., 0., z], [*a, z], [*b, z]])
        faces.extend([[[*a, -half_length], [*b, -half_length], [*a, half_length]],
                      [[*b, -half_length], [*a, half_length], [*b, half_length]]])
    transform = np.array(component['base_link_T_collision'])
    return np.array(faces) @ transform[:3, :3].T + transform[:3, 3]


def outward_bbox_excess(component, spheres):
    """Local-axis envelope excess, NOT distance from the true mesh surface."""
    transform = np.array(component['base_link_T_collision'])
    centers = np.array([s['center_base_link'] for s in spheres])
    local = (centers-transform[:3, 3]) @ transform[:3, :3]
    radii = np.array([s['radius'] for s in spheres])[:, None]
    bounds = component['local_bounds']
    lower = np.maximum(0., np.array(bounds['min'])-np.min(local-radii, axis=0))
    upper = np.maximum(0., np.max(local+radii, axis=0)-np.array(bounds['max']))
    return float(max(lower.max(), upper.max()))


def statistics(model):
    rows = []
    for component in model['components']:
        spheres = [s for s in model['spheres']
                   if s['link'] == component['link']
                   and s['collision_index'] == component['collision_index']]
        rows.append(dict(link=component['link'], count=len(spheres),
                         radius_min_m=min(s['radius'] for s in spheres),
                         radius_max_m=max(s['radius'] for s in spheres),
                         max_local_axis_bbox_excess_m=outward_bbox_excess(component, spheres)))
    return dict(count=len(model['spheres']), reduction_from_140=1.-len(model['spheres'])/140.,
                components=rows)


def render(models, out):
    triangles = []
    for component in models['compact']['components']:
        points = component_triangles(component)
        if points is None:
            points = primitive_triangles(component)
        points[:, :, 2] += models['compact']['frame_contract']['ground_offset']
        triangles.append(points)
    triangles = np.concatenate(triangles)
    old = original_base_spheres()
    options = [('当前底盘近似模型：140球', old),
               ('候选A：85球，较贴合', models['compact']),
               ('候选B：47球，较粗', models['coarse'])]
    fig, axes = plt.subplots(2, 3, figsize=(13, 8))
    for col, (title, model) in enumerate(options):
        balls = model if isinstance(model, np.ndarray) else np.array([
            [*s['center_ground'], s['radius']] for s in model['spheres']])
        for row, (first, second, name) in enumerate(((0, 1, '俯视'), (0, 2, '侧视'))):
            ax = axes[row, col]
            ax.add_collection(PolyCollection(triangles[:, :, [first, second]],
                                            facecolors='#555b62', edgecolors='none', alpha=.35))
            # Projection of the geometric spheres only, not a new 2-D collision gate.
            projected = np.unique(np.round(balls[:, [first, second, 3]], 9), axis=0)
            for x, y, radius in projected:
                ax.add_patch(Circle((x, y), radius, facecolor='none',
                                    edgecolor='#3275a1', linewidth=.65, alpha=.65))
            ax.set_aspect('equal')
            ax.set_xlim(-.46, .46)
            ax.set_ylim((-.44, .44) if row == 0 else (-.09, .91))
            if row == 1:
                ax.axhline(0., color='#999999', linewidth=.7)
            ax.set_xlabel('x (m)')
            ax.set_ylabel(('y' if row == 0 else 'z')+' (m)')
            ax.set_title(title+' / '+name, fontproperties=FONT, fontsize=12)
            ax.grid(alpha=.15)
    fig.suptitle('灰色：URDF固定部件；蓝色：几何球投影（不含安全余量）',
                 fontproperties=FONT, fontsize=15)
    fig.text(.5, .018, '不含运动机械臂、相机和夹爪；两档候选尚未接入规划器或实车',
             ha='center', fontproperties=FONT, fontsize=11)
    fig.tight_layout(rect=(0., .045, 1., .95))
    fig.savefig(out/'固定结构碰撞球对照.png', dpi=140)
    plt.close(fig)


def generate(urdf, output):
    models = {profile: build_model(urdf, profile=profile, ground_offset=.115)
              for profile in ('coarse', 'compact')}
    output.mkdir(parents=True, exist_ok=False)
    for profile, model in models.items():
        (output/('candidate_'+profile+'.json')).write_text(
            json.dumps(model, ensure_ascii=False, indent=2)+'\n')
    stats = {name: statistics(model) for name, model in models.items()}
    (output/'comparison.json').write_text(json.dumps(stats, ensure_ascii=False, indent=2)+'\n')
    render(models, output)
    lines = [
        'URDF碰撞球简化：独立几何候选，不是已经部署的避障版本', '',
        '一、这次做了什么',
        '按当前exp0_real实际加载的my_description/robot/mm.urdf，拆分底盘、轮子、支架、顶板、固定臂座和雷达。',
        '不再把这些固定部件统一当作0.5×0.5×0.6米的高箱体。活动机械臂、相机、夹爪本轮未简化。', '',
        '二、球数',
        '当前底盘近似模型：140球，并非已经逐部件验证覆盖全部URDF固定结构。',
        '候选A：85球，查询球数减少39.3%，分格更细，建议优先用于下一步隔离验证。',
        '候选B：47球，查询球数减少66.4%，球更大，局部外包更宽，不代表效果更好。',
        '这不是整机最终球数；不能把候选固定球加原20个臂球就宣称完整覆盖整台机器人。', '',
        '三、怎么保证没有为了减球而漏掉这些URDF固定部件',
        '网格部件：取collision局部包围盒，分成小盒，每格用外接球覆盖；解析上覆盖整个包围盒体积。',
        '圆柱部件：按sqrt(半径²+半长²)生成外接球，解析上覆盖圆柱。',
        '应用了collision origin和固定关节变换。输出保留link局部覆盖信息，不靠某帧世界坐标来代表模型。',
        '代价是部分空白空间也被包住。图里球变少不等于外形更紧，更不等于实车安全已验证。', '',
        '四、必须说明的输入问题',
        'URDF中arm_base网格软链失效；本候选明确使用同工作空间meshes/arm/base_link.stl替代并记录，没有修改原软链。',
        '采用现有配置base_link离地0.115米：URDF臂座离地0.688米，原规划参数0.680米，两者差8毫米。未擅自改安装参数。',
        'URDF雷达外壳仍是VLP16，而实车入口使用Livox话题/坐标系。候选只覆盖URDF描述，不等于已核实真实雷达外形。',
        '原20个臂球未证明覆盖相机和夹爪，后续扩展机械臂必须单独处理。', '',
        '五、这次没有改什么',
        '没有修改src、devel、现有实验构建或实车参数；没有启动ROS、发运动指令。',
        '候选球半径仅描述几何，不包含原base硬检查额外0.05米或优化软余量0.10米；图也没有叠加这些余量。',
        '没有测量MPPI/REMANI在线耗时或避障成功率，不能用减少球的比例推算实际提速。', '',
        '六、接入时真正要改的地方',
        '当前代码所有底盘球共用一个半径。候选是不同半径，不能只替换中心坐标。',
        '需要让中心、梯度、各类碰撞/自碰撞检查、优化代价、可视化和MPPI导出统一读取同一组逐球半径。',
        '球数变化还会改变逐球累加的代价大小，需与几何提速分开验证，不能直接当作等价替换。',
        '下一步应先接候选A到独立构建，做同输入耗时和一次在线闭环；通过后再考虑实车。', '',
        '附：每个部件球数、几何半径及局部包围盒最大外扩（不是到真实表面的误差）',
    ]
    for profile in ('compact', 'coarse'):
        lines.append(profile+':')
        for item in stats[profile]['components']:
            lines.append('  %s: %d球，半径%.1f~%.1f厘米，轴向包围盒最大外扩%.1f厘米' % (
                item['link'], item['count'], 100*item['radius_min_m'],
                100*item['radius_max_m'], 100*item['max_local_axis_bbox_excess_m']))
    (output/'简化说明.txt').write_text('\n'.join(lines)+'\n')
    return stats


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf', type=Path, default=ROOT/'src/my_description/robot/mm.urdf')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(generate(args.urdf, args.output), ensure_ascii=False))
