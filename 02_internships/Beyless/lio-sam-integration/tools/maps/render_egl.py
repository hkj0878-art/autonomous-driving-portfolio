"""Isolated EGL renderer: driver failures cannot terminate the exporter."""
import sys
import resource
resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
import numpy as np
import open3d as o3d
from matplotlib import colormaps

if __name__ == '__main__':
    source, target = sys.argv[1:]
    a = np.load(source)
    xyz, path = a['xyz'], a['path']
    lo, hi = np.percentile(xyz[:, 2], [2, 98])
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
    cloud.colors = o3d.utility.Vector3dVector(colormaps['viridis'](np.clip((xyz[:,2]-lo)/max(hi-lo, .01),0,1))[:,:3])
    renderer = o3d.visualization.rendering.OffscreenRenderer(1800, 1400)
    renderer.scene.set_background([1,1,1,1])
    renderer.scene.view.set_post_processing(False)
    material = o3d.visualization.rendering.MaterialRecord()
    material.shader = 'defaultUnlit'; material.point_size = 3.0
    renderer.scene.add_geometry('map', cloud, material)
    lines = o3d.geometry.LineSet(o3d.utility.Vector3dVector(path), o3d.utility.Vector2iVector(np.column_stack([np.arange(len(path)-1),np.arange(1,len(path))])))
    lines.paint_uniform_color([.84,.37,0])
    material_line = o3d.visualization.rendering.MaterialRecord()
    material_line.shader = 'unlitLine'; material_line.line_width = 3
    renderer.scene.add_geometry('trajectory', lines, material_line)
    center = (xyz.min(axis=0)+xyz.max(axis=0))/2
    span = max(np.ptp(xyz[:,:2],axis=0))
    renderer.setup_camera(55, center, center+span*np.array([.45,-.65,.95]), [0,0,1])
    if not o3d.io.write_image(target, renderer.render_to_image()):
        raise RuntimeError('EGL image write failed')
