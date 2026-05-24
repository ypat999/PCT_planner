#!/usr/bin/env python3
"""
Convert Gazebo SDF/DAE mesh to PCD point cloud for PCT planner.

Automatically reads pose offset from .world file and scale from model.sdf.
Filters near-vertical surfaces by default to simulate LiDAR-like point clouds
(walls and stair risers are removed, keeping floors and stair treads).

Usage:
    # Auto mode: read pose and scale from world + model files
    python3 mesh_to_pcd.py --world Building.world --output building.pcd

    # Manual mode: specify mesh, pose, scale directly
    python3 mesh_to_pcd.py --input Building.dae --output building.pcd \
        --pose 0 0 0 --scale 1 1 0.7

Supports: .dae (COLLADA), .obj, .stl, .ply, .glb
"""

import argparse
import os
import numpy as np
import trimesh
import open3d as o3d
import xml.etree.ElementTree as ET


def parse_pose(pose_str):
    vals = [float(v) for v in pose_str.strip().split()]
    x, y, z = vals[0], vals[1], vals[2]
    roll, pitch, yaw = (vals[3], vals[4], vals[5]) if len(vals) >= 6 else (0, 0, 0)
    return x, y, z, roll, pitch, yaw


def parse_scale(scale_str):
    vals = [float(v) for v in scale_str.strip().split()]
    return vals[0], vals[1], vals[2]


def yaw_to_rotation_matrix(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([
        [c, -s, 0],
        [s,  c, 0],
        [0,  0, 1],
    ])


def pose_to_transform(x, y, z, roll, pitch, yaw):
    R = yaw_to_rotation_matrix(yaw)
    t = np.array([x, y, z])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def find_mesh_in_sdf(sdf_path):
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    for mesh_elem in root.iter('mesh'):
        uri_elem = mesh_elem.find('uri')
        scale_elem = mesh_elem.find('scale')
        if uri_elem is not None:
            mesh_uri = uri_elem.text.strip()
            scale = parse_scale(scale_elem.text) if scale_elem is not None else (1, 1, 1)
            return mesh_uri, scale
    return None, None


def parse_world_file(world_path):
    tree = ET.parse(world_path)
    root = tree.getroot()
    models = []
    for include in root.iter('include'):
        uri_elem = include.find('uri')
        pose_elem = include.find('pose')
        name_elem = include.find('name')
        if uri_elem is not None:
            uri = uri_elem.text.strip()
            pose_str = pose_elem.text if pose_elem is not None else '0 0 0 0 0 0'
            name = name_elem.text if name_elem is not None else None
            x, y, z, roll, pitch, yaw = parse_pose(pose_str)
            models.append({
                'uri': uri,
                'name': name,
                'x': x, 'y': y, 'z': z,
                'roll': roll, 'pitch': pitch, 'yaw': yaw,
            })
    for model in root.iter('model'):
        model_name = model.get('name', '')
        pose_elem = model.find('pose')
        if pose_elem is not None and model_name != 'ground_plane':
            pose_str = pose_elem.text
            x, y, z, roll, pitch, yaw = parse_pose(pose_str)
            mesh_uri, scale = find_mesh_in_model_elem(model)
            if mesh_uri:
                models.append({
                    'uri': mesh_uri,
                    'name': model_name,
                    'x': x, 'y': y, 'z': z,
                    'roll': roll, 'pitch': pitch, 'yaw': yaw,
                    'scale': scale,
                })
    return models


def find_mesh_in_model_elem(model_elem):
    for mesh_elem in model_elem.iter('mesh'):
        uri_elem = mesh_elem.find('uri')
        scale_elem = mesh_elem.find('scale')
        if uri_elem is not None:
            mesh_uri = uri_elem.text.strip()
            scale = parse_scale(scale_elem.text) if scale_elem is not None else (1, 1, 1)
            return mesh_uri, scale
    return None, None


def resolve_model_path(uri, world_dir):
    if uri.startswith('file://'):
        return uri[7:]
    if os.path.isabs(uri):
        return uri
    return os.path.join(world_dir, uri)


def load_mesh_with_scale(mesh_path, scale):
    scene = trimesh.load(mesh_path, force='scene')
    scene.apply_scale(scale)
    all_points = []
    all_faces = []
    vertex_offset = 0
    for node_name in scene.graph.nodes_geometry:
        tf, geom_name = scene.graph.get(node_name)
        geom = scene.geometry[geom_name]
        if hasattr(geom, 'vertices') and len(geom.vertices) > 0 and hasattr(geom, 'faces') and len(geom.faces) > 0:
            transformed = geom.copy()
            transformed.apply_transform(tf)
            all_points.append(transformed.vertices)
            shifted_faces = transformed.faces + vertex_offset
            all_faces.append(shifted_faces)
            vertex_offset += len(transformed.vertices)
    if not all_points:
        raise RuntimeError("No valid geometry found in mesh")
    vertices = np.vstack(all_points)
    faces = np.vstack(all_faces)
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces)
    return mesh


def mesh_to_pcd(mesh, num_points, method='surface'):
    if method == 'surface':
        points, face_idx = trimesh.sample.sample_surface(mesh, num_points)
    elif method == 'uniform':
        points, face_idx = trimesh.sample.sample_surface_even(mesh, num_points)
    else:
        raise ValueError(f"Unknown method: {method}")
    return points, face_idx


def filter_by_normal(points, face_idx, mesh, min_nz=-1.0, max_nz=0.3,
                     keep_ceiling=False, wall_ratio=0.1):
    face_normals = mesh.face_normals
    sampled_normals = face_normals[face_idx]
    nz = sampled_normals[:, 2]

    if keep_ceiling:
        mask = (nz >= min_nz) & (nz <= max_nz) | (nz <= -0.7)
    else:
        mask = (nz >= min_nz) & (nz <= max_nz)

    non_wall = points[mask]
    wall_mask = ~mask
    wall_pts = points[wall_mask]
    if wall_ratio > 0 and len(wall_pts) > 0:
        n_keep = max(1, int(len(wall_pts) * wall_ratio))
        rng = np.random.default_rng(42)
        wall_sample = wall_pts[rng.choice(len(wall_pts), n_keep, replace=False)]
        return np.vstack([non_wall, wall_sample]), len(non_wall) + n_keep

    return non_wall, non_wall.shape[0]


def add_ground_plane(points, bounds, density=100):
    x_min, y_min, z_min = bounds[0]
    x_max, y_max, z_max = bounds[1]
    ground_z = min(z_min, 0.0)
    n = int((x_max - x_min) * (y_max - y_min) * density)
    gx = np.random.uniform(x_min, x_max, n)
    gy = np.random.uniform(y_min, y_max, n)
    gz = np.full(n, ground_z)
    ground = np.column_stack([gx, gy, gz])
    return np.vstack([points, ground])


def add_interior_ground(points, mesh, ground_z=0.0, resolution=0.1):
    from collections import Counter

    bounds = mesh.bounds
    x_min, y_min = bounds[0, 0], bounds[0, 1]
    x_max, y_max = bounds[1, 0], bounds[1, 1]

    x_range = np.arange(x_min, x_max + resolution, resolution)
    y_range = np.arange(y_min, y_max + resolution, resolution)

    ray_z = bounds[1, 2] + 1.0
    xx, yy = np.meshgrid(x_range, y_range)
    ray_origins = np.column_stack([xx.ravel(), yy.ravel(),
                                   np.full(xx.size, ray_z)])
    ray_directions = np.tile([0, 0, -1], (len(ray_origins), 1))

    print(f"  Ray casting: {len(ray_origins)} rays from z={ray_z:.1f}...")
    locations, index_ray, _ = mesh.ray.intersects_location(
        ray_origins=ray_origins,
        ray_directions=ray_directions,
    )

    intersection_count = Counter(index_ray)
    interior_rays = {idx for idx, cnt in intersection_count.items() if cnt % 2 == 1}

    interior_flat = np.zeros(len(ray_origins), dtype=bool)
    for idx in interior_rays:
        interior_flat[idx] = True

    interior_grid = interior_flat.reshape(len(y_range), len(x_range))
    interior_ys, interior_xs = np.where(interior_grid)

    gx = interior_xs * resolution + x_range[0] + resolution / 2
    gy = interior_ys * resolution + y_range[0] + resolution / 2
    gz = np.full(len(gx), ground_z)
    ground = np.column_stack([gx, gy, gz])

    print(f"  Interior ground: {len(ground)} points "
          f"({len(interior_rays)}/{len(ray_origins)} interior rays)")
    return np.vstack([points, ground])


def main():
    parser = argparse.ArgumentParser(description='Convert mesh to PCD')
    parser.add_argument('--world', '-w', type=str, default=None,
                        help='Gazebo .world file (auto-read pose + scale)')
    parser.add_argument('--input', '-i', type=str, default=None,
                        help='Input mesh file (.dae, .obj, .stl, .ply). '
                             'Auto-detected from --world if not specified.')
    parser.add_argument('--output', '-o', required=True, help='Output PCD file path')
    parser.add_argument('--pose', nargs=6, type=float, default=None,
                        help='Model pose x y z roll pitch yaw. '
                             'Auto-read from --world if not specified.')
    parser.add_argument('--scale', nargs=3, type=float, default=None,
                        help='Mesh scale factor (x y z). '
                             'Auto-read from model.sdf if --world is given.')
    parser.add_argument('--num_points', '-n', type=int, default=500000,
                        help='Number of sample points (default: 500000)')
    parser.add_argument('--voxel_size', '-v', type=float, default=0.05,
                        help='Voxel downsample size in meters (0 to skip, default: 0.05)')
    parser.add_argument('--add_ground', action='store_true',
                        help='Add ground plane points below the mesh')
    parser.add_argument('--add_interior_ground', action='store_true',
                        help='Add ground points inside building walls '
                             '(detects interior from wall footprint)')
    parser.add_argument('--interior_ground_res', type=float, default=0.05,
                        help='Interior ground grid resolution in meters (default: 0.05)')
    parser.add_argument('--ground_density', type=float, default=100,
                        help='Ground plane point density per m^2 (default: 100)')
    parser.add_argument('--method', choices=['surface', 'uniform'], default='surface',
                        help='Sampling method (default: surface)')
    parser.add_argument('--max_normal_z', type=float, default=0.3,
                        help='Max face normal Z to keep (0=horizontal only, '
                             '1=no filter). Removes walls/risers. (default: 0.3)')
    parser.add_argument('--wall_ratio', type=float, default=0.1,
                        help='Fraction of wall points to keep for structural '
                             'completeness (0=remove all, 1=keep all). '
                             '(default: 0.1)')
    parser.add_argument('--no_normal_filter', action='store_true',
                        help='Disable normal-based filtering (keep all surfaces)')
    parser.add_argument('--keep_ceiling', action='store_true',
                        help='Keep ceiling points (normals pointing down)')
    parser.add_argument('--visualize', action='store_true',
                        help='Visualize the result with Open3D')
    args = parser.parse_args()

    mesh_path = args.input
    pose = args.pose
    scale = args.scale

    if args.world:
        world_dir = os.path.dirname(os.path.abspath(args.world))
        print(f"Parsing world file: {args.world}")
        models = parse_world_file(args.world)

        building_model = None
        for m in models:
            if m['name'] and m['name'] != 'ground_plane':
                building_model = m
                break

        if building_model is None:
            print("  No model with pose found in world file, using defaults")
            pose = pose or [0, 0, 0, 0, 0, 0]
        else:
            print(f"  Found model: {building_model['name']}")
            print(f"  Pose: x={building_model['x']} y={building_model['y']} z={building_model['z']} "
                  f"roll={building_model['roll']} pitch={building_model['pitch']} yaw={building_model['yaw']}")
            if args.pose is None:
                pose = [building_model['x'], building_model['y'], building_model['z'],
                        building_model['roll'], building_model['pitch'], building_model['yaw']]

            uri = building_model['uri']
            model_sdf_path = resolve_model_path(uri, world_dir)
            if os.path.exists(model_sdf_path):
                print(f"  Reading scale from: {model_sdf_path}")
                mesh_uri, sdf_scale = find_mesh_in_sdf(model_sdf_path)
                if sdf_scale and args.scale is None:
                    scale = sdf_scale
                    print(f"  Scale from SDF: {scale}")
                if mesh_path is None and mesh_uri:
                    mesh_dir = os.path.dirname(model_sdf_path)
                    mesh_path = os.path.join(mesh_dir, mesh_uri)
                    print(f"  Mesh from SDF: {mesh_path}")

    if mesh_path is None:
        parser.error("No mesh file specified. Use --input or --world.")
    if pose is None:
        pose = [0, 0, 0, 0, 0, 0]
    if scale is None:
        scale = [1, 1, 1]

    print(f"\nConfiguration:")
    print(f"  Mesh:  {mesh_path}")
    print(f"  Pose:  x={pose[0]} y={pose[1]} z={pose[2]} "
          f"roll={pose[3]} pitch={pose[4]} yaw={pose[5]}")
    print(f"  Scale: {scale}")
    print(f"  Normal filter: {'disabled' if args.no_normal_filter else f'nz <= {args.max_normal_z}'}")

    print(f"\nLoading mesh: {mesh_path}")
    mesh = load_mesh_with_scale(mesh_path, scale)
    print(f"  Vertices: {len(mesh.vertices)}, Faces: {len(mesh.faces)}")
    print(f"  Bounds (before pose): {mesh.bounds}")

    print(f"Sampling {args.num_points} points ({args.method} method)...")
    points, face_idx = mesh_to_pcd(mesh, args.num_points, method=args.method)
    print(f"  Raw points: {len(points)}")

    if not args.no_normal_filter:
        kept, n_kept = filter_by_normal(
            points, face_idx, mesh,
            max_nz=args.max_normal_z,
            keep_ceiling=args.keep_ceiling,
            wall_ratio=args.wall_ratio,
        )
        print(f"  Normal filter (nz<={args.max_normal_z}, wall_ratio={args.wall_ratio}): "
              f"{len(points)} → {n_kept} ({100*n_kept/len(points):.1f}% kept)")
        points = kept

    x, y, z, roll, pitch, yaw = pose
    if abs(x) > 1e-6 or abs(y) > 1e-6 or abs(z) > 1e-6 or abs(yaw) > 1e-6:
        T = pose_to_transform(x, y, z, roll, pitch, yaw)
        print(f"Applying pose transform: x={x} y={y} z={z} yaw={yaw}")
        ones = np.ones((points.shape[0], 1))
        pts_h = np.hstack([points, ones])
        points = (T @ pts_h.T).T[:, :3]

    if args.add_ground:
        pts_min = points.min(axis=0)
        pts_max = points.max(axis=0)
        bounds = np.array([pts_min, pts_max])
        print(f"Adding ground plane (density={args.ground_density}/m^2)...")
        points = add_ground_plane(points, bounds, args.ground_density)
        print(f"  With ground: {len(points)}")

    if args.add_interior_ground:
        print(f"Adding interior ground (resolution={args.interior_ground_res}m)...")
        points = add_interior_ground(points, mesh, ground_z=0.0,
                                     resolution=args.interior_ground_res)
        print(f"  With interior ground: {len(points)}")

    if args.voxel_size > 0:
        print(f"Voxel downsampling (size={args.voxel_size}m)...")
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd = pcd.voxel_down_sample(args.voxel_size)
        points = np.asarray(pcd.points)
        print(f"  Downsampled: {len(points)}")

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points.astype(np.float32))

    print(f"Saving PCD: {args.output}")
    o3d.io.write_point_cloud(args.output, pcd)
    print(f"  Final points: {len(points)}")
    print(f"  Bounds: X[{points[:,0].min():.2f}, {points[:,0].max():.2f}] "
          f"Y[{points[:,1].min():.2f}, {points[:,1].max():.2f}] "
          f"Z[{points[:,2].min():.2f}, {points[:,2].max():.2f}]")

    if args.visualize:
        print("Visualizing...")
        o3d.visualization.draw_geometries([pcd], window_name="Mesh to PCD Result")


if __name__ == '__main__':
    main()
