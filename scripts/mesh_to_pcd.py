#!/usr/bin/env python3
"""
Convert Gazebo SDF/DAE mesh to PCD point cloud for PCT planner.

Automatically reads pose offset from .world file and scale from model.sdf.

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
    ns = {'sdf': 'http://www.collada.org/2005/11/COLLADASchema'}
    for mesh_elem in root.iter('mesh'):
        uri_elem = mesh_elem.find('uri')
        scale_elem = mesh_elem.find('scale')
        if uri_elem is not None:
            mesh_uri = uri_elem.text.strip()
            scale = parse_scale(scale_elem.text) if scale_elem is not None else (1, 1, 1)
            return mesh_uri, scale
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
        points = trimesh.sample.sample_surface_even(mesh, num_points)[0]
    else:
        raise ValueError(f"Unknown method: {method}")
    return points


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
    parser.add_argument('--ground_density', type=float, default=100,
                        help='Ground plane point density per m^2 (default: 100)')
    parser.add_argument('--method', choices=['surface', 'uniform'], default='surface',
                        help='Sampling method (default: surface)')
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
            pose = [building_model['x'], building_model['y'], building_model['z'],
                    building_model['roll'], building_model['pitch'], building_model['yaw']]

            uri = building_model['uri']
            model_sdf_path = resolve_model_path(uri, world_dir)
            if os.path.exists(model_sdf_path):
                print(f"  Reading scale from: {model_sdf_path}")
                mesh_uri, sdf_scale = find_mesh_in_sdf(model_sdf_path)
                if sdf_scale:
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

    print(f"\nLoading mesh: {mesh_path}")
    mesh = load_mesh_with_scale(mesh_path, scale)
    print(f"  Vertices: {len(mesh.vertices)}, Faces: {len(mesh.faces)}")
    print(f"  Bounds (before pose): {mesh.bounds}")

    print(f"Sampling {args.num_points} points ({args.method} method)...")
    points = mesh_to_pcd(mesh, args.num_points, method=args.method)
    print(f"  Raw points: {len(points)}")

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
