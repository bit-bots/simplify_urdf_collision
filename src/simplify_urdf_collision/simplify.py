#!/usr/bin/env python3

import argparse
from urdf_handler import URDFHandler
from scipy.spatial.transform import Rotation
import os
import trimesh
import simplify_urdf_collision 
import transforms3d
import numpy as np
from urdf_parser_py.urdf import Box 
from color import COLORS

TMP_MESH_FILENAME = "/tmp/temp_mesh.off"

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("input_urdf")
    parser.add_argument("output_urdf")
    parser.add_argument("-r", "--ros", action="store_true", help="when enabled, ros package paths in urdfs are resolved")
    parser.add_argument("-s", "--select", action="store_true", help="enable interactive deselecting of certain collision models not to be transformed into bounding boxes")
    # todo add option exclude links
    parser.add_argument("-e", "--exclude", nargs="+", default=[], help="names of links to be excluded from collision model simplification")
    args = parser.parse_args()
    if args.ros:
        import resource_retriever
    urdf_handler = URDFHandler(args.input_urdf, args.exclude)
    filename_dict = urdf_handler.get_filenames(args.select)
    i = 1
    n = len(filename_dict)
    for link, collision_models in filename_dict.items():
        print(f"{link} ({i}/{n})")
        for c in collision_models:
            # resolve filename if, uses package:// prefix if ros is enabled
            resolved_filename = c.geometry.filename
            if args.ros:
                resolved_filename = resource_retriever.get_filename(resolved_filename, use_protocol=False)
            mesh = trimesh.load(resolved_filename)

            bb_tf = np.linalg.inv(np.matrix(mesh.apply_obb()))
            bb_bounds = mesh.bounding_box_oriented.bounds
            bb_size = [bb_bounds[1][0]*2, bb_bounds[1][1]*2, bb_bounds[1][2]*2]

            original_rotation = transforms3d.euler.euler2mat(c.origin.rotation[0], c.origin.rotation[1], c.origin.rotation[2], axes="sxyz")
            original_transformation_affine = transforms3d.affines.compose(T=c.origin.position, R=original_rotation, Z=[1,1,1])

            new_affine = np.matmul(original_transformation_affine, bb_tf)
            T,R,_,_ = transforms3d.affines.decompose44(new_affine)
            c.origin.position = T
            c.origin.rotation = list(transforms3d.euler.mat2euler(R, axes="sxyz"))
            b = Box(bb_size)
            c.geometry = b
        i +=1
    print(f"{COLORS.OKGREEN}Writing urdf to {args.output_urdf}{COLORS.ENDC}")
    urdf_handler.write_urdf(args.output_urdf)
