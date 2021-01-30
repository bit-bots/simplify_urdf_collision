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
            
            # load mesh and resave it as .off file
            mesh = trimesh.load(resolved_filename)
            off_string = trimesh.exchange.off.export_off(mesh, digits=10)
            off_file = open(TMP_MESH_FILENAME, "w+")
            off_file.write(off_string)
            off_file.close()

            
            bb_tf_and_size = simplify_urdf_collision.create_optimal_bounding_box(TMP_MESH_FILENAME)
            bb_tf_affine = np.reshape(bb_tf_and_size[0], (4,4)) 

            # todo eval rotation order
            original_rotation = transforms3d.euler.euler2mat(c.origin.rotation[0], c.origin.rotation[1], c.origin.rotation[2], axes="sxyz")
            originial_tranformation_affine = transforms3d.affines.compose(T=c.origin.position, R=original_rotation, Z=[1,1,1])

            new_affine = np.matmul(bb_tf_affine, originial_tranformation_affine)

            T,R,_,_ = transforms3d.affines.decompose44(new_affine)
            c.origin.position = T
            c.origin.rotation = list(transforms3d.euler.mat2euler(R, axes="sxyz"))
            b = Box(bb_tf_and_size[1])
            c.geometry = b
        i +=1
    
    urdf_handler.write_urdf(args.output_urdf)

    # collision box is first translated, then rotated, therefore translation can stay the same, 
    # rotation should be the rotation of the object by the rotation of the bounding box
