#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>

#include <CGAL/optimal_bounding_box.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/IO/read_off_points.h>

#include <fstream>
#include <iostream>
#include <utility>
#include <limits>

#ifdef BUILD_PY

#include <Python.h>
#include <boost/python.hpp>
#include <boost/python/list.hpp>

#endif

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3                                     Point;
typedef Kernel::Aff_transformation_3                        Aff_transformation;
typedef CGAL::Surface_mesh<Point>                           Surface_mesh;


/**
 * returns the transformation matrix and the 
 */
std::pair<std::array<double, 16>, std::array<double,3>> createOptimalBoundingBox(std::string pathToSTL)
{
  std::ifstream stream(pathToSTL);
  std::list<Point> p;
  if (!stream || !CGAL::read_off_points(stream, std::back_inserter(p)))
  {
      std::cerr << "Error: cannot read file data" << std::endl;
      throw std::exception();
  }
  // compute rotated optimal bounding box
  std::array<Point, 8> obb_points;
  CGAL::oriented_bounding_box(p, obb_points, CGAL::parameters::use_convex_hull(true));
  
  /* untested */
  Aff_transformation tf;
  CGAL::oriented_bounding_box(p, tf, CGAL::parameters::use_convex_hull(true));
  
  std::array<double, 6> bb_min_max;
  for(int i = 0; i < 3; i++)
  {
    bb_min_max[i*2+0] = DBL_MIN;
    bb_min_max[i*2+1] = DBL_MAX;

  }
  // rotate points such that the bounding is axis aligned 
  // and get the minimum and maximum values for each axis
  std::array<Point, 8> obb_points_rotated;
  for(int i = 0; i < obb_points.size(); i++)
  {
    obb_points_rotated[i] = tf.transform(obb_points[i]);
    bb_min_max[0] = std::max(bb_min_max[0], obb_points_rotated[i].x());
    bb_min_max[1] = std::min(bb_min_max[1], obb_points_rotated[i].x());
    bb_min_max[2] = std::max(bb_min_max[2], obb_points_rotated[i].y());
    bb_min_max[3] = std::min(bb_min_max[3], obb_points_rotated[i].y());
    bb_min_max[4] = std::max(bb_min_max[4], obb_points_rotated[i].z());
    bb_min_max[5] = std::min(bb_min_max[5], obb_points_rotated[i].z());
  }

  std::array<double,3> bb_size;
  bb_size[0] =  bb_min_max[0] - bb_min_max[1]; 
  bb_size[1] =  bb_min_max[2] - bb_min_max[3];
  bb_size[2] =  bb_min_max[4] - bb_min_max[5];
  //std::cout << "size xyz: " << bb_size[0] <<  " " << bb_size[1] << " " << bb_size[1] << std::endl;

  double offset_x = bb_size[0]/2 - bb_min_max[0];
  double offset_y = bb_size[1]/2 - bb_min_max[2];
  double offset_z = bb_size[2]/2 - bb_min_max[4];
  //std::cout << "x_off: " << offset_x << " y_off: "  << offset_y << " z_off: " << offset_z << " " << std::endl;

  // apply offset to each point of the bounding box for when it is written to file
  Kernel::Vector_3 off(offset_x, offset_y, offset_z);
  for(int i = 0; i<8; i++)
  {
    obb_points_rotated[i] += off;
  }

  // Create mesh, both rotated and not rotated and write each to a file
  Surface_mesh obb_sm;
  CGAL::make_hexahedron(obb_points[0], obb_points[1], obb_points[2], obb_points[3],
                        obb_points[4], obb_points[5], obb_points[6], obb_points[7], obb_sm);
  std::ofstream("box.off") << obb_sm;
  Surface_mesh obb_sm_rotated;
  CGAL::make_hexahedron(obb_points_rotated[0], obb_points_rotated[1], obb_points_rotated[2], obb_points_rotated[3],
                        obb_points_rotated[4], obb_points_rotated[5], obb_points_rotated[6], obb_points_rotated[7], obb_sm_rotated);
  std::ofstream("box_rotated.off") << obb_sm_rotated;

  // put the values from the affine transformation matrix into a standard cpp array
  std::array<double,16> tf_array;
  for(int i = 0; i < 4; i++)
  {
    for(int j = 0; j < 4; j++)
    {
      tf_array[i*4+j] = tf.m(i,j);
    }
  }
  // translation of the bounding box is not included in the affine transformation matrix
  // therefore we add it here
  Kernel::Vector_3 off_rotated = tf.inverse().transform(off);
  tf_array[3] = -off_rotated.x();
  tf_array[7] = -off_rotated.y();
  tf_array[11] = -off_rotated.z();
  
  return std::make_pair(tf_array, bb_size);
}

#ifdef BUILD_PY
boost::python::list optimalBoundingBoxWrapper(std::string filename)
{
  std::pair<std::array<double, 16>, std::array<double,3>> bb_tf_and_size = createOptimalBoundingBox(filename);
  boost::python::list return_value;
  
  boost::python::list py_tf;
  for (double d : bb_tf_and_size.first)
  {
    py_tf.append(d);
  }
  boost::python::list py_bb_size;
  for (double limit : bb_tf_and_size.second)
  {
    py_bb_size.append(limit);
  }
  return_value.append(py_tf);
  return_value.append(py_bb_size);

  return return_value;
}
BOOST_PYTHON_MODULE(py_optimal_bounding_box)
{
    using namespace boost::python;

    def("create_optimal_bounding_box", &optimalBoundingBoxWrapper);
}
#endif

// main function if it is called by itself, takes filename as cli input
// prints resulting bounding box and saves it as .off file
int main(int argc, char** argv)
{
  if(argc != 2)
  {
      std::cerr << "Usage: " << argv[0] << " <input file.off>" << std::endl;
      return EXIT_FAILURE;
  }
  auto a = createOptimalBoundingBox(argv[1]);
  // TODO do something with result
} 