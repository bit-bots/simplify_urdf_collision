#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>

#include <CGAL/optimal_bounding_box.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/IO/read_off_points.h>
#include <CGAL/Real_timer.h>

#include <fstream>
#include <iostream>
#include <Python.h>
#include <boost/python.hpp>


typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3                                     Point;
typedef CGAL::Surface_mesh<Point>                           Surface_mesh;

int main(int argc, char** argv)
{
  if(argc != 2)
  {
      std::cerr << "Usage: " << argv[0] << " <input file.off>" << std::endl;
      return EXIT_FAILURE;
  }
  auto a = createOptimalBoundingBox(argv[1]);
  // todo do something with result
  /*for(Point p : obb_points)
  {
    std::cout << p << std::endl;
  }*/
} 

std::array<std::array<double,3>, 8> createOptimalBoundingBox(std::string pathToSTL, std::array<double, 3> rpy)
{
  std::ifstream stream(argv[1]);
  std::list<Point> p;
  if (!stream || !CGAL::read_off_points(stream, std::back_inserter(p)))
  {
      std::cerr << "Error: cannot read file data" << std::endl;
      return EXIT_FAILURE;
  }

  // Compute the extreme points of the mesh, and then a tightly fitted oriented bounding box
  std::array<Point, 8> obb_points;
  CGAL::oriented_bounding_box(p, obb_points,
                              CGAL::parameters::use_convex_hull(true));
  
  // Make a mesh out of the oriented bounding box
  Surface_mesh obb_sm;
  CGAL::make_hexahedron(obb_points[0], obb_points[1], obb_points[2], obb_points[3],
                        obb_points[4], obb_points[5], obb_points[6], obb_points[7], obb_sm);
  std::ofstream("box.off") << obb_sm;
  std::array<std::array<double,3>,8> s;
  return s;
}

boost::python::list optimalBoundingBoxWrapper(std::string filename, std::python::list rpy)
{
  std::array<double, 3> rpy_array = boost::python::extract<std:array<double,3>>(rpy);
  auto l = createOptimalBoundingBox(filename, rpy_array);
}

BOOST_PYTHON_MODULE(py_boundingbox)
    {
        using namespace boost::python;
        using namespace simpliify_urdf_collision;

        def("create_optimal_bounding_box", &optimalBoundingBoxWrapper);
    }
