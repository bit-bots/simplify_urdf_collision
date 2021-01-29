#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>

#include <CGAL/optimal_bounding_box.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/IO/read_off_points.h>

#include <fstream>
#include <iostream>
#include <Python.h>
#include <boost/python.hpp>
#include <boost/python/list.hpp>


typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3                                     Point;
typedef CGAL::Surface_mesh<Point>                           Surface_mesh;



std::array<std::array<double,3>, 8> createOptimalBoundingBox(std::string pathToSTL)
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

boost::python::list optimalBoundingBoxWrapper(std::string filename)
{
  std::array<std::array<double,3>,8> bb_points = createOptimalBoundingBox(filename);
  boost::python::list return_value;
  for(std::array<double,3> point : bb_points)
  {
    boost::python::list py_point;
    for (double coordinate : point)
    {
      py_point.append(coordinate);
    }
    return_value.append(py_point);
  }
  return return_value;
}

BOOST_PYTHON_MODULE(simplify_urdf_collision)
{
    using namespace boost::python;

    def("create_optimal_bounding_box", &optimalBoundingBoxWrapper);
}


// main function if it is called by itself, takes filename as cli input
// prints resulting bounding box and saves it as .off file
int main(int argc, char** argv)
{
  if(argc != 2)
  {
      std::cerr << "Usage: " << argv[0] << " <input file.off> <output_file.off> (output optional, defaults to obb.off)" << std::endl;
      return EXIT_FAILURE;
  }
  auto a = createOptimalBoundingBox(argv[1]);
  // todo do something with result
  /*for(Point p : obb_points)
  {
    std::cout << p << std::endl;
  }*/
} 