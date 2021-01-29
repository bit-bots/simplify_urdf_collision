#include "boundingbox_pywrapper.h"

PyOptimalBoundingBox::PyOptimalBoundingBox(){
  // constructor
}

void PyOptimalBoundingBox::createOptimalBoundingBox(std::string pathToSTL){

}

BOOST_PYTHON_MODULE(py_boundingbox)
    {
        using namespace boost::python;
        using namespace simpliify_urdf_collision;

        class_<PyOptimalBoundingBox>("PyOptimalBoundingBox", init<std::string>());
        .def("create_optimal_bounding_box", &PyWalkWrapper::createOptimalBoundingBox);
    }
