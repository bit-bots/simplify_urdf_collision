
PyOptimalBoundingBox::PyOptimalBoundingBox(){
  // constructor
}

PyOptimalBoundingBox::createOptimalBoundingBox(std::string pathToSTL){

}

BOOST_PYTHON_MODULE(py_boundingbox)
    {
        using namespace boost::python;
        using namespace simpliify_urdf_collision;

        class_<PyWalkWrapper>("PyOptimalBoundingBox", init<std::string>())
        .def("create_optimal_bounding_box", &PyWalkWrapper::createOptimalBoundingBox);
    }