#ifndef BOUNDINGBOX_PYWRAPPER_H_
#define BOUNDINGBOX_PYWRAPPER_H_
#include <Python.h>
#include <boost/python.hpp>

class PyOptimalBoundingBox{
  public:
    PyOptimalBoundingBox();
    createOptimalBoundingBox(std::string pathToSTL);
  private:
}

#endif