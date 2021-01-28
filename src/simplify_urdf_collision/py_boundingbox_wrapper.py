from simplify_urdf_collision.py_boundingbox import PyOptimalBoundingBox, create_optimal_bounding_box


class PyWalk(object):
    def __init__(self):
        self.optimal_bounding_box = PyOptimalBoundingBox()

    def create_optimal_bounding_box(self, path_to_stl):
        return self.optimal_bounding_box.create_optimal_bounding_box(path_to_stl)
