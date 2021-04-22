# simplify_urdf_collision

This git provides a script to simplify URDF collision models with optimal bounding boxes.

Installation
------------

you can install this as a catikin package or just run the script.

Python Dependencies (install using pip):
  - transforms3d
  - trimesh
  - numpy

You need to install urdf-parser-py manually since the pip package is not maintained:

```
git clone git@github.com:ros/urdf_parser_py.git
cd urdf_parser_py
pip install .
```
depending on your pip configuration you may need to use the `--user` option.



Run and usage
-------------

Run the script with -h to get the usage options.

```python src/simplify_urdf_collision/simplify.py -h```
