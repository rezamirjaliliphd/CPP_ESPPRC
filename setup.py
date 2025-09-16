# setup.py
from setuptools import setup, Extension
from Cython.Build import cythonize
import numpy as np, os, sys

base_dir = os.path.dirname(__file__)

cpp_sources = [os.path.join(base_dir, s) for s in [
    "Edge.cpp", "Graph.cpp", "Label.cpp", "LabelManager.cpp", "Solution.cpp", "Utils.cpp"
]]

if sys.platform == "win32":
    extra_compile_args = ["/std:c++17", "/O2", "/DNOMINMAX", "/D_CRT_SECURE_NO_WARNINGS"]
    extra_link_args = []
else:
    extra_compile_args = ["-std=c++17", "-O3"]
    extra_link_args = []

ext_modules = [
    Extension(
        name="src.CPP_ESPPRC.graph_wrapper",          # <- put it inside the package
        sources=[os.path.join(base_dir, "graph_wrapper.pyx")] + cpp_sources,
        language="c++",
        include_dirs=[base_dir, np.get_include()],
        extra_compile_args=extra_compile_args,
        extra_link_args=extra_link_args,
    )
]

setup(
    name="ESPPRC_Graph",
    version="0.1.0",
    ext_modules=cythonize(ext_modules, language_level=3),
    zip_safe=False,
)
