from setuptools import setup, Extension
from Cython.Build import cythonize
import numpy as np
import os
import sys
base_dir = os.path.dirname(__file__)

cpp_sources = [
    "Edge.cpp",
    "Graph.cpp",
    "Label.cpp",
    "LabelManager.cpp",
    "Solution.cpp",
    "Utils.cpp",
]

cpp_sources = [os.path.join(base_dir, src) for src in cpp_sources]

ext_modules = [
    Extension(
        name="graph_wrapper",
        sources=[os.path.join(base_dir, "graph_wrapper.pyx")] + cpp_sources,
        language="c++",
        include_dirs=[
            base_dir,
            np.get_include()
        ],
        extra_compile_args=["/std:c++17"] if sys.platform == "win32" else ["-std=c++17"]

    )
]

setup(
    name="ESPPRC_Graph",
    ext_modules=cythonize(ext_modules, language_level=3),
)
