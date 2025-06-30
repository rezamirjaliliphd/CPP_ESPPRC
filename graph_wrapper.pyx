# distutils: language = c++
# cython: language_level=3
# cython: boundscheck=False, wraparound=False

import numpy as np
cimport numpy as np
from libcpp.vector cimport vector
from libcpp.utility cimport pair
from libcpp.memory cimport shared_ptr
from cython.operator cimport dereference as deref

from libc.stdint cimport int64_t

cdef extern from "Graph.h":
    cdef cppclass Graph:
        Graph(int n, const vector[double]& r_max, int Phi_ID, const vector[double]& pi)
        void addEdge(int t, int h, double cost, const vector[double]& resources)
        void display()

cdef extern from "LabelManager.h":
    cdef cppclass LabelManager:
        LabelManager(Graph& g)
        void Run(Graph& g)
        void displaySolutions() const
        vector[pair[vector[int], double]] getSolutions() const




cdef class PyGraph:
    cdef Graph* thisptr

    def __cinit__(self, double[:,:,:] r, double[:] r_max, int Phi_ID, double[:] pi):
        cdef int n = r.shape[0]
        cdef int n_res = r.shape[2] - 1 
        cdef vector[double] rmax_cpp, Pi
        cdef Graph* gtr
        cdef int i_, j_, k  # use different names here to avoid collision
        
        rmax_cpp.clear()
        Pi.clear()
        for i_ in range(n_res):
            rmax_cpp.push_back(r_max[i_])
        for i_ in range(Phi_ID, n_res):
            Pi.push_back(pi[i_])
        
        gtr = new Graph(n, rmax_cpp,Phi_ID, Pi)
        self.thisptr = gtr
        cdef double cost
        cdef vector[double] res
        for i_ in range(n):
            for j_ in range(n):
                cost = r[i_, j_, 0]
                if i_!=j_:
                    res.clear()
                    for k in range(1, n_res + 1):
                        res.push_back(r[i_, j_, k])
                    self.thisptr.addEdge(i_, j_, cost, res)

    def display(self):
        self.thisptr.display()


cdef class PyLabelManager:
    cdef LabelManager* thisptr
    cdef PyGraph g  # hold reference to Python Graph

    def __cinit__(self, PyGraph g):
        self.g = g
        self.thisptr = new LabelManager(deref(g.thisptr))

    def run(self):
        self.thisptr.Run(deref(self.g.thisptr))

    def display(self):
        self.thisptr.displaySolutions()

    def get_solutions(self):
        cdef vector[pair[vector[int],double]] sols = self.thisptr.getSolutions()
        cdef int i,j
        cdef pair[vector[int],double] sol
        py_sols = []
        for i in range(sols.size()):
            sol = sols[i]
            path = [sol.first[j] for j in range(sol.first.size())]
            py_sols.append((path, sol.second))
        return py_sols

    def __dealloc__(self):
        del self.thisptr
