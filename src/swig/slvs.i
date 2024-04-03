%module slvs

%begin %{
#define PY_SSIZE_T_CLEAN
#include "slvs_swig.hpp"
%}

%include "stdint.i"

%include "typemaps.i"
%apply double *OUTPUT { double *x, double *y, double *z };
%apply double *OUTPUT { double *qw, double *qx, double *qy, double *qz };

%ignore Slvs_System;
%ignore Slvs_Solve;
%rename("%(regex:/^Slvs_(.*)/\\l\\1/)s") "";
%rename("%(regex:/^SLVS_//)s") "";

%naturalvar;

%include "slvs.h"

%include "std_vector.i"
namespace std {
    %template(Vec_hConstraint) vector<Slvs_hConstraint>;
}

%include "exception.i"
%exception {
    try {
        $action
    } catch (std::out_of_range& e) {
        SWIG_exception(SWIG_IndexError, const_cast<char*>(e.what()));
    } catch (std::invalid_argument &e) {
        SWIG_exception(SWIG_ValueError, const_cast<char*>(e.what()));
    } catch (std::exception &e){
        SWIG_exception(SWIG_RuntimeError, const_cast<char*>(e.what()));
    } catch (...) {
        SWIG_exception(SWIG_RuntimeError, "Unknown exception");
    }
}

%immutable System::Failed;
%immutable System::Dof;

%include "slvs_swig.hpp"