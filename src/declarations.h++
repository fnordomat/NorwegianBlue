#pragma once

#include <Eigen/Core>
#include <gmpxx.h>

typedef Eigen::Matrix<mpz_class,Eigen::Dynamic,Eigen::Dynamic> Matrix;
typedef Eigen::Matrix<mpz_class,Eigen::Dynamic,1> Vector;
typedef Eigen::Matrix<mpz_class,1,Eigen::Dynamic> RowVector;

using integer = mpz_class;
