#pragma once

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <gmpxx.h>

// #define DEBUG_STDOUT
// #define BOOST_SPIRIT_DEBUG
// #define DOT_DEBUG

typedef Eigen::Matrix<mpz_class,Eigen::Dynamic,Eigen::Dynamic> Matrix;
typedef Eigen::Matrix<mpz_class,Eigen::Dynamic,1> Vector;
typedef Eigen::Matrix<mpz_class,1,Eigen::Dynamic> RowVector;

typedef Eigen::SparseMatrix<mpz_class, Eigen::ColMajor> SMatrix;

using integer = mpz_class;
