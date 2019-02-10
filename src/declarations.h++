#pragma once

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <gmpxx.h>

#define DEBUG_STDOUT
// #define BOOST_SPIRIT_DEBUG
// #define DOT_DEBUG

typedef Eigen::Matrix<mpz_class,Eigen::Dynamic,Eigen::Dynamic> Matrix;
typedef Eigen::Matrix<mpz_class,Eigen::Dynamic,1> Vector;
typedef Eigen::Matrix<mpz_class,1,Eigen::Dynamic> RowVector;

typedef Eigen::SparseMatrix<mpz_class, Eigen::ColMajor> SMatrix;

using integer = mpz_class;

#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_BOLD          "\x1b[1m"
#define ANSI_RESET         "\x1b[0m"
