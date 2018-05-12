AC_INIT([NorwegianBlue], [0.0.1-alpha], [fnordomat])

AM_INIT_AUTOMAKE([subdir-objects])

CPPFLAGS="-I/usr/include/eigen3 -Isrc ${CPPFLAGS}"
CXXFLAGS="-std=gnu++1y -Wall -Wextra -Wpedantic -Wno-int-in-bool-context -Werror -I/usr/include/eigen3 -Isrc ${CXXFLAGS}"

AC_PROG_CXX(["ccache g++" g++ clang++])
AC_LANG([C++])

AC_HEADER_STDC
AC_TYPE_SIZE_T

AC_CHECK_HEADER([gmpxx.h],
                [AC_DEFINE(HAVE_GMPXX_H, 1, [Presence of gmpxx headers (required)] )],
                [AC_MSG_ERROR([gmpxx.h not found])])
AC_CHECK_HEADER([Eigen/Core],
                [AC_DEFINE(HAVE_EIGEN_CORE_H, 1, [Presence of eigen3 headers (required)] )],
                [AC_MSG_ERROR([Eigen/Core not found])])
AC_LIB_LINKFLAGS([gmp])
AC_LIB_LINKFLAGS([gmpxx])

AC_CONFIG_HEADERS([config.h])

AC_CONFIG_FILES([Makefile])

AC_OUTPUT
