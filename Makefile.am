AUTOMAKE_OPTIONS = foreign

bin_PROGRAMS = gtest benchmark ftencode ftdecode

SRC=obj/stringcodec.c++ obj/DFA.c++ obj/regexparser.c++

gtest_SOURCES = test/testgtest.cpp $(SRC)
benchmark_SOURCES = test/benchmark.cpp $(SRC)
ftencode_SOURCES =  tools/ftencode.c++ $(SRC)
ftdecode_SOURCES =  tools/ftdecode.c++ $(SRC)

gtest_LDFLAGS = -lgtest
benchmark_LDFLAGS = -lgtest
ftencode_LDFLAGS = -lboost_system -lboost_program_options
ftdecode_LDFLAGS = -lboost_system -lboost_program_options

LIBS = -lgmp -lgmpxx
LDADD =

all: gtest

LIB_FTXXCODE =  -lboost_system -lboost_program_options 

DEL_FILE = rm -f

SUFFIXES: .o .c++

obj/%.o: src/%.c++
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

