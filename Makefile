all: gtest

CXX=ccache g++
LD=g++

# DEBUG: # -ggdb -O0
CXXFLAGS=-std=gnu++1y -O3 \
 -Wall -Wextra -Wpedantic -Werror
INCPATH = -I/usr/local/ssl -I./src -I/usr/include/eigen3
LIB=-lgmp -lgmpxx

# -pg
# CXXFLAGS+=-fprofile-arcs -ftest-coverage
# LIB+=-lgcov

OBJ=obj/stringcodec.o obj/DFA.o obj/regexparser.o
LIB_FTXXCODE =  -lboost_system -lboost_program_options 

ftxxcode: $(OBJ)
	$(CXX) -c $(CXXFLAGS) $(INCPATH) tools/ftencode.c++ -oftencode.o
	$(CXX) -c $(CXXFLAGS) $(INCPATH) tools/ftdecode.c++ -oftdecode.o
	$(LD) $(LIB) $(LIB_FTXXCODE) ftencode.o $(OBJ) -o ftencode
	$(LD) $(LIB) $(LIB_FTXXCODE) ftdecode.o $(OBJ) -o ftdecode

benchmark: $(OBJ)
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -Itest test/benchmark.cpp -obenchmark.o
	$(LD) $(LIB) obj/*.o benchmark.o -o benchmark

gtest: $(OBJ) 
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -Itest test/testgtest.cpp -ogtest.o
	$(LD) $(LIB) obj/*.o gtest.o -o gtest -lgtest

DEL_FILE = rm -f

clean:
	-$(DEL_FILE) $(OBJ)
	-$(DEL_FILE) *.o obj/*.o
	-$(DEL_FILE) gtest ftencode ftdecode

.PHONY: clean

.SUFFIXES: .o .c++

obj/%.o: src/%.c++
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

