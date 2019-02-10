AUTOMAKE_OPTIONS = foreign

bin_PROGRAMS = gtest benchmark ftencode ftdecode learndfa

SRC=obj/stringcodec.c++ obj/DFA.c++ obj/regexparser.c++ obj/aux.c++

gtest_SOURCES = test/testgtest.cpp $(SRC) obj/Markov.c++
benchmark_SOURCES = test/benchmark.cpp $(SRC)
ftencode_SOURCES =  tools/ftencode.c++ $(SRC)
ftdecode_SOURCES =  tools/ftdecode.c++ $(SRC)
learndfa_SOURCES =  tools/learndfa.c++ $(SRC) obj/Learner.c++ obj/Markov.c++ obj/MDL.c++

gtest_LDFLAGS = -lgtest -ljsoncpp 
benchmark_LDFLAGS = -lgtest -ljsoncpp
ftencode_LDFLAGS = -lboost_system -lboost_program_options -ljsoncpp
ftdecode_LDFLAGS = -lboost_system -lboost_program_options -ljsoncpp
learndfa_LDFLAGS = -lboost_system -lboost_program_options -ljsoncpp

LIBS = -lgmp -lgmpxx
LDADD =

all: gtest

DEL_FILE = rm -f

obj/%.o: src/%.c++
	$(CXX) -c $(CXXFLAGS) $(CCFLAGS) $(INCPATH) -o "$@" "$<"

