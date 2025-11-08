CXX = g++
CXXFLAGS = -std=c++20 -Wall -Wextra

TARGET_BIND = beavs_sim$(shell python3-config --extension-suffix)

LINKFLAGS = -shared
BUILDFLAGS = -fPIC -O3

LDFLAGS = -lboost_context -lboost_system $(shell python3-config --ldflags)
DEPFLAGS = -MMD -MP $(shell python3 -m pybind11 --includes)

SOURCES = $(wildcard *.cpp)
OBJECTS = $(SOURCES:.cpp=.o)

DEPS = $(OBJECTS:.o=.d)

.PHONY: build exp clean clean_exp

build: $(TARGET_BIND)

exp: ../$(TARGET_BIND)

# TODO: Use things like $@ and $^
../$(TARGET_BIND): $(TARGET_BIND)
	cp $(TARGET_BIND) ../$(TARGET_BIND)

$(TARGET_BIND): $(OBJECTS) board.o
	$(CXX) $(CXXFLAGS) $(LINKFLAGS) -o $@ $^ $(LDFLAGS)

%.o: %.cpp board.h
	$(CXX) $(CXXFLAGS) $(BUILDFLAGS) $(DEPFLAGS) -c $< -o $@

# TODO:Find a better way to do this because this is really stupid
board.h board.cpp: BEAVS5_Main/BEAVS5_Main.ino additions.txt
	python3 classify.py BEAVS5_Main/BEAVS5_Main.ino additions.txt board Board

-include $(DEPS)

clean:
	rm $(TARGET_BIND) $(OBJECTS) $(DEPS) board.cpp board.h

clean_exp: clean
	rm ../$(TARGET_BIND) ../Test/$(TARGET_BIND)

