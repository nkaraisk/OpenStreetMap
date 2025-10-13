CXX = g++
CXXFLAGS = -Wall -Wextra -g -std=c++17 -fsanitize=address
LDFLAGS = -fsanitize=address

TARGET = hw5
OBJS = HW5.o Graph.o Vertex.o Edge.o Utilities.o tinyxml2.o

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS) $(LDFLAGS)

HW5.o: HW5.cpp Graph.hpp Vertex.hpp Edge.hpp Utilities.hpp
	$(CXX) $(CXXFLAGS) -c HW5.cpp

Graph.o: Graph.cpp Graph.hpp Vertex.hpp Edge.hpp Utilities.hpp
	$(CXX) $(CXXFLAGS) -c Graph.cpp

Vertex.o: Vertex.cpp Vertex.hpp Edge.hpp
	$(CXX) $(CXXFLAGS) -c Vertex.cpp

Edge.o: Edge.cpp Edge.hpp
	$(CXX) $(CXXFLAGS) -c Edge.cpp

Utilities.o: Utilities.cpp Utilities.hpp Vertex.hpp tinyxml2.h
	$(CXX) $(CXXFLAGS) -c Utilities.cpp

tinyxml2.o: tinyxml2.cpp tinyxml2.h
	$(CXX) $(CXXFLAGS) -c tinyxml2.cpp

clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean
