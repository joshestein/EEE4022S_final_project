CC=g++ -std=c++11
IMAGE=/home/josh/Documents/UCT/Thesis/Datasets/robotcar-dataset/stereo/centre/1418381799701502.png 
SOURCES=selective_search.cpp graph.cpp
OBJECTS=$(SOURCES:.cpp=.o)
EXECS=$(SOURCES:.cpp=)
FLAGS=`pkg-config --libs --cflags opencv`

all: $(OBJECTS) $(EXECS) run_graph

clean:
	rm -rf $(OBJECTS) $(EXECS) 

$(OBJECTS): %.o: %.cpp
	$(CC) -c $< $(FLAGS) -o $@

$(EXECS): %: %.o 
	$(CC) $< $(FLAGS) -o $@

run_ss:
	./selective_search $(IMAGE) f

run_graph:
	./graph $(IMAGE) 0.5 100