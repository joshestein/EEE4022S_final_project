CC=g++ -std=c++11
IMAGE=/home/josh/Documents/UCT/Thesis/Datasets/2014-05-14-13-53-47/stereo/centre/1400075815389497.png
# IMAGE=/home/josh/Documents/UCT/Thesis/Code/robotcar/image_points.jpg
SOURCES=selective_search.cpp graph.cpp superpixels.cpp fld.cpp
OBJECTS=$(SOURCES:.cpp=.o)
EXECS=$(SOURCES:.cpp=)
FLAGS=`pkg-config --libs --cflags opencv`

all: $(OBJECTS) $(EXECS)

clean:
	rm -rf $(OBJECTS) $(EXECS)

$(OBJECTS): %.o: %.cpp
	$(CC) -c $< $(FLAGS) -o $@

$(EXECS): %: %.o
	$(CC) $< $(FLAGS) -o $@

run_ss:
	./selective_search $(IMAGE) f 0

run_graph:
	./graph $(IMAGE) 0.5 100

run_superpixels:
	./superpixels $(IMAGE) 0

run_fld:
	./fld $(IMAGE)
