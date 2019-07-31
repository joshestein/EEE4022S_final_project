CC=g++ -std=c++11
IMAGE=/home/josh/Documents/UCT/Thesis/Datasets/robotcar-dataset/stereo/centre/1418381799701502.png 

all: clean build run

clean:
	rm -rf selective_search

build:
	$(CC) selective_search.cpp `pkg-config --libs --cflags opencv` -o selective_search

run:
	./selective_search $(IMAGE) q
