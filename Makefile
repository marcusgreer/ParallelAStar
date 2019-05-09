CC=clang
CXX=clang++
MPICC = mpicc

MPI=-DMPI
OMP=-Xpreprocessor -fopenmp -lomp


DEBUG=0
CFLAGS=-g -O3 -Wall -DDEBUG=$(DEBUG)
DDIR = ./data

CFILES = dijkstra.c a-star.c
HFILES = 
LDFLAGS= -lm



all: a-star a-star-omp a-star-serial dij-omp dij-serial

# A* versions: 
a-star: a-star-seq
	cp -p a-star-seq a-star

a-star-seq: a-star-seq.cpp
	$(CC)  -o a-star-seq a-star-seq.c $(LDFLAGS)

a-star-omp: a-star-omp.cpp
	$(CC) $(OMP) -o a-star-omp a-star-omp.c $(LDFLAGS)

# Dijkstra versions:
dij-serial: dijkstra_serial.cpp
	$(CXX) -o dij-serial dijkstra_serial.cpp


dij-omp: dijkstra_omp.cpp
	$(CXX) $(OMP) -o dij-omp dijkstra_omp.cpp



clean:
	rm *.o
	rm rm -f a-star a-star-seq a-star-omp a-star-serial dij-omp dij-serial
