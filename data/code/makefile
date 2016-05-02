CFLAGS = `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv`

mclocalization: load-log-data.o bee-map.o mclocalization.cpp mclocalization.h
	g++ -std=c++0x -D MCLOCALIZATION mclocalization.cpp load-log-data.o bee-map.o -o mclocalization $(CFLAGS) $(LIBS)
load-log-data.o: loadlogData.cpp loadlogData.h
	g++ -std=c++0x -c loadlogData.cpp -o load-log-data.o
load-log-data: loadlogData.cpp loadlogData.h
	g++ -std=c++0x -D LOADLOGDATA loadlogData.cpp -o load-log-data
bee-map.o: bee-map.c bee-map.h
	g++ -std=c++0x -c bee-map.c -o bee-map.o	
bee-map: bee-map.c bee-map.h
	g++ -std=c++0x -D BEEMAP bee-map.c -o bee-map $(CFLAGS) $(LIBS)
clean:
	rm -f *.o bee-map load-log-data mclocalization
