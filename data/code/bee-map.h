#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct
{
	int resolution;
	int size_x,size_y;
	int min_x,min_y,max_x,max_y;
	float offset_x, offset_y;
	float** prob;
} map_type;

int read_beesoft_map(char*, map_type*);
int new_hornetsoft_map(map_type*, int,int);
