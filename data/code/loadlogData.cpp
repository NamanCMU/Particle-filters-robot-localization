#include "loadlogData.h"
#include "bee-map.h"

// OPENCV Headers
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
	
using namespace cv;
using namespace std;

void loadlogdata(char* logfile, vector<logdata>& logfiledata)
{
	ifstream loaddata (logfile);
	string logline;
	logdata logdata_indv;

	if(loaddata.is_open())
	{
		while(getline(loaddata,logline)){
			istringstream iss(logline);
			char c;
			iss >> c;
			int j = 0;			
			switch(c){
				case 'L':
					logdata_indv.data_source = 1;
					iss >> logdata_indv.x >> logdata_indv.y >> logdata_indv.theta;
					iss >> logdata_indv.xl >> logdata_indv.yl >> logdata_indv.thetal;
					logdata_indv.readings = (float*) malloc(sizeof(float) * total_readings);
					while (j < total_readings){
						iss >> logdata_indv.readings[j];
						logdata_indv.readings[j] /= 10.0;
						j++;
					}
					iss >> logdata_indv.timestamp;
					break;
				case 'O':
					logdata_indv.data_source = 0;
					iss >> logdata_indv.x >> logdata_indv.y >> logdata_indv.theta;
					iss >> logdata_indv.timestamp;
					break;
				default:
					break;
			}
			logfiledata.push_back(logdata_indv);
		}
	}
	loaddata.close();	
}

#ifdef LOADLOGDATA
int main(int argc, char** argv)
{
	char logfilename[100];  // Name of the map file
	strncpy(logfilename,"robotdata1.log",100);
	vector<logdata> logfiledata;
	loadlogdata(logfilename,logfiledata);


	logdata indv;
	for (int i = 0;i < logfiledata.size();i++)
	{
		indv = logfiledata[i];
		cout << indv.data_source << endl;
	}
	return 0;
	
}
#endif