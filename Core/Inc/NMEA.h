/*
 * 
 *      Author: GP Team
 */

#ifndef INC_NMEA_H_
#define INC_NMEA_H_


typedef struct {
	int hour;
	int min;
	int sec;
}TIME;

typedef struct {
	double latitude;
	char NS;
	double longitude;
	char EW;
}LOCATION;

typedef struct {
	double altitude;
	char unit;
}ALTITUDE;

typedef struct {
	int Day;
	int Mon;
	int Yr;
}DATE;

typedef struct {
	LOCATION lcation;
	TIME tim;
	int isfixValid;
	ALTITUDE alt;
	int numofsat;
}GGASTRUCT;

typedef struct {
	DATE date;
	float speed;
	float course;
	int isValid;
}RMCSTRUCT;

typedef struct {
	GGASTRUCT ggastruct;
	RMCSTRUCT rmcstruct;
}GPSSTRUCT;

int decodeGGA (char *GGAbuffer, GGASTRUCT *gga);
#if 0
int decodeRMC (char *RMCbuffer, RMCSTRUCT *rmc);
#endif
double StrDeg_To_FloatDec(char Deg_cord []);
void Build_Msg (char * msg , double lat , double longt,char NS,char EW, double direction);
double Deg_To_Rad(double deg) ;
double Distance_Calc(char Lati1_Str [], char Long1_Str [], char Lati2_Str [], char Long2_Str []);
#endif /* INC_NMEA_H_ */
