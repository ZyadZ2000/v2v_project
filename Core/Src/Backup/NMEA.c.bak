/*
 *
 *      Author: GP Team
 */

#include "NMEA.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

int GMT = +200;

int inx = 0;
int hr = 0, min = 0, day = 0, mon = 0, yr = 0;
int daychange = 0;

/* Decodes the GGA Data
 GGAbuffer is the buffer which stores the GGA Data
 GGASTRUCT is the pointer to the GGA Structure (in the GPS Structure)
 Returns 0 on success
 */

int decodeGGA(char *GGAbuffer, GGASTRUCT *gga) {
	inx = 0;
	char buffer[12];
	int i = 0;
	while (GGAbuffer[inx] != ',')
		inx++;  // 1st ','
	inx++;
	while (GGAbuffer[inx] != ',')
		inx++;  // After time ','
	inx++;
	while (GGAbuffer[inx] != ',')
		inx++;  // after latitude ','
	inx++;
	while (GGAbuffer[inx] != ',')
		inx++;  // after NS ','
	inx++;
	while (GGAbuffer[inx] != ',')
		inx++;  // after longitude ','
	inx++;
	while (GGAbuffer[inx] != ',')
		inx++;  // after EW ','
	inx++;  // reached the character to identify the fix
	if ((GGAbuffer[inx] == '1') || (GGAbuffer[inx] == '2')
			|| (GGAbuffer[inx] == '6'))   // 0 indicates no fix yet
			{
		gga->isfixValid = 1;   // fix available
		inx = 0; // reset the index. We will start from the inx=0 and extract information now
	} else {
		gga->isfixValid = 0;   // If the fix is not available
		return 1;  // return error
	}
	while (GGAbuffer[inx] != ',')
		inx++;  // 1st ','

	/*********************** Get TIME ***************************/
//(Update the GMT Offset at the top of this file)
	inx++;   // reach the first number in time
	memset(buffer, '\0', 12);
	i = 0;
	while (GGAbuffer[inx] != ',')  // copy upto the we reach the after time ','
	{
		buffer[i] = GGAbuffer[inx];
		i++;
		inx++;
	}

	hr = (atoi(buffer) / 10000) + GMT / 100; // get the hours from the 6 digit number

	min = ((atoi(buffer) / 100) % 100) + GMT % 100; // get the minutes from the 6 digit number

	// adjust time.. This part still needs to be tested
	if (min > 59) {
		min = min - 60;
		hr++;
	}
	if (hr < 0) {
		hr = 24 + hr;
		daychange--;
	}
	if (hr >= 24) {
		hr = hr - 24;
		daychange++;
	}

	// Store the time in the GGA structure
	gga->tim.hour = hr;
	gga->tim.min = min;
	gga->tim.sec = atoi(buffer) % 100;

	/***************** Get LATITUDE  **********************/
	inx++;   // Reach the first number in the lattitude
	memset(buffer, '\0', 12);
	i = 0;
	while (GGAbuffer[inx] != ',') // copy upto the we reach the after lattitude ','
	{
		buffer[i] = GGAbuffer[inx];
		i++;
		inx++;
	}
	if (strlen(buffer) < 6)
		return 2;  // If the buffer length is not appropriate, return error
	int16_t num = (atoi(buffer)); // change the buffer to the number. It will only convert upto decimal
	int j = 0;
	while (buffer[j] != '.')
		j++;   // Figure out how many digits before the decimal
	j++;
	int declen = (strlen(buffer)) - j; // calculate the number of digit after decimal
	int dec = atoi((char*) buffer + j); // conver the decimal part a a separate number
	double lat = (num / 100.0) + (dec / pow(10, (declen + 2))); // 1234.56789 = 12.3456789
	gga->lcation.latitude = lat;  // save the lattitude data into the strucure
	inx++;
	gga->lcation.NS = GGAbuffer[inx];  // save the N/S into the structure

	/***********************  GET LONGITUDE **********************/
	inx++;  // ',' after NS character
	inx++;  // Reach the first number in the longitude
	memset(buffer, '\0', 12);
	i = 0;
	while (GGAbuffer[inx] != ',') // copy upto the we reach the after longitude ','
	{
		buffer[i] = GGAbuffer[inx];
		i++;
		inx++;
	}
	num = (atoi(buffer)); // change the buffer to the number. It will only convert upto decimal
	j = 0;
	while (buffer[j] != '.')
		j++;  // Figure out how many digits before the decimal
	j++;
	declen = (strlen(buffer)) - j; // calculate the number of digit after decimal
	dec = atoi((char*) buffer + j); // conver the decimal part a a separate number
	lat = (num / 100.0) + (dec / pow(10, (declen + 2))); // 1234.56789 = 12.3456789
	gga->lcation.longitude = lat;  // save the longitude data into the strucure
	inx++;
	gga->lcation.EW = GGAbuffer[inx];  // save the E/W into the structure

	/**************************************************/
	// skip positition fix
	inx++;   // ',' after E/W
	inx++;   // position fix
	inx++;   // ',' after position fix;

	// number of sattelites
	inx++;  // Reach the first number in the satellites
	memset(buffer, '\0', 12);
	i = 0;
	while (GGAbuffer[inx] != ',') // copy upto the ',' after number of satellites
	{
		buffer[i] = GGAbuffer[inx];
		i++;
		inx++;
	}
	gga->numofsat = atoi(buffer); // convert the buffer to number and save into the structure

	/***************** skip HDOP  *********************/
	inx++;
	while (GGAbuffer[inx] != ',')
		inx++;

	/*************** Altitude calculation ********************/
	inx++;
	memset(buffer, '\0', 12);
	i = 0;
	while (GGAbuffer[inx] != ',') {
		buffer[i] = GGAbuffer[inx];
		i++;
		inx++;
	}
	num = (atoi(buffer));
	j = 0;
	while (buffer[j] != '.')
		j++;
	j++;
	declen = (strlen(buffer)) - j;
	dec = atoi((char*) buffer + j);
	lat = (num) + (dec / pow(10, (declen)));
	gga->alt.altitude = lat;

	inx++;
	gga->alt.unit = GGAbuffer[inx];

	return 0;

}
/* to decode RMC String*/
#if 0
int decodeRMC (char *RMCbuffer, RMCSTRUCT *rmc)
{
	inx = 0;
	char buffer[12];
	int i = 0;
	while (RMCbuffer[inx] != ',') inx++;  // 1st ,
	inx++;
	while (RMCbuffer[inx] != ',') inx++;  // After time ,
	inx++;
	if (RMCbuffer[inx] == 'A')  // Here 'A' Indicates the data is valid, and 'V' indicates invalid data
	{
		rmc->isValid = 1;
	}
	else
	{
		rmc->isValid =0;
		return 1;
	}
	inx++;
	inx++;
	while (RMCbuffer[inx] != ',') inx++;  // after latitude,
	inx++;
	while (RMCbuffer[inx] != ',') inx++;  // after NS ,
	inx++;
	while (RMCbuffer[inx] != ',') inx++;  // after longitude ,
	inx++;
	while (RMCbuffer[inx] != ',') inx++;  // after EW ,

	// Get Speed
	inx++;
	i=0;
	memset(buffer, '\0', 12);
	while (RMCbuffer[inx] != ',')
	{
		buffer[i] = RMCbuffer[inx];
		i++;
		inx++;
	}

	if (strlen (buffer) > 0){          // if the speed have some data
		int16_t num = (atoi(buffer));  // convert the data into the number
		int j = 0;
		while (buffer[j] != '.') j++;   // same as above
		j++;
		int declen = (strlen(buffer))-j;
		int dec = atoi ((char *) buffer+j);
		float lat = num + (dec/pow(10, (declen)));
		rmc->speed = lat;
	}
	else rmc->speed = 0;

	// Get Course
	inx++;
	i=0;
	memset(buffer, '\0', 12);
	while (RMCbuffer[inx] != ',')
	{
		buffer[i] = RMCbuffer[inx];
		i++;
		inx++;
	}

	if (strlen (buffer) > 0){  // if the course have some data
		int16_t num = (atoi(buffer));   // convert the course data into the number
		int j = 0;
		while (buffer[j] != '.') j++;   // same as above
		j++;
		int declen = (strlen(buffer))-j;
		int dec = atoi ((char *) buffer+j);
		float lat = num + (dec/pow(10, (declen)));
		rmc->course = lat;
	}
	else
		{
			rmc->course = 0;
		}

	// Get Date
	inx++;
	i=0;
	memset(buffer, '\0', 12);
	while (RMCbuffer[inx] != ',')
	{
		buffer[i] = RMCbuffer[inx];
		i++;
		inx++;
	}

	// Date in the format 280222
	day = atoi(buffer)/10000;  // extract 28
	mon = (atoi(buffer)/100)%100;  // extract 02
	yr = atoi(buffer)%100;  // extract 22

	day = day+daychange;   // correction due to GMT shift

	// save the data into the structure
	rmc->date.Day = day;
	rmc->date.Mon = mon;
	rmc->date.Yr = yr;

	return 0;
}

#endif

double StrDeg_To_FloatDec(char Deg_cord[]) {
	char Deg_arr[15];
	char Min_arr[15];
	char Sec_arr[15];
	int i;
	char Deg_cord_2[1];
	if (Deg_cord[0] == '0') {
		for (i = 1; i < strlen(Deg_cord); i++) {
			Deg_cord_2[0] += Deg_cord[i];
		}
		Deg_cord = &Deg_cord_2[0];
	}
	int Cursor;

	int Min_Cursor = 0;
	int Sec_Cursor = 0;
	for (Cursor = 0; Cursor < 11; Cursor++) {
		if (Cursor < 2) {
			Deg_arr[Cursor] = Deg_cord[Cursor];
		} else if (Cursor < 4 && Cursor >= 2) {
			Min_arr[Min_Cursor] = Deg_cord[Cursor];
			Min_Cursor++;
		} else if (Cursor > 4) {
			Sec_arr[Sec_Cursor] = Deg_cord[Cursor];
			Sec_Cursor++;
			if (Cursor == 6) {
				Sec_arr[Sec_Cursor] = '.';
				Sec_Cursor++;
			}
		}
	}

	char *pend;
	double Deg = strtof(Deg_arr, &pend);
	double Min = strtof(Min_arr, &pend);
	double Sec = strtof(Sec_arr, &pend);
	double Dec = Deg + (Min / 60.00) + (Sec / 3600.00);

	return Dec;
}

/*msg should be array of 32 chars*/
void Build_Msg(char *msg, double lat, double longt, char NS, char EW,
		double direction) {

	char direction_string[8] = { 0 };
	char longtiude[11] = { 0 };
	char lattiude[11] = { 0 };
	//lat = lat * 100;
	//longt = longt *100;

	// from double to string
	snprintf(lattiude,11, "%.6f", lat);
	snprintf(longtiude,11, "%.6f", longt);
	snprintf(direction_string,8, "%.3f", direction);

	msg[0] = '#';
	msg[1] = lattiude[0];
	msg[2] = lattiude[1];
	msg[3] = lattiude[2];
	msg[4] = lattiude[3];
	msg[5] = lattiude[4];
	msg[6] = lattiude[5];
	msg[7] = lattiude[6];
	msg[8] = lattiude[7];
	msg[9] = lattiude[8];
	msg[10] = lattiude[9];

	msg[11] = NS;

	msg[12] = longtiude[0];
	msg[13] = longtiude[1];
	msg[14] = longtiude[2];
	msg[15] = longtiude[3];
	msg[16] = longtiude[4];
	msg[17] = longtiude[5];
	msg[18] = longtiude[6];
	msg[19] = longtiude[7];
	msg[20] = longtiude[8];
	msg[21] = longtiude[9];
	msg[22] = EW;
	msg[23] = direction_string[0];
	msg[24] = direction_string[1];
	msg[25] = direction_string[2];
	msg[26] = direction_string[3];
	msg[27] = direction_string[4];
	msg[28] = direction_string[5];
	msg[29] = direction_string[6];
	msg[30] = '#';
	msg[31] = '\r';
	msg[32] = '\n';
	msg[33] = '\0';

	/*3003.95734N3116.64210E# is the message
	 * first 10 chars is lattiude N
	 * then 10 chars for longtiude then E 
	 * And # specifies the end of message */

}

double Deg_To_Rad(double deg) {
	return deg * (M_PI / 180);
}

double Distance_Calc(char Lati1_Str[], char Long1_Str[], char Lati2_Str[],
		char Long2_Str[]) {

	// convert Long & Lati from degree to Decimal
	double Lati1 = StrDeg_To_FloatDec(Lati1_Str);
	double Long1 = StrDeg_To_FloatDec(Long1_Str);
	double Lati2 = StrDeg_To_FloatDec(Lati2_Str);
	double Long2 = StrDeg_To_FloatDec(Long2_Str);

	// Starting calculations
	double R = 6371000; // Radius of the earth in meter
	double Lati = Deg_To_Rad(Lati2 - Lati1);
	double Long = Deg_To_Rad(Long2 - Long1);
	double a = sin(Lati / 2) * sin(Lati / 2)
			+ cos(Deg_To_Rad(Lati1)) * cos(Deg_To_Rad(Lati2)) * sin(Long / 2)
					* sin(Long / 2);
	double c = 2 * atan2(sqrt(a), sqrt(1 - a));
	double d = R * c; // Distance in meter
	return d;
}
