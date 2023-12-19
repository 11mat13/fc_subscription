#include "dji_typedef.h"


#ifndef DATA_STRUCTURE_H
#define DATA_STRUCTURE_H

struct payloadData {
	struct date {
		int day;
		int month;
		int year;
		int hour;
		int minute;
		int second;
	}date;

	struct coordinates{
		dji_f64_t latitude;  /*!< Latitude, unit: deg. */
		dji_f64_t longitude; /*!< Longitude, unit: deg. */
		dji_f32_t altitude;      /*!< Height above mean sea level, unit: m. */
	}coordinates;
	
	unsigned int GPSdataAge;
	
	struct satelliteNumber {
		uint32_t gpsSatelliteNumberUsed; /*!< Number of GPS satellites used for fixing position. */
		uint32_t glonassSatelliteNumberUsed; /*!< Number of GLONASS satellites used for fixing position. */
		uint16_t totalSatelliteNumberUsed; /*!< Total number of satellites used for fixing position. */
	}satelliteNum;

	struct thermocoupleData {
		double temperature;
		double referenceTemperature;
	}thermocouple;

	struct BME280 {
		double temperature;
		double humidity;
		double pressure;
	}bme;

	struct PMvalues {
		double PM1;
		double PM2_5;
		double PM10;
	}pm;

	struct FRvalues {
		double fr03;
		double fr05;
		double fr10;
		double fr25;
		double fr50;
		double fr100;
	}fr;

	int batteryLevel;

};

#endif