#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "nmea_parser.h"

void get_position_from_nmea_parser(void);
int read_nmea_data(gps_t *pNMEAReading);

#ifdef __cplusplus
}
#endif