#ifndef GPS_POLLING_H_INCLUDED
#define GPS_POLLING_H_INCLUDED

#include <atomic>
#include "process_values_class.h"

void GpsPollingThread( ProcessValues *processvalues,
					   std::atomic<bool> *exitsignal);

#endif // GPS_POLLING_H_INCLUDED
