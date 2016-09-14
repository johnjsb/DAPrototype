#ifndef GPS_POLLING_H_INCLUDED
#define GPS_POLLING_H_INCLUDED

#include <atomic>

void GpsPollingThread( std::atomic<bool> *exitsignal);

#endif // GPS_POLLING_H_INCLUDED
