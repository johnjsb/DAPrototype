#ifndef LIDAR_POLLING_H_INCLUDED
#define LIDAR_POLLING_H_INCLUDED

#include <atomic>
#include "process_values_class.h"

void LidarPolingThread( ProcessValues *processvalues,
						std::atomic<bool> *exitsignal );

#endif // LIDAR_POLLING_H_INCLUDED
