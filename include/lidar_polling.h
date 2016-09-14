#ifndef LIDAR_POLLING_H_INCLUDED
#define LIDAR_POLLING_H_INCLUDED

#include <atomic>

void LidarPolingThread( std::atomic<bool> *exitsignal );

#endif // LIDAR_POLLING_H_INCLUDED
