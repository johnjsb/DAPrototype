#ifndef GPIO_HANDLER_H_INCLUDED
#define GPIO_HANDLER_H_INCLUDED

#include <atomic>

void GpioHandlerThread( std::atomic<bool> *exitsignal,
						std::atomic<bool> *shutdownsignal );

#endif // GPIO_HANDLER_H_INCLUDED
