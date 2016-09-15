#ifndef GPIO_HANDLER_H_INCLUDED
#define GPIO_HANDLER_H_INCLUDED

#include <atomic>
#include "process_values_class.h"

void GpioHandlerThread( ProcessValues *processvalues,
						std::atomic<bool> *exitsignal,
						std::atomic<bool> *shutdownsignal );

#endif // GPIO_HANDLER_H_INCLUDED
