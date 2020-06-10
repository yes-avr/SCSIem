/*
 * logging.h
 */

#ifndef	_LOGGING_H_
#define	_LOGGING_H_

// debug_level > 2 : availble all_messages appear on serial console
// debug_level > 1 : availble informative messages on serial console
// debug_level > 0 : availble warning messages on serial console
// debug_level ==0 : dont print any messages on serial console

#ifndef debug_level
#  define	debug_level 	(0)
#endif

#if debug_level > 0
#  define	debugout    	Serial1
#  define	debugin     	Serial1
#else
#  define	debugout    	Serial1 // It hangs. til the USB serial to be connected,
#  define	debugin     	Serial1 // If the USB is not connected. cos. assigned the USART1 now.
#endif



#  define	log_err(...)	debugout.print(__VA_ARGS__)
#  define	logln_err(...)	debugout.println(__VA_ARGS__)

#if debug_level > 0
#  define	log_warn(...)	debugout.print(__VA_ARGS__)
#  define	logln_warn(...)	debugout.println(__VA_ARGS__)
#else
#  define	log_warn(...)
#  define	logln_warn(...)
#endif

#if debug_level > 1
#  define	log_info(...)	debugout.print(__VA_ARGS__)
#  define	logln_info(...)	debugout.println(__VA_ARGS__)
#else
#  define	log_info(...)
#  define	logln_info(...)
#endif

#if debug_level > 2
#  define	log_debug(...)  	debugout.print(__VA_ARGS__)
#  define	logln_debug(...)	debugout.println(__VA_ARGS__)
#else
#  define	log_debug(...)
#  define	logln_debug(...)
#endif

#endif	/* _LOGGING_H_ */

