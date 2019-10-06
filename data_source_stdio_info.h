#ifndef DATA_SOURCE_STDIO_INFO_H
#define DATA_SOURCE_STDIO_INFO_H

#include <stddef.h>
#include <stdint.h>

#include "data_source.h"

//Prints out packet size and count
class data_source_stdio_info: public data_source
	{
	public:
	void write( const uint8_t * data, size_t bytes );

	private:
	int num_packets = 0;
	};

#endif
