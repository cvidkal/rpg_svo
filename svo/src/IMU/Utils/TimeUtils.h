#pragma once

#include <chrono>
using namespace std::chrono;

#define TimestampToSecond(t)		((t)*1e-9)
#define TimestampToLongLong(t)		((long long)((t)*1e9))

//-----------------------------------------------------------------------------
// use c++ chrono
// unit: ns
inline long long getCppTimestamp()
{
	auto timestamp = duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();
	return timestamp;
}


//-----------------------------------------------------------------------------
// use c++ chrono
// unit: us
inline long long getCppTimestamp_us()
{
	auto timestamp = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
	return timestamp;
}

