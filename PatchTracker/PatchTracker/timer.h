/*
 * Copyright (c) 2014. Jonathan Ventura. All rights reserved.
 * Licensed pursuant to the terms and conditions available for viewing at:
 * http://opensource.org/licenses/BSD-3-Clause
 *
 * File: timing.h
 * Author: Jonathan Ventura
 * Last Modified: 7.19.2014
 */

#ifndef TIMER_H
#define TIMER_H

#ifdef __APPLE__
#include "mach/mach_time.h"
#endif

struct Timer
{
    Timer() : nsamples( 0 ), total( 0 ) { mach_timebase_info(&info); }
#ifdef __APPLE__
    uint64_t getTime() { return mach_absolute_time(); }
    double convertToNanoseconds( uint64_t duration ) { return (duration * info.numer) / info.denom; }
#else
    uint64_t getTime() { return 0; }
    double convertToNanonseconds( uint64_t duration ) { return 0.; }
#endif
    void reset() { nsamples = 0; total = 0; }
    void start() { start_time = getTime(); }
    void stop() { total += getTime()-start_time; nsamples++; }
    double average() { return ((double)total)/nsamples; }
private:
    size_t nsamples;
    uint64_t total;
    mach_timebase_info_data_t info;
    uint64_t start_time;
};

#endif