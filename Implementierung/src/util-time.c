// This file is part of papo-model
//
// papo-model is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// papo-model is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// Author: Julian Kunkel

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <util-time.h>

void print_current_time(){
    char buff[100];
    time_t now = time(0);
    strftime (buff, 100, "%Y-%m-%d %H:%M:%S", localtime (&now));
    printf("%s", buff);
}

#ifdef ESM
void start_timer(timer * t1) {
    *t1 = clock64();
}

double stop_timer(timer t1) {
    timer end;
    start_timer(& end);
    return (end - t1) / 1000.0 / 1000.0;
}

#else // POSIX COMPLAINT

void start_timer(timer * t1) {
    clock_gettime(CLOCK_MONOTONIC, t1);
}

static timer time_diff (struct timespec end, struct timespec start) {
    struct timespec diff;
    if (end.tv_nsec < start.tv_nsec) {
        diff.tv_sec = end.tv_sec - start.tv_sec - 1;
        diff.tv_nsec = 1000000000 + end.tv_nsec - start.tv_nsec;
    } else {
        diff.tv_sec = end.tv_sec - start.tv_sec;
        diff.tv_nsec = end.tv_nsec - start.tv_nsec;
    }
    return diff;
}

static double time_to_double (struct timespec t) {
    double d = (double)t.tv_nsec;
    d /= 1000000000.0;
    d += (double)t.tv_sec;
    return d;
}

double stop_timer(timer t1) {
    timer end;
    start_timer(& end);
    return time_to_double(time_diff(end, t1));
}

#endif
