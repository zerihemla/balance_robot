
#include "logInterface.h"

#include <stdio.h>

#include "rtosUtility.h"

void log_printOutput(char* output)
{
    uint32_t curTime = taskGetTimeMs();

    printf("(%u) %s\n", curTime, output);
}