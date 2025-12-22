#include "log.hpp"
#include <cstdarg>
#include <cstdio>

void log(const char* tag, const char* format, ...) { 
    va_list args; 
    va_start(args, format); 
    printf("[%s] ", tag); 
    vprintf(format, args); 
    printf("\n"); 
    va_end(args); 
}

void logw(const char* tag, const char* format, ...) { 
    va_list args; 
    va_start(args, format); 
    printf("[%s] WARNING: ", tag); 
    vprintf(format, args); 
    printf("\n"); 
    va_end(args); 
}