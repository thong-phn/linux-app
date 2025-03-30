#include "main_functions.h"

#define NUM_LOOPS 1
 
int main(int argc, char *argv[])
{
    setup();
    for (int i = 0; i < NUM_LOOPS; i++) {
        loop();
    }
    return 0;
}