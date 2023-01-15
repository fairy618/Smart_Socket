#include <stdio.h>
#include "BH1750.h"

void func(void)
{
    static int cnt = 0;
    printf("bh1750: %d\n", cnt++);
}
