#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"

#include "BH1750.h"

void func(void)
{
    static int cnt = 0;
    printf("bh1750: %d\n", cnt++);
}
