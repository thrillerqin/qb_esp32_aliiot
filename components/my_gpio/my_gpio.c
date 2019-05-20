#include <stdio.h>
#include "my_gpio.h"

void my_gpio_test(void)
{
#if(CONFIG_MY_GPIO_ENABLE == y)
printf("my_gpio is configed\n");
#else
printf("not config");
#endif
}