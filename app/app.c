#include "gpio.h"

extern void SystemClock_Config();

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    while(1)
    {
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
        HAL_Delay(1000);
    }
}