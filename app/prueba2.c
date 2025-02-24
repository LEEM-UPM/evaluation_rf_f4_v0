#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"
#include "leds.h"
#include "spi.h"
#include "sx126x.h"
#include "sx126x_hal_context.h"
#include "sx126x_hal.h"

#define SX1262_SPI_HANDLE      &hspi1

#define SX1262_NSS_GPIO_PORT   GPIOD
#define SX1262_NSS_GPIO_PIN    GPIO_PIN_0   

#define SX1262_RESET_GPIO_PORT GPIOB
#define SX1262_RESET_GPIO_PIN  GPIO_PIN_0

#define SX1262_IRQ_GPIO_PORT   GPIOB
#define SX1262_IRQ_GPIO_PIN    GPIO_PIN_1

#define SX1262_BUSY_GPIO_PORT  GPIOC
#define SX1262_BUSY_GPIO_PIN   GPIO_PIN_0

sx126x_hal_context_t sx1262_context =
{
    .spi = SX1262_SPI_HANDLE,
    .nss =
    {
        .port = SX1262_NSS_GPIO_PORT,
        .pin = SX1262_NSS_GPIO_PIN,
    },
    .reset =
    {
        .port = SX1262_RESET_GPIO_PORT,
        .pin = SX1262_RESET_GPIO_PIN,
    },
    .irq =
    {
        .port = SX1262_IRQ_GPIO_PORT,
        .pin = SX1262_IRQ_GPIO_PIN,
    },
    .busy =
    {
        .port = SX1262_BUSY_GPIO_PORT,
        .pin = SX1262_BUSY_GPIO_PIN,
    },
};

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();

    sx126x_chip_status_t sx1262_status;

    sx126x_hal_reset(&sx1262_context);

    while (1)
    {
        sx126x_get_status(&sx1262_context, &sx1262_status);

        if (sx1262_status.chip_mode == SX126X_CHIP_MODE_STBY_RC)
        {
            HAL_GPIO_WritePin(GREEN_LED_PORT, GREEN_LED_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(ORANGE_LED_PORT, ORANGE_LED_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, GPIO_PIN_RESET);
        }
        else if (sx1262_status.chip_mode == SX126X_CHIP_MODE_STBY_XOSC)
        {
            HAL_GPIO_WritePin(ORANGE_LED_PORT, ORANGE_LED_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GREEN_LED_PORT, GREEN_LED_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, GPIO_PIN_RESET);
        }
        else
        {
            HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(ORANGE_LED_PORT, ORANGE_LED_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GREEN_LED_PORT, GREEN_LED_PIN, GPIO_PIN_RESET);
        }

        HAL_Delay(500);
    }
}