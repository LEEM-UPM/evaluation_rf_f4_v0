#include <stdbool.h>

#include "spi.h"
#include "leds.h"
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

extern void SystemClock_Config(void);
static bool spi_sanity_check(void);
static bool buffer_sanity_check(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();

    sx126x_hal_reset(&sx1262_context);

    if (!spi_sanity_check())
    {
        while (1)
        {
            HAL_GPIO_TogglePin(RED_LED_PORT, RED_LED_PIN);
            HAL_Delay(500);
        }
    }

    while (1)
    {
        HAL_Delay(1000);

        spi_sanity_check();

        if (buffer_sanity_check())
        {
            HAL_GPIO_WritePin(BLUE_LED_PORT, BLUE_LED_PIN, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, GPIO_PIN_SET);
        }

        HAL_Delay(1000);

        HAL_GPIO_WritePin(GREEN_LED_PORT, GREEN_LED_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BLUE_LED_PORT, BLUE_LED_PIN, GPIO_PIN_RESET);
    }
}

static bool spi_sanity_check(void)
{
    sx126x_chip_status_t radio_status;
    if (sx126x_get_status(&sx1262_context, &radio_status) != SX126X_STATUS_OK)
    {
        return false;
    }
    uint8_t chip_status = radio_status.chip_mode;

    switch (chip_status)
    {
    case SX126X_CHIP_MODE_STBY_RC:
        HAL_GPIO_WritePin(GREEN_LED_PORT, GREEN_LED_PIN, GPIO_PIN_SET);
        break;

    case SX126X_CHIP_MODE_STBY_XOSC:
        HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, GPIO_PIN_SET);
        break;

    case SX126X_CHIP_MODE_FS:
        HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, GPIO_PIN_SET);
        break;

    case SX126X_CHIP_MODE_RX:
        HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, GPIO_PIN_SET);
        break;

    case SX126X_CHIP_MODE_TX:
        HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, GPIO_PIN_SET);
        break;
        
    default:
        return false;
    }

    return true;
}

static bool buffer_sanity_check(void)
{
    uint8_t testPattern[4] = {0x55, 0xAA, 0x33, 0xCC};
    uint8_t readBuffer[4] = {0};
    uint8_t offset = 0;

    if (sx126x_write_buffer(&sx1262_context, offset, testPattern, 4) != SX126X_STATUS_OK)
    {
        return false;
    }

    if (sx126x_read_buffer(&sx1262_context, offset, readBuffer, 4) != SX126X_STATUS_OK)
    {
        return false;
    }

    for (uint8_t i = 0; i < 4; i++)
    {
        if (testPattern[i] != readBuffer[i])
        {
            HAL_GPIO_WritePin(ORANGE_LED_PORT, ORANGE_LED_PIN, GPIO_PIN_SET);
            return false;
        }
    }

    return true;
}