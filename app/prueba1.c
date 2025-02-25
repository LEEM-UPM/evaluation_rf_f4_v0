#include "stm32f4xx_hal.h"
#include "leds.h"
#include "stm32f4xx_hal_spi.h"
#include "spi.h"

#include <stdbool.h>

#define SX1262_SPI_HANDLE      &hspi1

#define SX1262_NSS_GPIO_PORT   GPIOD
#define SX1262_NSS_GPIO_PIN    GPIO_PIN_0   

#define SX1262_RESET_GPIO_PORT GPIOB
#define SX1262_RESET_GPIO_PIN  GPIO_PIN_0

#define SX1262_IRQ_GPIO_PORT   GPIOB
#define SX1262_IRQ_GPIO_PIN    GPIO_PIN_1

#define SX1262_BUSY_GPIO_PORT  GPIOC
#define SX1262_BUSY_GPIO_PIN   GPIO_PIN_0

#define SX1262_CMD_GET_STATUS 0xC0
#define SX1262_CMD_WRITE_BUFFER 0x0E
#define SX1262_CMD_READ_BUFFER 0x1E

extern void SystemClock_Config(void);
static bool spi_sanity_check(void);
static uint8_t sx1262_get_status(void);
static bool buffer_sanity_check(void);
static bool sx1262_write_buffer(uint8_t offset, const uint8_t *data, uint8_t length);
static bool sx1262_read_buffer(uint8_t offset, uint8_t *data, uint8_t length);
static void sx1262_wait_on_busy(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();

    HAL_GPIO_WritePin(SX1262_RESET_GPIO_PORT, SX1262_RESET_GPIO_PIN, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(SX1262_RESET_GPIO_PORT, SX1262_RESET_GPIO_PIN, GPIO_PIN_SET);

    sx1262_wait_on_busy();

    bool success = spi_sanity_check();
    if (!success)
    {
        while (1)
        {
            HAL_GPIO_TogglePin(RED_LED_PORT, RED_LED_PIN);
            HAL_Delay(200);
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
        HAL_GPIO_WritePin(ORANGE_LED_PORT, ORANGE_LED_PIN, GPIO_PIN_RESET);
    }
}

static bool spi_sanity_check(void)
{
    uint8_t status = sx1262_get_status();

    if (status == 0xFF)
    {
        return false;
    }

    uint8_t mode = (status >> 4) & 0x07;

    switch (mode)
    {
    case 0x2:
        HAL_GPIO_WritePin(GREEN_LED_PORT, GREEN_LED_PIN, GPIO_PIN_SET);
        break;
    case 0x3:
        HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, GPIO_PIN_SET);
        break;
    case 0x4:
        //HAL_GPIO_WritePin(ORANGE_LED_PORT, ORANGE_LED_PIN, GPIO_PIN_SET);
        break;
    case 0x5:
        HAL_GPIO_WritePin(ORANGE_LED_PORT, ORANGE_LED_PIN, GPIO_PIN_SET);
        break;
    case 0x6:
        HAL_GPIO_WritePin(BLUE_LED_PORT, BLUE_LED_PIN, GPIO_PIN_SET);
        break;
    default:
        return false;
    }

    return true;
}

static uint8_t sx1262_get_status(void)
{
    uint8_t txData = SX1262_CMD_GET_STATUS;
    uint8_t rxData = 0x00;

    sx1262_wait_on_busy();

    HAL_GPIO_WritePin(SX1262_NSS_GPIO_PORT, SX1262_NSS_GPIO_PIN, GPIO_PIN_RESET);

    if (HAL_SPI_Transmit(&hspi1, &txData, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        HAL_GPIO_WritePin(SX1262_NSS_GPIO_PORT, SX1262_NSS_GPIO_PIN, GPIO_PIN_SET);
        return 0xFF;
    }

    if (HAL_SPI_Receive(&hspi1, &rxData, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        HAL_GPIO_WritePin(SX1262_NSS_GPIO_PORT, SX1262_NSS_GPIO_PIN, GPIO_PIN_SET);
        return 0xFF;
    }

    HAL_GPIO_WritePin(SX1262_NSS_GPIO_PORT, SX1262_NSS_GPIO_PIN, GPIO_PIN_SET);

    return rxData;
}

static bool buffer_sanity_check(void)
{
    uint8_t testPattern[4] = {0x55, 0xAA, 0x33, 0xCC};
    uint8_t readBuffer[4] = {0};
    uint8_t offset = 0;

    if (!sx1262_write_buffer(offset, testPattern, (uint8_t)4))
    {
        return false;
    }

    if (!sx1262_read_buffer(offset, readBuffer, (uint8_t)4))
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

static bool sx1262_write_buffer(uint8_t offset, const uint8_t *data, uint8_t length)
{
    uint8_t header[2];
    header[0] = SX1262_CMD_WRITE_BUFFER;
    header[1] = offset;

    sx1262_wait_on_busy();

    HAL_GPIO_WritePin(SX1262_NSS_GPIO_PORT, SX1262_NSS_GPIO_PIN, GPIO_PIN_RESET);

    if (HAL_SPI_Transmit(&hspi1, header, 2, HAL_MAX_DELAY) != HAL_OK)
    {
        HAL_GPIO_WritePin(SX1262_NSS_GPIO_PORT, SX1262_NSS_GPIO_PIN, GPIO_PIN_SET);
        return false;
    }

    if (HAL_SPI_Transmit(&hspi1, (uint8_t *)data, length, HAL_MAX_DELAY) != HAL_OK)
    {
        HAL_GPIO_WritePin(SX1262_NSS_GPIO_PORT, SX1262_NSS_GPIO_PIN, GPIO_PIN_SET);
        return false;
    }

    HAL_GPIO_WritePin(SX1262_NSS_GPIO_PORT, SX1262_NSS_GPIO_PIN, GPIO_PIN_SET);

    return true;
}

static bool sx1262_read_buffer(uint8_t offset, uint8_t *data, uint8_t length)
{
    uint8_t header[3];
    header[0] = SX1262_CMD_READ_BUFFER;
    header[1] = offset;
    header[2] = 0x00; // TODO: Investigar por qué es necesario

    sx1262_wait_on_busy();

    HAL_GPIO_WritePin(SX1262_NSS_GPIO_PORT, SX1262_NSS_GPIO_PIN, GPIO_PIN_RESET);

    if (HAL_SPI_Transmit(&hspi1, header, 3, HAL_MAX_DELAY) != HAL_OK)
    {
        HAL_GPIO_WritePin(SX1262_NSS_GPIO_PORT, SX1262_NSS_GPIO_PIN, GPIO_PIN_SET);
        return false;
    }

    if (HAL_SPI_Receive(&hspi1, data, length, HAL_MAX_DELAY) != HAL_OK)
    {
        HAL_GPIO_WritePin(SX1262_NSS_GPIO_PORT, SX1262_NSS_GPIO_PIN, GPIO_PIN_SET);
        return false;
    }

    HAL_GPIO_WritePin(SX1262_NSS_GPIO_PORT, SX1262_NSS_GPIO_PIN, GPIO_PIN_SET);

    return true;
}

static void sx1262_wait_on_busy()
{
    GPIO_PinState gpio_state;

    do
    {
        gpio_state = HAL_GPIO_ReadPin(SX1262_BUSY_GPIO_PORT, SX1262_BUSY_GPIO_PIN);
    } while (gpio_state == GPIO_PIN_SET);
}