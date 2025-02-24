#ifndef __LEDS_H_
#define __LEDS_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "gpio.h"

#define GREEN_LED_PIN  GPIO_PIN_12
#define GREEN_LED_PORT GPIOD

#define ORANGE_LED_PIN  GPIO_PIN_13
#define ORANGE_LED_PORT GPIOD

#define RED_LED_PIN  GPIO_PIN_14
#define RED_LED_PORT GPIOD

#define BLUE_LED_PIN  GPIO_PIN_15
#define BLUE_LED_PORT GPIOD

#ifdef __cplusplus
}
#endif

#endif /* __LEDS_H_ */