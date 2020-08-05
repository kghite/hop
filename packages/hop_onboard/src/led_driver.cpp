/*
 * Raspberry Pi LED control
 */

#include <led_driver.h>

#include <wiringPi.h>


/*
 * Motor class constructor
 */
Led::Led(int pin) {
	wiringPiSetupGpio();

	m_pin = pin;

	pinMode(pin, OUTPUT);
}


/*
 * Turn on led
 */
Led::led_on() {

	digitalWrite(m_pin, HIGH)
}


/*
 * Turn off led
 */
Led::led_off() {

	digitalWrite(m_pin, LOW)
}