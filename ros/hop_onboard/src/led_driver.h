#ifndef LED_DRIVER_H
#define LED_DRIVER_H

class Led {

	public:
		int m_pin;

		Led(int pin);
		
		void led_on();
		void led_off();
}

#endif