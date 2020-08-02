#include motor_driver.h

#include<wiringPi.h>
#include<softPwm.h>


/*
 * Motor class constructor
 */
Motor::Motor(int pin_forward, int pin_reverse) {
	wiringPiSerup();

	pinMode(pin_forward, PWM_OUTPUT);
	pinMode(pin_reverse, PWM_OUTPUT);

	softPwmCreate(pin_forward, 0, 100);
	softPwmCreate(pin_reverse, 0, 100);
}


/*
 * Drive motor at given speed with encoded direction
 * int speed => -100 to 100
 */
Motor::drive_motor(int speed) {

	if (speed > 0) {
		softPwmWrite(pin_forward, speed);
		softPwmWrite(pin_reverse, 0);
	}
	else if (speed < 0) {
		softPwmWrite(pin_forward, 0);
		softPwmWrite(pin_reverse, speed);
	}
	else {
		softPwmWrite(pin_forward, 0);
		softPwmWrite(pin_reverse, 0);
	}
}