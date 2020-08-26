#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

class Motor {

	public:

		Motor(int pin_forward, int pin_reverse);
		
		void drive_motor(int speed);
}

#endif