#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_

#define TRIG_GPIO_Port GPIOA
#define TRIG_Pin GPIO_PIN_2
#define speedOfSound  (0.0343 / 2);


double getUltrasonicDistance(void) ;

#endif
