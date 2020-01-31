#ifndef __CALIBRATION_HH
#define __CALIBRATION_HH



// BASE, SHOULDER, ELBOW, ROLL, PITCH, WRIST (defined in controllerbase.hh)
const int SERVOPIN[] = {0, 1, 2, 3, 4, 5};
// Average of MIN & MAX (Mean position)
const int OFFSET[] = {1350, 1465, 700, 1372, 1550, 1366};// normally around 1500 except for elbow

#warning "Adjust these values for your robot to avoid self-collisions!"
#warning "Control your robot carefully until you have inserted appropriate boundaries here!"
const int MIN[] = {575, 538, 500, 544, 544, 544};// must not be below 544
const int MAX[] = {2340, 2300, 2000, 2200, 2253, 2400};// must not be above 2400

const float RESOLUTION[] = {8.844444444444445, 8.844444444444445, 8.844444444444445, 8.844444444444445, 11.433333333333334, 10.133333333333333};
const float MAXJERK = 250;

#endif
