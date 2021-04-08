// Robot hardware parameter
#define robot_width 0.2
#define wheel_radius 0.015

#define PWM_R 9
#define DIR_R 10
#define PWM_L 11
#define DIR_L 12

#define ENCO_CPR 48
#define GEAR_RATIO 98.78

const byte ENCO_R_1 = 4;
const byte ENCO_R_2 = 3;
// yellow
const byte ENCO_L_1 = 5; 
// white
const byte ENCO_L_2 = 6;

#define read_R_1 bitRead(PIND, ENCO_R_1)
#define read_R_2 bitRead(PIND, ENCO_R_2)
#define read_L_1 bitRead(PIND, ENCO_L_1)
#define read_L_2 bitRead(PIND, ENCO_L_2)
