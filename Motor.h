#define RW_Speed 3    // Speed on 3/PD0
#define LW_Speed 11  // Speed on 11/PB7

#define RW_Dir 7   // Direction on 7/PE6
#define LW_Dir 17 // Direction on 17/PB0  

extern volatile int16_t LW_Current;
extern volatile int16_t RW_Current;

void Motor_Init();
void Speed_Set(int16_t, int16_t);
void Speed_Set(int16_t, int16_t);


// Motor A is now controlled as follows 
// Direction is controlled by PB0 --------> 17
// Speed or PWM is now on OC0A or PB7 ---------> 


// Motor B is now controlled as follows 
// Direction is controlled by PE6 ------> 7
// Speed or PWM is now on OC0B or PD0  -------->3