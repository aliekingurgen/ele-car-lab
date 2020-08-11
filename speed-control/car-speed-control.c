#include "project.h"
#include <stdio.h>

int error; // present error
int accError = 0; // accumulated error for integral control
int diffError = 0; // differential error for derivative control
int prevError = 0; // previous error
int refSpeed = 12190; // mm/sec
int time; // time since last Hall sensor reading
int PWMnow = 40; // pulse width modulation to motor between 0 and 255, 40 is the initial value
int startCount = 0; // start integral control after certain time


CY_ISR(inter) { // runs when Hall sensor reads
    int captureVal;
    char strbuf[64];
    double speed;
    
    startCount++;

    captureVal = Timer_ReadCapture();
    time = (65536 - captureVal); // milliseconds
    speed = (3.93 / time)*1000; // mm/sec
    speed *= 1.08; // correction factor, 3.93 may not be completely accurate
    
    int speed100cm = (int) (speed * 100);
    if (startCount > 10) { // start integral control later, not immediately
        accError += error;
    }
    prevError = error;
    error = refSpeed - speed100cm; // in cm/sec
    diffError = (error - prevError) / (time*1000);
  
    sprintf(strbuf,"Time: %i Speed: %i Error: %i PWMnow: %i", time, speed100cm, error, PWMnow); // time in ms, speed in mm per sec
   
    // print to UART for debugging and collecting data
    char strbuf2[64];
    sprintf(strbuf2,"%s\r\n",strbuf);
    UART_PutString(strbuf2);
    
}

int PIController() {
    double kp = 0.05; // proportional constant
    double ki = 0.005; // integral constant
    double kd = 0.0; // differential constant
    
    double P_term = 0;
    double I_term = 0;
    double D_term = 0;
    
    // Compute the P term
    P_term = kp * error;
    
    // Compute the I term
    if (startCount > 10) {
        I_term = ki * accError;
    }

    // Compute the D term
    D_term = kd * diffError;
    
    int result = P_term + I_term + D_term;
    
    
    if (result > 255){ // make sure result is within range for car to work
        result = 255;
    } 
    else if (result < 0) {
        result = 0; // arbitrary, may change
    }
    
    return result;
}

void UpdatePWM(int fOutput) { // change the pulse width modulation to motor

    PWMnow = fOutput;
    PWM_WriteCompare(PWMnow);

}

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */
    int fOutput;
    

    /* initialization code */
    PWM_Start();
    UART_Start();
    LCD_Start();
    Timer_Start();
    
    LCD_Position(0,0);
    LCD_PrintString("C"); // make sure correct code got to PSoC :)
    
    PWM_WriteCompare(PWMnow);
    error = refSpeed; // To TEST initial conditions
    
    Hall_Interrupt_Start();
    Hall_Interrupt_SetVector(inter);

    for(;;)
    {
        // continuously update the motor
        fOutput = PIController();
        UpdatePWM(fOutput);
        
    }
}

/* [] END OF FILE */
