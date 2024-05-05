#ifndef PID_H
#define PID_H

#include "Arduino.h"

struct PIDContext {
    float Kp; // Proportional gain
    float Ki; // Integral gain
    float Kd; // Derivative gain
    float integral;
    float previous_error;
    float setpoint; // Desired value
    float output; // Output value
    unsigned long lastTime;
};

class PID {
    public:
        PIDContext pitch;
        PIDContext roll;
        PIDContext yaw;
        
        void pid_setup(float Kp, float Ki, float Kd) {
            // Initialize PID contexts for pitch, roll, and yaw
            initializePID(pitch, Kp, Ki, Kd);
            initializePID(roll, Kp, Ki, Kd);
            initializePID(yaw, Kp, Ki, Kd);
        }
        
        void pid_loop(float current_pitch, float current_roll, float current_yaw, float dt) {
            pitch.setpoint = current_pitch; // Assuming setpoint is updated elsewhere
            roll.setpoint = current_roll; // Assuming setpoint is updated elsewhere
            yaw.setpoint = current_yaw; // Assuming setpoint is updated elsewhere

            pitch.output = computePID(pitch, dt);
            roll.output = computePID(roll, dt);
            yaw.output = computePID(yaw, dt);
        }
    
    private:
        void initializePID(PIDContext &ctx, float Kp, float Ki, float Kd) {
            ctx.Kp = Kp;
            ctx.Ki = Ki;
            ctx.Kd = Kd;
            ctx.integral = 0.0;
            ctx.previous_error = 0.0;
            ctx.lastTime = millis();
        }
        
        float computePID(PIDContext &ctx, float dt) {
            float error = ctx.setpoint - ctx.output; // Calculate error
            ctx.integral += error * dt; // Update integral
            float derivative = (error - ctx.previous_error) / dt; // Calculate derivative
            ctx.previous_error = error; // Update error history

            // Calculate PID output
            return (ctx.Kp * error) + (ctx.Ki * ctx.integral) + (ctx.Kd * derivative);
        }
};

#endif
