#ifndef PID_HPP
#define PID_HPP

#include <chrono>

class PID
{
private:
    float Kp;
    float Ki;
    float Kd;
    float dt;
    float min_out;
    float max_out;
    float min_integral;
    float max_integral;
    float proportional;
    float integral;
    float derivative;
    float last_error;
    float output_speed;
    std::chrono::system_clock::time_point last_call;

public:
    PID()
    {
        this->Kp = 0;
        this->Ki = 0;
        this->Kd = 0;
        this->dt = 0;
        this->min_out = 0;
        this->max_out = 0;
        this->min_integral = 0;
        this->max_integral = 0;
        this->proportional = 0;
        this->integral = 0;
        this->derivative = 0;
        this->last_error = 0;
        this->output_speed = 0;
        this->last_call = std::chrono::high_resolution_clock::now();
    }

    void init(float Kp, float Ki, float Kd, float dt, float min_out, float max_out, float min_integral, float max_integral)
    {
        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;
        this->dt = dt;
        this->min_out = min_out;
        this->max_out = max_out;
        this->min_integral = min_integral;
        this->max_integral = max_integral;
    }

    void set_dt(float dt)
    {
        this->dt = dt;
    }

    float calculate(float error)
    {
        std::chrono::high_resolution_clock::time_point t_now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_seconds = t_now - this->last_call;
        if (elapsed_seconds.count() > 2)
        {
            this->integral = 0;
            this->last_error = 0;
        }
        this->last_call = std::chrono::high_resolution_clock::now();

        float min_out_used = this->min_out;
        float max_out_used = this->max_out;
        float min_integral_used = this->min_integral;
        float max_integral_used = this->max_integral;

        this->proportional = this->Kp * error;
        this->integral += this->Ki * error;
        this->derivative = this->Kd * (error - this->last_error);

        this->last_error = error;

        if (this->integral > this->max_integral)
            this->integral = this->max_integral;
        else if (this->integral < this->min_integral)
            this->integral = this->min_integral;

        this->output_speed = this->proportional + this->integral + this->derivative;

        if (this->output_speed > this->max_out)
            this->output_speed = this->max_out;
        else if (this->output_speed < this->min_out)
            this->output_speed = this->min_out;
        return this->output_speed;
    }

    float calculate(float error, float minmax)
    {
        std::chrono::high_resolution_clock::time_point t_now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_seconds = t_now - this->last_call;
        if (elapsed_seconds.count() > 2)
        {
            this->integral = 0;
            this->last_error = 0;
        }
        this->last_call = std::chrono::high_resolution_clock::now();

        float min_out_used = -minmax;
        float max_out_used = minmax;
        float min_integral_used = this->min_integral;
        float max_integral_used = this->max_integral;

        this->proportional = this->Kp * error;
        this->integral += this->Ki * error;
        this->derivative = this->Kd * (error - this->last_error);

        this->last_error = error;

        if (this->integral > this->max_integral)
            this->integral = this->max_integral;
        else if (this->integral < this->min_integral)
            this->integral = this->min_integral;

        this->output_speed = this->proportional + this->integral + this->derivative;

        if (this->output_speed > this->max_out)
            this->output_speed = this->max_out;
        else if (this->output_speed < this->min_out)
            this->output_speed = this->min_out;
        return this->output_speed;
    }

    float calculate(float error, float min_out, float max_out)
    {
        std::chrono::high_resolution_clock::time_point t_now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_seconds = t_now - this->last_call;
        if (elapsed_seconds.count() > 2)
        {
            this->integral = 0;
            this->last_error = 0;
        }
        this->last_call = std::chrono::high_resolution_clock::now();

        float min_out_used = min_out;
        float max_out_used = max_out;
        float min_integral_used = this->min_integral;
        float max_integral_used = this->max_integral;

        this->proportional = this->Kp * error;
        this->integral += this->Ki * error;
        this->derivative = this->Kd * (error - this->last_error);

        this->last_error = error;

        if (this->integral > max_integral_used)
            this->integral = max_integral_used;
        else if (this->integral < min_integral_used)
            this->integral = min_integral_used;

        this->output_speed = this->proportional + this->integral + this->derivative;

        if (this->output_speed > max_out_used)
            this->output_speed = max_out_used;
        else if (this->output_speed < min_out_used)
            this->output_speed = min_out_used;
        return this->output_speed;
    }

    void reset()
    {
        this->integral = 0;
        this->last_error = 0;
    }
};

#endif