package org.firstinspires.ftc.teamcode.p2p;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {

    double Kp = 0;
    double Ki = 0;
    double Kd = 0;

    double reference = 0;

    double integralSum = 0;

    double lastError = 0;

    // Elapsed timer class from SDK, please use it, it's epic
    ElapsedTime timer = new ElapsedTime();

    public PIDController(double p, double i, double d) {
        this.Kp = p; this.Ki = i; this.Kd = d;
    }

    public double calculate(double error) {

        timer.reset();
        // rate of change of the error
        double derivative = (error - lastError) / timer.seconds();

        // sum of all error over time

        lastError = error;

        // reset the timer for next time

        return (Kp * error) + (Kd * derivative);
    }

    public void setP(double p) {
        this.Kp = p;
    }

}
