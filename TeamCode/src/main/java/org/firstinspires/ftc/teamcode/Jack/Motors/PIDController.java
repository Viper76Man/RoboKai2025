package org.firstinspires.ftc.teamcode.Jack.Motors;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    public ElapsedTime timer;
    public double power = 0;
    public double error = 0;
    public double kP = 0;
    public double kI = 0;
    public double kD = 0;


    public double previousError = error;
    public double integralError = 0;

    public PIDController(double kP, double kI, double kD){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double getOutputPOnly(int currentPosition, int target){
        error = target - currentPosition;
        power = error * kP;
        return power;
    }

    public double getOutput(int currentPosition, int target){
        //Calculations
        error = target - currentPosition;
        double errorChange = (error - previousError) / timer.seconds();
        integralError = integralError + (error * timer.seconds());

        //Calculate output
        power = (error * kP) + (integralError * kI) + (errorChange * kD);

        //Cleanup
        previousError = error;
        timer.reset();
        return power;
    }
}
