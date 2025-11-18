package org.firstinspires.ftc.teamcode.Jack.Motors;

import com.qualcomm.robotcore.util.ElapsedTime;

public class VelocityController {
    public ElapsedTime timer = new ElapsedTime();
    public double power = 0;
    public double error = 0;
    public double kP = 0;
    public double kI = 0;
    public double kD = 0;

    public double motorTPR;
    public PIDController controller;

    public double previousError = error;
    public double integralError = 0;

    public VelocityController(double motorTicksPerRev, double kP, double kI, double kD){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.motorTPR = motorTicksPerRev;
        this.controller = new PIDController(kP, kI, kD);
    }

    public double getOutputPOnly(int currentPosition, int target){
        error = target - currentPosition;
        power = error * kP;
        return power;
    }

    //@param currentPosition gets
    public double getOutput(double currentRPM, int targetRPM){
        return controller.getOutput((int) currentRPM, (int) targetRPM);
    }

    public double getError(){
        return controller.getError();
    }
}
