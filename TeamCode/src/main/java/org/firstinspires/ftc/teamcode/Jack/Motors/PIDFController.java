package org.firstinspires.ftc.teamcode.Jack.Motors;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;

public class PIDFController {
    public ElapsedTime timer = new ElapsedTime();
    public PIDController controller;
    public double power = 0;
    public double powerDrop = 0;
    public double error = 0;
    public double kP = 0;
    public double kI = 0;
    public double kD = 0;
    public double kF = 0;


    public double previousError = error;
    public double previousPower = 0;
    public double integralError = 0;

    public PIDFController(double kP, double kI, double kD, double kF){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        controller = new PIDController(kP, kI, kD);
    }

    public double getOutputPOnly(int currentPosition, int target){
        error = target - currentPosition;
        power = error * kP;
        return power;
    }

    public void updatePIDsFromConstants(){
        kF = RobotConstantsV1.arcPIDs.f;
        controller.updatePIDsFromConstants();
    }

    //@param currentPosition gets
    public double getOutput(int currentPosition, int target){
        updatePIDsFromConstants();
        return controller.getOutput(currentPosition, target) + (kF * target);
    }

    public double getError(){
        return error;
    }
}
