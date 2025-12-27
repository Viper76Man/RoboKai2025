package org.firstinspires.ftc.teamcode.Jack.Motors;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;

public class PIDController {
    public ElapsedTime timer = new ElapsedTime();
    public double power = 0;
    public double powerDrop = 0;
    public double error = 0;
    public double kP = 0;
    public double kI = 0;
    public double kD = 0;
    public double previousFilterEstimate = 0;
    public double currentFilterEstimate = 0;


    public double previousError = error;
    public double previousPower = 0;
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

    public void updatePIDsFromConstants(PIDCoefficients coefficients){
        kP = coefficients.p;
        kI = coefficients.i;
        kD = coefficients.d;
    }

    //@param currentPosition gets
    public double getOutput(int currentPosition, int target){
        previousPower = power;
        //Calculations
        error = currentPosition - target;
        double errorChange = (error - previousError) / timer.seconds();
        integralError = integralError + (error * timer.seconds());
        //Calculate output
        power = (error * kP) + (integralError * kI) + (errorChange * kD);
        powerDrop = previousPower - power;
        //Cleanup
        previousError = error;
        timer.reset();
        if(power > 1){
            power = 1;
        }
        if(power < -1){
            power = -1;
        }
        return power;
    }

    public double getOutput(double error){
        previousPower = power;
        //Calculations
        double errorChange = (error - previousError) / timer.seconds();
        integralError = integralError + (error * timer.seconds());
        //Calculate output
        power = (error * kP) + (integralError * kI) + (errorChange * kD);
        powerDrop = previousPower - power;
        //Cleanup
        previousError = error;
        timer.reset();
        if(power > 1){
            power = 1;
        }
        if(power < -1){
            power = -1;
        }
        return power;
    }

    public double getError(){
        return error;
    }

    public void setConstants(double kP, double kI, double kD){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setConstants(PIDCoefficients coefficients){
        this.kP = coefficients.p;
        this.kI = coefficients.i;
        this.kD = coefficients.d;
    }
}
