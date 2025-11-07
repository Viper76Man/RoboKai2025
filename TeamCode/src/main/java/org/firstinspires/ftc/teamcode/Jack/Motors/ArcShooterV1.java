package org.firstinspires.ftc.teamcode.Jack.Motors;

import android.app.backup.BackupAgent;

import androidx.core.text.util.LocalePreferences;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;

public class ArcShooterV1 {
    public HardwareMap hardwareMap;
    public DcMotor motor;
    public DcMotorEx shooter;
    public double velocity = 0;
    public double measureInterval;
    public double lastTPS = 0;
    public double newTicks = 0;
    public ElapsedTime tickTimer = new ElapsedTime();


    public double lastTicks = 0;
    public VelocityController controller;
    public double kP, kI, kD;
    public boolean usingPID = false;


    public void init(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        motor = this.hardwareMap.get(DcMotor.class, RobotConstantsV1.arcShooterName);
        shooter = (DcMotorEx) motor;
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setDirection(DcMotorSimple.Direction.FORWARD);
        this.usingPID = false;
    }

    public void init(HardwareMap hardwareMap, double kP, double kI, double kD) {
        this.hardwareMap = hardwareMap;
        motor = this.hardwareMap.get(DcMotor.class, RobotConstantsV1.arcShooterName);
        shooter = (DcMotorEx) motor;
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setDirection(DcMotorSimple.Direction.FORWARD);
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        controller = new VelocityController(RobotConstantsV1.SHOOTER_PPR, this.kP, this.kI, this.kD);
        this.usingPID = true;
    }

    public void setMotorPower(double power){
        motor.setPower(power);
    }

    public void setDirection(DcMotorSimple.Direction direction){
        shooter.setDirection(direction);
    }

    public void switchDirection(){
        if(shooter.getDirection() == DcMotorSimple.Direction.FORWARD){
            setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else {
            setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    public void setTargetVelocity(double velocity_){
        velocity = velocity_;
        updateVelocity();
    }

    public double getTargetVelocity(){
        return velocity;
    }
    public double getVelocity(){
        if(tickTimer.seconds() > measureInterval) {
            newTicks = Math.abs(shooter.getCurrentPosition()) - Math.abs(lastTicks);
            tickTimer.reset();
            lastTicks = shooter.getCurrentPosition();
            lastTPS = newTicks/(1/measureInterval);
            return lastTicks;
        }
        else {
            return lastTPS;
        }

    }

    public boolean ready(){
        return getVelocity() >= RobotConstantsV1.SHOOTER_TARGET_VELOCITY;
    }

    private void updateVelocity(){
        shooter.setVelocity(velocity);
    }

    public double runToVelocity(double currentTPS, double targetRPM){
        if(!usingPID){
            return 0.0;
        }
        else {
            return controller.getOutput(currentTPS, targetRPM);
        }
    }

    public void log(Telemetry telemetry){
        telemetry.addData("Arc Motor Velocity: ", velocity);
        telemetry.addData("Arc Motor Position: ", shooter.getCurrentPosition());
        telemetry.addData("Ready? : ", ready());
        telemetry.update();
    }

}
