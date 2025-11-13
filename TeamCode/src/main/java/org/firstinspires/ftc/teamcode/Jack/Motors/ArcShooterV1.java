package org.firstinspires.ftc.teamcode.Jack.Motors;

import android.app.backup.BackupAgent;
import android.provider.Settings;

import androidx.core.text.util.LocalePreferences;

import com.bylazar.graph.GraphManager;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.Jack.Other.MultipleTelemetry;

public class ArcShooterV1 {
    public HardwareMap hardwareMap;
    public DcMotor motor;
    public DcMotorEx shooter;
    public double velocity = 0;
    public double measureInterval = 0.3;
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

    public double getVelocity() {
        double elapsedTime = tickTimer.seconds();

        // Only update every so often (optional, e.g., every 50 ms)
        if (elapsedTime >= measureInterval) {
            int currentTicks = shooter.getCurrentPosition();
            double deltaTicks = currentTicks - lastTicks;
            // ticks per second = deltaTicks / elapsedTime
            lastTPS = deltaTicks / elapsedTime;
            // Update tracking vars
            lastTicks = currentTicks;
            tickTimer.reset();
        }
        return tPStoRPM(lastTPS, 28);
    }

    public double tPStoRPM(double tps, double motorTicksPerRev){
        return (tps / motorTicksPerRev) * 60.0;
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

    public void log(MultipleTelemetry telemetry){
        telemetry.addData("Arc Motor Velocity: ", velocity);
        telemetry.addData("Arc Motor Position: ", shooter.getCurrentPosition());
        telemetry.addData("Ready? : ", ready());
        telemetry.update();
    }

    public void graph(MultipleTelemetry telemetry){
        telemetry.panels.addData("Target Velocity: ", getTargetVelocity());
        telemetry.panels.addData("Actual Velocity: ", getVelocity());
        telemetry.panels.update();
    }

    public void graph(Telemetry telemetry){
        telemetry.addData("Target Velocity: ", getTargetVelocity());
        telemetry.addData("Actual Velocity: ", getVelocity());
    }

}
