package org.firstinspires.ftc.teamcode.Jack.Motors;

import com.bylazar.graph.GraphEntry;
import com.bylazar.graph.GraphManager;
import com.bylazar.panels.Panels;
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
    public double measureInterval = 0.1;
    public double lastTPS = 0;
    public double newTicks = 0;
    public ElapsedTime tickTimer = new ElapsedTime();


    public double lastTicks = 0;
    public VelocityController controller;
    public double kP, kI, kD;
    public boolean usingPID = false;
    public double rpm = 0;


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

    public void setTargetRPM(double rpm_){
        rpm = rpm_;
    }

    public void run(){
        shooter.setPower(runToVelocity(getVelocityRPM(), 6000));
    }

    public double getTargetVelocity(){
        return velocity;
    }
    public double getTargetRPM(){
        return rpm;
    }

    public double getVelocityRPM() {
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
    public double rPMtoTPS(double rpm, double motorTicksPerRev){
        return (rpm / 60.0) * motorTicksPerRev;
    }


    public boolean ready(){
        return getVelocityRPM() >= RobotConstantsV1.SHOOTER_TARGET_RPM;
    }

    private void updateVelocity(){
        shooter.setVelocity(velocity);
    }

    public double runToVelocity(double currentRPM, double targetRPM){
        if(!usingPID){
            return 0.0;
        }
        else {
            return controller.getOutput(currentRPM, targetRPM);
        }
    }

    public void log(Telemetry telemetry){
        telemetry.addData("Arc Motor Velocity: ", velocity);
        telemetry.addData("Arc Motor Position: ", shooter.getCurrentPosition());
        telemetry.addData("Ready? : ", ready());
        telemetry.update();
    }

    public void log(MultipleTelemetry telemetry){
        telemetry.telemetry.addData("Arc Motor Velocity: ", velocity);
        telemetry.telemetry.addData("Arc Motor Position: ", shooter.getCurrentPosition());
        telemetry.telemetry.addData("Ready? : ", ready());
        telemetry.telemetry.update();
    }

    public void graph(MultipleTelemetry telemetry){
        telemetry.panels.addData("Target Velocity", getTargetVelocity());
        telemetry.panels.addData("RPM", getVelocityRPM());
        telemetry.panels.update(telemetry.telemetry);
    }

    public void graph(Telemetry telemetry){
        telemetry.addData("Target Velocity: ", getTargetVelocity());
        telemetry.addData("RPM: ", getVelocityRPM());
    }

}
