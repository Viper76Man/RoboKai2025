package org.firstinspires.ftc.teamcode.Jack.Motors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.Jack.Other.MultipleTelemetry;

public class ArcShooterV1 {
    public HardwareMap hardwareMap;
    public DcMotor motor;
    public DcMotorEx shooter;
    public double velocity = 0;
    public double error = 0;
    public double targetRPM = RobotConstantsV1.defaultShooterRPM;
    public double lastTime = 0;
    public double measureInterval = 0.3;
    public double lastRPM= 0;
    public double newTicks = 0;
    public ElapsedTime tickTimer = new ElapsedTime();
    public ElapsedTime tickTimer2 = new ElapsedTime();


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

    public void init(HardwareMap hardwareMap, PIDCoefficients pidCoefficients) {
        this.hardwareMap = hardwareMap;
        motor = this.hardwareMap.get(DcMotor.class, RobotConstantsV1.arcShooterName);
        shooter = (DcMotorEx) motor;
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setDirection(DcMotorSimple.Direction.FORWARD);
        this.kP = pidCoefficients.p;
        this.kI = pidCoefficients.i;
        this.kD = pidCoefficients.d;
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
        targetRPM = rpm_;
    }

    public void run(){
        //targetRPM = 2000 * Math.sin(tickTimer2.seconds());

        error = controller.getError();
        shooter.setPower(runToVelocity(getVelocityRPM(), (int) targetRPM));
    }

    public double getTargetVelocity(){
        return velocity;
    }
    public double getTargetRPM(){
        return targetRPM;
    }
    public double getVelocityRPM(){
        int currentTicks = shooter.getCurrentPosition();
        double elapsed = tickTimer.seconds();
        if(elapsed > RobotConstantsV1.SHOOTER_UPDATE_TIME_SECONDS) {
            double delta = currentTicks - lastTicks;
            double tps = (elapsed > 0) ? delta / elapsed : 0;
            double rpm = (tps/28.0)*60.0;
            lastTicks = currentTicks;
            tickTimer.reset();
            lastRPM = rpm;
            return rpm;
        }
        return lastRPM;
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

    public double runToVelocity(double currentRPM, int targetRPM){
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
        telemetry.addData("Arc Motor Velocity: ", velocity);
        telemetry.addData("Arc Motor Position: ", shooter.getCurrentPosition());
        telemetry.addData("Ready? : ", ready());
    }

    public void graph(MultipleTelemetry telemetry){
        telemetry.addData("Error", error);
        telemetry.addData("Power", shooter.getPower());
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("RPM", getVelocityRPM());
        telemetry.addData("Pos", shooter.getCurrentPosition());
        telemetry.panels.update(telemetry.telemetry);
    }

    public void graph(Telemetry telemetry){
        telemetry.addData("Target Velocity: ", getTargetVelocity());
        telemetry.addData("RPM: ", getVelocityRPM());
    }

}
