package org.firstinspires.ftc.teamcode.Jack.Motors;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Jack.Drive.Robot;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.Jack.Other.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.Jack.Other.Range;

public class ArcShooterV1 {
    public HardwareMap hardwareMap;
    public DcMotor motor;
    public DcMotorEx shooter;
    public DcMotor motor2;
    public DcMotorEx shooter2;
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
    public double kP, kI, kD, kF;
    public boolean usingPID = false;
    public double rpm = 0;


    public void init(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        motor = this.hardwareMap.get(DcMotor.class, RobotConstantsV1.arcShooterName);
        motor2 = this.hardwareMap.get(DcMotor.class, "leftArc");
        shooter = (DcMotorEx) motor;
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setDirection(RobotConstantsV1.rightShooterDirection);
        shooter2 = (DcMotorEx) motor2;
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setDirection(RobotConstantsV1.leftShooterDirection);
        this.usingPID = false;
    }

    public void init(HardwareMap hardwareMap, double kP, double kI, double kD, double kF) {
        this.hardwareMap = hardwareMap;
        motor = this.hardwareMap.get(DcMotor.class, RobotConstantsV1.arcShooterName);
        motor2 = this.hardwareMap.get(DcMotor.class, "leftArc");
        shooter = (DcMotorEx) motor;
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setDirection(RobotConstantsV1.rightShooterDirection);
        shooter2 = (DcMotorEx) motor2;
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setDirection(RobotConstantsV1.leftShooterDirection);
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        controller = new VelocityController(RobotConstantsV1.SHOOTER_PPR, this.kP, this.kI, this.kD, this.kF);
        this.usingPID = true;
    }

    public void init(HardwareMap hardwareMap, PIDFCoefficients pidfCoefficients) {
        this.hardwareMap = hardwareMap;
        motor = this.hardwareMap.get(DcMotor.class, RobotConstantsV1.arcShooterName);
        motor2 = this.hardwareMap.get(DcMotor.class, "leftArc");
        shooter = (DcMotorEx) motor;
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setDirection(RobotConstantsV1.rightShooterDirection);
        shooter2 = (DcMotorEx) motor2;
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setDirection(RobotConstantsV1.leftShooterDirection);
        this.kP = pidfCoefficients.p;
        this.kI = pidfCoefficients.i;
        this.kD = pidfCoefficients.d;
        this.kF = pidfCoefficients.f;
        controller = new VelocityController(RobotConstantsV1.SHOOTER_PPR, this.kP, this.kI, this.kD, this.kF);
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
        double power = runToVelocity(getVelocityRPM(), (int) targetRPM);
        shooter.setPower(power);
        shooter2.setPower(power);
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

    public double getVelocityFeetPerSecond(double distanceFeetPerRotation){
        double rpm = getVelocityRPM();
        double rotationsPerSec = rpm / 60.0;
        //11.87 in per rotation = 0.99 ft
        return rotationsPerSec * distanceFeetPerRotation;
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

    public void updatePIDsFromConstants(){
        if(usingPID) {
            controller.updatePIDsFromConstants(RobotConstantsV1.arcPIDs);
        }
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

    public void log(TelemetryManager telemetry){
        telemetry.addData("Arc Motor Velocity: ", velocity);
        telemetry.addData("Arc Motor Position: ", shooter.getCurrentPosition());
        telemetry.addData("Ready? : ", ready());
    }

    public void graph(MultipleTelemetry telemetry){
        telemetry.addData("Error", error);
        telemetry.addData("Power", shooter.getPower());
        telemetry.addData("Power 2", shooter2.getPower());
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("RPM", getVelocityRPM());
        telemetry.addData("Pos", shooter.getCurrentPosition());
    }

    public void graph(TelemetryManager telemetry){
        telemetry.addData("Error", error);
        telemetry.addData("Power", shooter.getPower());
        telemetry.addData("Power 2", shooter2.getPower());
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("RPM", getVelocityRPM());
        telemetry.addData("Pos", shooter.getCurrentPosition());
    }

    public void graph(Telemetry telemetry){
        telemetry.addData("Error", error);
        telemetry.addData("Power", shooter.getPower());
        telemetry.addData("Power 2", shooter2.getPower());
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("RPM", getVelocityRPM());
        telemetry.addData("Pos", shooter.getCurrentPosition());
    }

    public boolean isInRange(double tolerance){
        return new Range((targetRPM), tolerance).isInRange(getVelocityRPM());
    }

}
