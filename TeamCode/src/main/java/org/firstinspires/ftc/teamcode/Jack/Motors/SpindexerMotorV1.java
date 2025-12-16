package org.firstinspires.ftc.teamcode.Jack.Motors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.Jack.Other.MultipleTelemetry;

public class SpindexerMotorV1 {
    public HardwareMap hardwareMap;
    public DcMotor motor;
    public DcMotorEx spindexer;
    public double velocity = 0;
    public double error = 0;
    public double targetPosition = 0;
    public double lastRPM= 0;
    public ElapsedTime tickTimer = new ElapsedTime();


    public double lastTicks = 0;
    public PIDController controller;
    public double kP, kI, kD, kF;
    public boolean usingPID = false;
    public double rpm = 0;


    public void init(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        motor = this.hardwareMap.get(DcMotor.class, RobotConstantsV1.arcShooterName);
        spindexer = (DcMotorEx) motor;
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setDirection(DcMotorSimple.Direction.FORWARD);
        this.usingPID = false;
    }

    public void init(HardwareMap hardwareMap, double kP, double kI, double kD, double kF) {
        this.hardwareMap = hardwareMap;
        motor = this.hardwareMap.get(DcMotor.class, RobotConstantsV1.arcShooterName);
        spindexer = (DcMotorEx) motor;
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setDirection(DcMotorSimple.Direction.FORWARD);
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        controller = new PIDController(this.kP, this.kI, this.kD);
        this.usingPID = true;
    }

    public void init(HardwareMap hardwareMap, PIDFCoefficients pidfCoefficients) {
        this.hardwareMap = hardwareMap;
        motor = this.hardwareMap.get(DcMotor.class, RobotConstantsV1.arcShooterName);
        spindexer = (DcMotorEx) motor;
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setDirection(DcMotorSimple.Direction.FORWARD);
        this.kP = pidfCoefficients.p;
        this.kI = pidfCoefficients.i;
        this.kD = pidfCoefficients.d;
        this.kF = pidfCoefficients.f;
        controller = new PIDController(this.kP, this.kI, this.kD);
        this.usingPID = true;
    }

    public void setMotorPower(double power){
        motor.setPower(power);
    }

    public void setDirection(DcMotorSimple.Direction direction){
        spindexer.setDirection(direction);
    }

    public void switchDirection(){
        if(spindexer.getDirection() == DcMotorSimple.Direction.FORWARD){
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

    public void setTargetPos(double pos_){
        targetPosition = pos_;
    }

    public void run(){
        //targetRPM = 2000 * Math.sin(tickTimer2.seconds());
        error = controller.getError();
        spindexer.setPower(controller.getOutput(spindexer.getCurrentPosition(), (int) targetPosition));
    }

    public double getTargetVelocity(){
        return velocity;
    }
    public double getVelocityRPM(){
        int currentTicks = spindexer.getCurrentPosition();
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
        spindexer.setVelocity(velocity);
    }


    public void log(Telemetry telemetry){
        telemetry.addData("Arc Motor Velocity: ", velocity);
        telemetry.addData("Arc Motor Position: ", spindexer.getCurrentPosition());
        telemetry.addData("Ready? : ", ready());
        telemetry.update();
    }

    public void log(MultipleTelemetry telemetry){
        telemetry.addData("Arc Motor Velocity: ", velocity);
        telemetry.addData("Arc Motor Position: ", spindexer.getCurrentPosition());
        telemetry.addData("Ready? : ", ready());
    }

    public void graph(MultipleTelemetry telemetry){
        telemetry.addData("Error", error);
        telemetry.addData("Power", spindexer.getPower());
        telemetry.addData("RPM", getVelocityRPM());
        telemetry.addData("Pos", spindexer.getCurrentPosition());
        telemetry.update();
    }

    public void graph(Telemetry telemetry){
        telemetry.addData("Target Velocity: ", getTargetVelocity());
        telemetry.addData("RPM: ", getVelocityRPM());
    }

}
