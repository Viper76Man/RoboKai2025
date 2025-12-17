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
    public enum State {
        BALL_1_INTAKE,
        BALL_1_SHOOT,
        BALL_2_INTAKE,
        BALL_2_SHOOT,
        BALL_3_INTAKE,
        BALL_3_SHOOT
    }
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
    public State state = State.BALL_1_INTAKE;


    public void init(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        motor = this.hardwareMap.get(DcMotor.class, RobotConstantsV1.spindexerMotorName);
        spindexer = (DcMotorEx) motor;
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setDirection(DcMotorSimple.Direction.FORWARD);
        this.usingPID = false;
    }

    public void init(HardwareMap hardwareMap, double kP, double kI, double kD, double kF) {
        this.hardwareMap = hardwareMap;
        motor = this.hardwareMap.get(DcMotor.class, RobotConstantsV1.spindexerMotorName);
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
        if(usingPID) {
            error = controller.getError();
            update();
            spindexer.setPower(controller.getOutput(spindexer.getCurrentPosition(), (int) targetPosition));
        }
        else {
            spindexer.setTargetPosition((int) targetPosition);
            spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spindexer.setPower(0.7);
        }
    }

    public double getTargetVelocity(){
        return velocity;
    }

    public void setState(State state_){
        this.state = state_;
    }

    public State getNextState() {
        int state_ = state.ordinal() + 1;
        if(state_ < State.values().length){
            state_ = 0;
        }
        return State.values()[state_];
    }

    public void update(){
        switch (state) {
            case BALL_1_INTAKE:
                setTargetPos(RobotConstantsV1.SPINDEXER_MOTOR_BALL_1_INTAKE);
                break;
            case BALL_1_SHOOT:
                setTargetPos(RobotConstantsV1.SPINDEXER_MOTOR_BALL_1_SHOOT);
                break;
            case BALL_2_INTAKE:
                setTargetPos(RobotConstantsV1.SPINDEXER_MOTOR_BALL_2_INTAKE);
                break;
            case BALL_2_SHOOT:
                setTargetPos(RobotConstantsV1.SPINDEXER_MOTOR_BALL_2_SHOOT);
                break;
            case BALL_3_INTAKE:
                setTargetPos(RobotConstantsV1.SPINDEXER_MOTOR_BALL_3_INTAKE);
                break;
            case BALL_3_SHOOT:
                setTargetPos(RobotConstantsV1.SPINDEXER_MOTOR_BALL_3_SHOOT);
                break;
        }
    }

    public boolean isReady(){
        return getError() < 5;
    }
    private void updateVelocity(){
        spindexer.setVelocity(velocity);
    }

    public double getError(){
        if(usingPID) {
            return Math.abs(controller.error);
        }
        else {
            return Math.abs((targetPosition - spindexer.getCurrentPosition()));
        }
    }


    public void log(Telemetry telemetry){
        telemetry.addData("Arc Motor Velocity: ", velocity);
        telemetry.addData("Arc Motor Position: ", spindexer.getCurrentPosition());
        telemetry.update();
    }

    public void log(MultipleTelemetry telemetry){
        telemetry.addData("Arc Motor Velocity: ", velocity);
        telemetry.addData("Arc Motor Position: ", spindexer.getCurrentPosition());
    }

    public void graph(MultipleTelemetry telemetry){
        telemetry.addData("Error", error);
        telemetry.addData("Power", spindexer.getPower());
        telemetry.addData("Pos", spindexer.getCurrentPosition());
        telemetry.update();
    }

    public void graph(Telemetry telemetry){
        telemetry.addData("Target Velocity: ", getTargetVelocity());
    }

}
