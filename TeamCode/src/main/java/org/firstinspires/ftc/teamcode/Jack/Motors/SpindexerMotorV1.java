package org.firstinspires.ftc.teamcode.Jack.Motors;

import com.pedropathing.ftc.localization.Encoder;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.Jack.Other.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.Jack.Other.Range;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;

public class SpindexerMotorV1 {
    public enum State {
        BALL_1_INTAKE,
        BALL_2_INTAKE,
        BALL_3_INTAKE,
        BALL_1_SHOOT,
        BALL_2_SHOOT,
        BALL_3_SHOOT,
    }

    public enum EncoderMeasurementMethod {
        ELC2,
        MOTOR
    }
    public HardwareMap hardwareMap;
    public DcMotor motor;
    public DcMotorEx spindexer;
    public double velocity = 0;
    public double error = 0;
    public double targetPosition = 0;
    public double targetPositionEncoder = 0;
    public Range range;
    public Range motorRange;
    public double lastRPM = 0;
    public ElapsedTime zeroTimer = new ElapsedTime();
    public EncoderMeasurementMethod method = EncoderMeasurementMethod.MOTOR;
    public DigitalChannel channel;

    public int index = 0;


    public double lastTicks = 0;
    public PIDController controller;
    public double kP, kI, kD, kF;
    public boolean usingPID = false;
    public double rpm = 0;
    public State state = State.BALL_1_INTAKE;
    public SpindexerEncoderV1 encoderv1 = new SpindexerEncoderV1();
    public PIDController controller_;

    public AnalogInput encoder;


    public void init(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        motor = this.hardwareMap.get(DcMotor.class, RobotConstantsV1.spindexerMotorName);
        spindexer = (DcMotorEx) motor;
        //encoderv1.init(hardwareMap);
        encoder = hardwareMap.get(AnalogInput.class, "spindexerEncoder");
        spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setDirection(DcMotorSimple.Direction.FORWARD);
        setState(State.BALL_1_INTAKE);
        controller = new PIDController(RobotConstantsV1.spindexerPIDs.p, RobotConstantsV1.spindexerPIDs.i, RobotConstantsV1.spindexerPIDs.d);
        this.usingPID = true;
        setTargetPos(RobotConstantsV1.SPINDEXER_MOTOR_BALL_1_INTAKE, EncoderMeasurementMethod.MOTOR);
    }

    public void init(HardwareMap hardwareMap, double kP, double kI, double kD, double kF) {
        this.hardwareMap = hardwareMap;
        motor = this.hardwareMap.get(DcMotor.class, RobotConstantsV1.spindexerMotorName);
        spindexer = (DcMotorEx) motor;
        //encoderv1.init(hardwareMap);
        spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setDirection(DcMotorSimple.Direction.FORWARD);
        encoder = hardwareMap.get(AnalogInput.class, "spindexerEncoder");
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        controller = new PIDController(this.kP, this.kI, this.kD);
        this.usingPID = true;
        setTargetPos(RobotConstantsV1.SPINDEXER_MOTOR_BALL_1_INTAKE, EncoderMeasurementMethod.MOTOR);
    }

    public void init(HardwareMap hardwareMap, PIDCoefficients pidCoefficients) {
        this.hardwareMap = hardwareMap;
        motor = this.hardwareMap.get(DcMotor.class, RobotConstantsV1.spindexerMotorName);
        spindexer = (DcMotorEx) motor;
        spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoder = hardwareMap.get(AnalogInput.class, "spindexerEncoder");
        setDirection(DcMotorSimple.Direction.FORWARD);
        this.kP = pidCoefficients.p;
        this.kI = pidCoefficients.i;
        this.kD = pidCoefficients.d;
        controller = new PIDController(this.kP, this.kI, this.kD);
        this.usingPID = true;
        setTargetPos(RobotConstantsV1.SPINDEXER_MOTOR_BALL_1_INTAKE, EncoderMeasurementMethod.MOTOR);
        //encoderv1.init(hardwareMap);
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
    }

    public void setTargetPos(double pos_, EncoderMeasurementMethod method){
        switch (method) {
            case ELC2:
                targetPosition = pos_;
                range = new Range(targetPosition, RobotConstantsV1.SPINDEXER_TOLERANCE);
                break;
            case MOTOR:
                targetPositionEncoder = pos_;
                motorRange = new Range(targetPositionEncoder, 50);
                break;
        }
    }

    public void run(){
        if(usingPID) {
            switch (method) {
                case ELC2:
                    error = targetPosition - getCurrentPosition();
                    double power = getPower(RobotConstantsV1.spindexerPIDs.p, 0.003);
                    setMotorPower(power);
                case MOTOR:
                    error = targetPositionEncoder - getCurrentPosition();
                    double power2 = getControllerPower(error);
                    setMotorPower(power2);
            }
        }
        else {
            error = targetPosition - spindexer.getCurrentPosition();
        }
        if(getCurrentPositionEncoder() != 0){
            zeroTimer.reset();
        }
    }

    public double getTargetVelocity(){
        return velocity;
    }
    public double getTargetPos(){
        return targetPosition;
    }

    public void setState(State state_){
        this.state = state_;
        this.index = state_.ordinal();
        update();
    }

    public State getNextState() {
       switch (state){
           case BALL_1_INTAKE:
               return State.BALL_2_INTAKE;
           case BALL_2_INTAKE:
               return State.BALL_3_INTAKE;
           case BALL_3_INTAKE:
               return State.BALL_1_INTAKE;
           case BALL_1_SHOOT:
               return State.BALL_2_SHOOT;
           case BALL_2_SHOOT:
               return State.BALL_3_SHOOT;
           case BALL_3_SHOOT:
               return State.BALL_1_SHOOT;
       }
       return State.BALL_1_INTAKE;
    }
    public State getPreviousState() {
        switch (state){
            case BALL_1_INTAKE:
                return State.BALL_3_INTAKE;
            case BALL_2_INTAKE:
                return State.BALL_1_INTAKE;
            case BALL_3_INTAKE:
                return State.BALL_2_INTAKE;
            case BALL_1_SHOOT:
                return State.BALL_3_SHOOT;
            case BALL_2_SHOOT:
                return State.BALL_1_SHOOT;
            case BALL_3_SHOOT:
                return State.BALL_2_SHOOT;
        }
        return State.BALL_1_INTAKE;
    }


    public void switchToShootOrIntake(){
        switch (state){
            case BALL_1_INTAKE:
            case BALL_2_INTAKE:
            case BALL_3_INTAKE:
                setState(State.BALL_1_SHOOT);
                break;
            case BALL_1_SHOOT:
            case BALL_2_SHOOT:
            case BALL_3_SHOOT:
                setState(State.BALL_1_INTAKE);
                break;
        }
    }

    public void update(){
        switch (method) {
            case ELC2:
                switch (state) {
                    case BALL_1_INTAKE:
                        setTargetPos(RobotConstantsV1.SPINDEXER_ENCODER_BALL_1_INTAKE, method);
                        break;
                    case BALL_1_SHOOT:
                        setTargetPos(RobotConstantsV1.SPINDEXER_ENCODER_BALL_1_SHOOT, method);
                        break;
                    case BALL_2_INTAKE:
                        setTargetPos(RobotConstantsV1.SPINDEXER_ENCODER_BALL_2_INTAKE, method);
                        break;
                    case BALL_2_SHOOT:
                        setTargetPos(RobotConstantsV1.SPINDEXER_ENCODER_BALL_2_SHOOT, method);
                        break;
                    case BALL_3_INTAKE:
                        setTargetPos(RobotConstantsV1.SPINDEXER_ENCODER_BALL_3_INTAKE, method);
                        break;
                    case BALL_3_SHOOT:
                        setTargetPos(RobotConstantsV1.SPINDEXER_ENCODER_BALL_3_SHOOT, method);
                        break;
                }
            case MOTOR:
                switch (state){
                    case BALL_1_INTAKE:
                        setTargetPos(RobotConstantsV1.SPINDEXER_MOTOR_BALL_1_INTAKE, method);
                        break;
                    case BALL_1_SHOOT:
                        setTargetPos(RobotConstantsV1.SPINDEXER_MOTOR_BALL_1_SHOOT, method);
                        break;
                    case BALL_2_INTAKE:
                        setTargetPos(RobotConstantsV1.SPINDEXER_MOTOR_BALL_2_INTAKE, method);
                        break;
                    case BALL_2_SHOOT:
                        setTargetPos(RobotConstantsV1.SPINDEXER_MOTOR_BALL_2_SHOOT, method);
                        break;
                    case BALL_3_INTAKE:
                        setTargetPos(RobotConstantsV1.SPINDEXER_MOTOR_BALL_3_INTAKE, method);
                        break;
                    case BALL_3_SHOOT:
                        setTargetPos(RobotConstantsV1.SPINDEXER_MOTOR_BALL_3_SHOOT, method);
                        break;
                }
        }
    }

    public boolean isReady(){
        return getError() < 5;
    }
    private void updateVelocity(){
        spindexer.setVelocity(velocity);
    }

    public double getEncoderError(){
        return Math.abs((targetPosition - getCurrentPositionEncoder()));
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

    //TODO: Check this
    public double getCurrentPositionEncoder(){
        double voltage = encoder.getVoltage();
        return (voltage / 3.3) * 4096;
    }

    public void resetEncoder(){
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getCurrentPosition(){
        if(getCurrentPositionEncoder() != 0){
            zeroTimer.reset();
        }
        if(method == EncoderMeasurementMethod.MOTOR) {
            spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            return spindexer.getCurrentPosition();
        }
        else {
            return getCurrentPositionEncoder();
        }
    }
    public void setMeasurementMethod(EncoderMeasurementMethod method){
        this.method = method;
    }

    public EncoderMeasurementMethod getMeasurementMethod(){
        return this.method;
    }

    public void switchEncoderMeasurementMethods(){
        if(this.method == EncoderMeasurementMethod.MOTOR){
            this.method = EncoderMeasurementMethod.ELC2;
        }
        else {
            this.method = EncoderMeasurementMethod.MOTOR;
        }
    }
    public List<Boolean> getStatesOfChannels(){
        List<Boolean> list = new ArrayList<>();
        list.add(encoderv1.getChannelAState());
        list.add(encoderv1.getChannelBState());
        return list;
    }

    public boolean isSpindexerReady(){
        switch (method) {
            case MOTOR:
                return motorRange.isInRange(getCurrentPosition());
            case ELC2:
                return range.isInRange(getCurrentPositionEncoder());
        }
        return false;
    }

    public double getPower(double kP, double maximumPower){
        double power = 0;
        maximumPower = Math.abs(maximumPower);
        if (!range.isInRange(getCurrentPositionEncoder())) {
            switch (method) {
                case MOTOR:
                    power = (error * kP);
                    break;
                case ELC2:
                    power = -(error * kP);
                    break;
            }
            if (power > maximumPower) {
                power = maximumPower;
            } else if (power < -maximumPower) {
                power = -maximumPower;
            }
            return power;

        }
        else {
            double p = 0;
            switch (method){
                case MOTOR:
                    p = (error * (kP / 10));
                    break;
                case ELC2:
                    p = -(error * (kP / 200));
                    break;
            }

            if (p > maximumPower) {
                p = maximumPower;
            }
            else if (p < -maximumPower) {
                p = -maximumPower;
            }
            return p;
        }
    }

    public double getError(){
        if(controller != null){
            switch (method) {
                case MOTOR:
                    return targetPositionEncoder - getCurrentPosition();
                case ELC2:
                    return targetPosition - getCurrentPositionEncoder();
            }
        }
        else {
            return 0;
        }
        return 0;
    }

    public double getControllerPower(){
        if(controller != null){
            switch (method) {
                case MOTOR:
                    return controller.getOutput((int) targetPositionEncoder, (int) getCurrentPosition());
                case ELC2:
                    return controller.getOutput((int) targetPosition, (int) getCurrentPositionEncoder());
            }
        }
        else {
            return 0;
        }
        return 0;
    }

    public double getControllerPower(double error){
        if(controller != null){
            return controller.getOutput(error);
        }
        else {
            return 0;
        }
    }



}
