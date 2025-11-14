package org.firstinspires.ftc.teamcode.Jack.Servos;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Jack.Drive.Robot;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;

public class StorageServoV1{
    public HardwareMap hardwareMap;
    public Servo servo;
    public double position = 0;
    public Ball intakeBall = Ball.BALL_1;

    public enum Ball {
        BALL_1,
        BALL_2,
        BALL_3
    }

    public void init(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        servo = hardwareMap.get(Servo.class, RobotConstantsV1.storageServoName);
    }

    public void setPosition(double position) {
        if (position < 0) {
            position = 1;
        }
        if (position > 1) {
            position = 0;
        }
        if (position == RobotConstantsV1.STORAGE_BALL_1) {
            intakeBall = Ball.BALL_1;
        } else if (position == RobotConstantsV1.STORAGE_BALL_2) {
            intakeBall = Ball.BALL_2;
        }
        else if(position == RobotConstantsV1.STORAGE_BALL_3){
            intakeBall = Ball.BALL_3;
        }
        this.position = position;
        servo.setPosition(position);
    }

    public void runToIntakeBall(Ball intake_ball){
        switch (intake_ball){
            case BALL_1:
                setPosition(RobotConstantsV1.STORAGE_BALL_1);
                break;
            case BALL_2:
                setPosition(RobotConstantsV1.STORAGE_BALL_2);
                break;
            case BALL_3:
                setPosition(RobotConstantsV1.STORAGE_BALL_3);
                break;
        }
    }

    public Ball getNextBall(Ball ball){
        switch (ball){
            case BALL_1:
                return Ball.BALL_2;
            case BALL_2:
                return Ball.BALL_3;
        }
        return Ball.BALL_1;
    }

    public Ball getIntakeBall(){
        return intakeBall;
    }

    public double getPosition(){
        return position;
    }

    public void log(Telemetry telemetry){
        telemetry.addData("Selected ball intake slot: ", intakeBall.name());
        telemetry.addData("Storage pos: ", getPosition());
    }
}
