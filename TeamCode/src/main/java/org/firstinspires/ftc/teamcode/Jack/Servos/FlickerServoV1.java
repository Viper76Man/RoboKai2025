package org.firstinspires.ftc.teamcode.Jack.Servos;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.Jack.Other.MultipleTelemetry;

public class FlickerServoV1 {
    public Servo flicker;
    public enum State {
        UP,
        DOWN
    }
    public State state = State.DOWN;
    public ElapsedTime timer = new ElapsedTime();
    public void init(HardwareMap hardwareMap, String servoName){
        flicker = hardwareMap.get(Servo.class, servoName);
    }

    public void log(MultipleTelemetry telemetry){
        telemetry.addData("Flicker pos: ", flicker.getPosition());
    }
    public void setPosition(double position){
        if(position == RobotConstantsV1.FLICKER_SERVO_UP){
            state = State.UP;
        }
        else if(position == RobotConstantsV1.FLICKER_SERVO_DOWN){
            state = State.DOWN;
        }
        flicker.setPosition(position);
    }
    public void setPosition(State position){
        switch (position){
            case UP:
                setPosition(RobotConstantsV1.FLICKER_SERVO_UP);
                state = State.UP;
                break;
            case DOWN:
                setPosition(RobotConstantsV1.FLICKER_SERVO_DOWN);
                state = State.DOWN;
                break;
        }
    }

    public State getState(){
        return state;
    }

    public void switchPositions(){
        if(flicker.getPosition() == RobotConstantsV1.FLICKER_SERVO_DOWN){
            setPosition(RobotConstantsV1.FLICKER_SERVO_UP);
        }
        else {
            setPosition(RobotConstantsV1.FLICKER_SERVO_DOWN);
        }
    }

    public void resetTimer(){
        timer.reset();
    }
}
