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
    public ElapsedTime downForTimer = new ElapsedTime();
    public double position = RobotConstantsV1.FLICKER_SERVO_DOWN;
    public boolean up = false;
    public ElapsedTime timer = new ElapsedTime();
    public ElapsedTime travelTimer = new ElapsedTime();
    public void init(HardwareMap hardwareMap, String servoName){
        flicker = hardwareMap.get(Servo.class, servoName);
    }

    public void log(MultipleTelemetry telemetry){
        telemetry.addData("Flicker pos: ", flicker.getPosition());
    }
    public void setPosition(double position_){
        resetTravelTimer();
        this.position = position_;
    }

    public void up(){
        resetTravelTimer();
        flicker.setPosition(RobotConstantsV1.FLICKER_SERVO_UP);
        this.position = RobotConstantsV1.FLICKER_SERVO_UP;
        up = true;
    }

    public void down(){
        resetTravelTimer();
        flicker.setPosition(RobotConstantsV1.FLICKER_SERVO_DOWN);
        this.position = RobotConstantsV1.FLICKER_SERVO_DOWN;
        up = false;
    }

    public void update(){
        flicker.setPosition(position);
        if(position == RobotConstantsV1.FLICKER_SERVO_UP && travelTimer.seconds() > 0.4){
            state = State.UP;
        }
        if(position == RobotConstantsV1.FLICKER_SERVO_UP){
            up = true;
        }
        if(position == RobotConstantsV1.FLICKER_SERVO_DOWN){
            up = false;
        }
        else if(position == RobotConstantsV1.FLICKER_SERVO_DOWN && travelTimer.seconds() > 0.4){
            state = State.DOWN;
        }
        if(position == RobotConstantsV1.FLICKER_SERVO_UP){
            downForTimer.reset();
        }

    }
    public void resetTravelTimer(){
        travelTimer.reset();
    }
    public void setPosition(State position){
        switch (position){
            case UP:
                setPosition(RobotConstantsV1.FLICKER_SERVO_UP);
                this.position = RobotConstantsV1.FLICKER_SERVO_UP;
                up = true;
                break;
            case DOWN:
                setPosition(RobotConstantsV1.FLICKER_SERVO_DOWN);
                this.position = RobotConstantsV1.FLICKER_SERVO_DOWN;
                up = false;
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
