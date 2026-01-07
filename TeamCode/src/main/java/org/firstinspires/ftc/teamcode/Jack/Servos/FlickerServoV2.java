package org.firstinspires.ftc.teamcode.Jack.Servos;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.Jack.Other.MultipleTelemetry;

public class FlickerServoV2 {
    public Servo flicker;
    public enum State {
        IDLE,
        DOWN,
        TRAVEL_UP,
        UP,
        TRAVEL_DOWN
    }
    public State state = State.DOWN;
    public double position = RobotConstantsV1.FLICKER_SERVO_DOWN;
    public ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();

    public void init(HardwareMap hardwareMap, String servoName){
        flicker = hardwareMap.get(Servo.class, servoName);
    }

    public void log(MultipleTelemetry telemetry){
        telemetry.addData("Flicker pos: ", flicker.getPosition());
    }
    public void log(TelemetryManager telemetry){
        telemetry.addData("Flicker pos: ", flicker.getPosition());
    }
    public void log(Telemetry telemetry){
        telemetry.addData("Flicker pos: ", flicker.getPosition());
    }

    public void setPosition(double position_){
        this.position = position_;
    }


    public void update(boolean isSpindexerReady){
        flicker.setPosition(position);
        if(state == State.DOWN && isSpindexerReady){
            setPosition(RobotConstantsV1.FLICKER_SERVO_UP);
            setState(State.TRAVEL_UP);
        }
        else if(state == State.TRAVEL_UP && stateTimer.seconds() > 0.35){
            setState(State.UP);
        }
        else if(state == State.UP && stateTimer.seconds() > 0.4){
            setPosition(RobotConstantsV1.FLICKER_SERVO_DOWN);
            setState(State.TRAVEL_DOWN);
        }
        else if(position == RobotConstantsV1.FLICKER_SERVO_DOWN && stateTimer.seconds() > 0.3){
            setState(State.IDLE);
            resetStateTimer();
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

    public void setState(State state_){
        this.state = state_;
        switch(state_){
            case TRAVEL_DOWN:
            case TRAVEL_UP:
                resetStateTimer();
                break;
        }
    }

    public void resetTimer(){
        timer.reset();
    }
    public void resetStateTimer(){
        stateTimer.reset();
    }

    public double getStateTimerSeconds(){
        return stateTimer.seconds();
    }
}
