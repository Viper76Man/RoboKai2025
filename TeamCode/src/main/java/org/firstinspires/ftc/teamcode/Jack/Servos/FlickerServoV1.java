package org.firstinspires.ftc.teamcode.Jack.Servos;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.Jack.Other.MultipleTelemetry;

public class FlickerServoV1 {
    public Servo flicker;
    public ElapsedTime timer = new ElapsedTime();
    public void init(HardwareMap hardwareMap, String servoName){
        flicker = hardwareMap.get(Servo.class, servoName);
    }

    public void log(MultipleTelemetry telemetry){
        telemetry.addData("Flicker pos: ", flicker.getPosition());
    }
    public void setPosition(double position){
        flicker.setPosition(position);
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
