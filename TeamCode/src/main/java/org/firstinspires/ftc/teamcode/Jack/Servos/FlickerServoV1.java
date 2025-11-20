package org.firstinspires.ftc.teamcode.Jack.Servos;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.Jack.Other.MultipleTelemetry;

public class FlickerServoV1 {
    public Servo flicker;
    public void init(HardwareMap hardwareMap){
        flicker = hardwareMap.get(Servo.class, RobotConstantsV1.flickerServoName);
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
}
