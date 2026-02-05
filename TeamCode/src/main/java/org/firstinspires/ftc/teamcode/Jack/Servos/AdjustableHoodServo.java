package org.firstinspires.ftc.teamcode.Jack.Servos;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Jack.Drive.Robot;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;

public class AdjustableHoodServo {
    public Servo hood;
    public double currentDeg = RobotConstantsV1.HOOD_STARTING_ANGLE;
    public void init(HardwareMap hardwareMap){
        hood = hardwareMap.get(Servo.class, RobotConstantsV1.hoodName);
    }

    public void setPos(double pos){
        hood.setPosition(pos);
    }

    public void log(TelemetryManager telemetryM){
        telemetryM.addLine("Hood pos: " + hood.getPosition());
        telemetryM.addLine("Hood angle: " + currentDeg);

    }

    public void goToDeg(double deg){
        deg = Math.max(
                RobotConstantsV1.HOOD_STARTING_ANGLE,
                Math.min(deg, RobotConstantsV1.HOOD_MAX_ANGLE)
        );
        this.currentDeg = deg;
        setPos(degToRange(deg));
    }

    public double degToRange(double deg) {
        deg = Math.max(
                RobotConstantsV1.HOOD_STARTING_ANGLE,
                Math.min(deg, RobotConstantsV1.HOOD_MAX_ANGLE)
        );
        return Math.max(0, Math.min((deg - RobotConstantsV1.HOOD_STARTING_ANGLE) / (RobotConstantsV1.HOOD_MAX_ANGLE - RobotConstantsV1.HOOD_STARTING_ANGLE), 1)) * 0.2;
    }

}
