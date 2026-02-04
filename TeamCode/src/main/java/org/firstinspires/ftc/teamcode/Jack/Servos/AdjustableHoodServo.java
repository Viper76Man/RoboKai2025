package org.firstinspires.ftc.teamcode.Jack.Servos;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;

public class AdjustableHoodServo {
    public Servo hood;
    public void init(HardwareMap hardwareMap){
        hood = hardwareMap.get(Servo.class, RobotConstantsV1.hoodName);
    }

    public void setPos(double pos){
        hood.setPosition(pos);
    }

    public void log(TelemetryManager telemetryM){
        telemetryM.addLine("Hood pos: " + hood.getPosition());

    }

    public double degToRange(double deg) {
        // Clamp input angle
        deg = Math.max(
                RobotConstantsV1.HOOD_STARTING_ANGLE,
                Math.min(deg, RobotConstantsV1.HOOD_MAX_ANGLE)
        );

        // Map degrees → [0.0, 0.2]
        return Math.max(0, Math.min((deg - RobotConstantsV1.HOOD_STARTING_ANGLE) / (RobotConstantsV1.HOOD_MAX_ANGLE - RobotConstantsV1.HOOD_STARTING_ANGLE) * RobotConstantsV1.HOOD_MAX_ANGLE, 1)) * 0.2;
    }
}
