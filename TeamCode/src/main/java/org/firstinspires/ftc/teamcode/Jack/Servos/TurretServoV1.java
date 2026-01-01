package org.firstinspires.ftc.teamcode.Jack.Servos;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Jack.Camera.Limelight3A.LimelightV1;
import org.firstinspires.ftc.teamcode.Jack.Drive.Robot;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;

//TODO: Test thissss
public class TurretServoV1 {
    public Servo servo;
    public double kP = RobotConstantsV1.turretPIDs.p;
    public AnalogInput encoder;
    public double errorDeg, correction, next = 0;
    public double pos;
    public boolean useEncoder = true;

    public void init(HardwareMap hardwareMap, boolean useEncoder){
        servo = hardwareMap.get(Servo.class, RobotConstantsV1.turretServoName);
        if(useEncoder) {
            this.useEncoder = true;
            encoder = hardwareMap.get(AnalogInput.class, "turretEncoder");
        }
    }

    public double update(double targetDeg) {
        if (!useEncoder) {
            return servo.getPosition();
        }

        double currentDeg = getEncoderPos();
        errorDeg = targetDeg - currentDeg;

        correction = (errorDeg / 360.0) * (-RobotConstantsV1.turretPIDs.p);

        next = pos + correction;
        next = Math.max(0, Math.min(1, next));

        setPosition(next);
        return next;
    }

    public void setPosition(double position){
        this.pos = position;
        servo.setPosition((1.0 - position));
    }

    public boolean isDoneTurning(double targetDeg) {
        if (!useEncoder) {
            return true;
        }
        return Math.abs(getEncoderPos() - targetDeg) <= 3;
    }

    public double getPosition(){
        return pos;
    }

    public double getEncoderPos(){
        return (encoder.getVoltage() / encoder.getMaxVoltage()) * 360.0;
    }

}
