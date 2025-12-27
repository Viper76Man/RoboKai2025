package org.firstinspires.ftc.teamcode.Jack.Servos;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Jack.Camera.Limelight3A.LimelightV1;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;

//TODO: Test thissss
public class TurretServoV1 {
    public Servo servo;
    public double kP = RobotConstantsV1.turretPIDs.p;
    public AnalogInput encoder;
    public double pos;
    public boolean useEncoder = false;

    public void init(HardwareMap hardwareMap, boolean useEncoder){
        servo = hardwareMap.get(Servo.class, RobotConstantsV1.turretServoName);
        if(useEncoder) {
            this.useEncoder = true;
            encoder = hardwareMap.get(AnalogInput.class, "turretEncoder");
        }
    }

    public void update(double cameraError){
        pos = pos - (cameraError * RobotConstantsV1.turretServoPower);
        servo.setPosition(pos);
    }

    public double update(int targetEncoderPos){
        if(useEncoder) {
            double current = (encoder.getVoltage() / encoder.getMaxVoltage()) * 360.0;
            double next = servo.getPosition() - ((current - targetEncoderPos) * kP);
            if(next > 1){
                next = next - 1;
            }
            else if(next < 0){
                next = next + 1;
            }
            setPosition(next);
            return next;
        }
        return -1;
    }

    public void setPosition(double position){
        this.pos = position;
        servo.setPosition(position);
    }

    public boolean isDoneTurning(){
        if(useEncoder) {
            return (encoder.getVoltage() / encoder.getMaxVoltage()) == pos;
        }
        else {
            return true;
        }
    }

    public double getPosition(){
        return servo.getPosition();
    }

}
