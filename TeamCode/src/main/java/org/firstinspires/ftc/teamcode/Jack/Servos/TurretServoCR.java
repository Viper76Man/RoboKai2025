package org.firstinspires.ftc.teamcode.Jack.Servos;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;

public class TurretServoCR {
    public CRServo turret;
    public double target = 0;
    public double current = 0;
    public double error = 0;
    public boolean usePower = true;
    public AnalogInput encoder;

    public void init(HardwareMap hardwareMap){
        turret = hardwareMap.get(CRServo.class, RobotConstantsV1.turretServoName);
        turret.setPower(0);
        encoder = hardwareMap.get(AnalogInput.class, "turretEncoder");
    }

    public void setPower(double power){
        turret.setPower(power);
        turret.setDirection(DcMotorSimple.Direction.FORWARD);
        this.usePower = true;
    }

    public void update(){
        if(!usePower) {
            current = getEncoderPos();
            error = current - target;
            turret.setPower(error * RobotConstantsV1.turretPIDs.p);
        }
    }


    public void setTargetPos(double pos){
        this.target = pos;
        this.usePower = false;
    }

    public double getEncoderPos(){
        return (encoder.getVoltage() / encoder.getMaxVoltage()) * 360.0;
    }
}
