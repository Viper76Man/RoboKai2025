package org.firstinspires.ftc.teamcode.Jack.Servos;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.Jack.Camera.Limelight3A.LimelightV1;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.PIDController;

public class TurretServoCR {
    public CRServo turret;
    public PIDController controller;
    public double target = 0;
    public double current = 0;
    public double error = 0;
    public boolean usePower = true;
    public AnalogInput encoder;

    public double cameraTx, latestTagID;

    public void init(HardwareMap hardwareMap){
        turret = hardwareMap.get(CRServo.class, RobotConstantsV1.turretServoName);
        turret.setPower(0);
        controller = new PIDController(RobotConstantsV1.turretPIDs.p, RobotConstantsV1.turretPIDs.i, RobotConstantsV1.turretPIDs.d);
        encoder = hardwareMap.get(AnalogInput.class, "turretEncoder");
    }

    public void setPower(double power){
        turret.setPower(power);
        turret.setDirection(DcMotorSimple.Direction.FORWARD);
        this.usePower = true;
    }

    public void run(LimelightV1 limelight, double TURRET_OFFSET_ANGLE, double modifier){
        double power;
        LLResultTypes.FiducialResult latest_result = limelight.getLatestAprilTagResult();
        if(latest_result != null) {
            latestTagID = latest_result.getFiducialId();
            cameraTx = latest_result.getTargetXDegreesNoCrosshair();
            power = modifier * controller.getOutput(cameraTx + TURRET_OFFSET_ANGLE);
        }
        else {
            cameraTx = 0;
            power = modifier * controller.getOutput((int)getEncoderPos(), 236);
        }
        if(getEncoderPos() >= RobotConstantsV1.TURRET_MAX_ENCODER_VALUE && power < 0){
            power = 0;
        }
        if(Math.abs((cameraTx + TURRET_OFFSET_ANGLE)) < RobotConstantsV1.degreeToleranceCamera){
            power = power / 2;
        }
        //if(noResultTimer.seconds() > 1){
        //turret.setPower(0);

        //}

        controller.updatePIDsFromConstants(RobotConstantsV1.turretPIDs);
        turret.setPower(power);
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
