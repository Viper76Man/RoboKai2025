package org.firstinspires.ftc.teamcode.Jack.Servos;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Jack.Camera.Limelight3A.LimelightV1;
import org.firstinspires.ftc.teamcode.Jack.Drive.Robot;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;

public class TurretServoCR {
    public CRServo turret;
    public ControlSystem controller;
    public PIDCoefficients coefficients;
    public double target = 0;
    public double current = 0;
    public double error = 0;
    public ElapsedTime noResultTimer = new ElapsedTime();
    public boolean usePower = true;
    public AnalogInput encoder;
    public double power_ = 0;

    public double cameraTx, latestTagID;

    public void init(HardwareMap hardwareMap){
        turret = hardwareMap.get(CRServo.class, RobotConstantsV1.turretServoName);
        turret.setPower(0);
        coefficients = new PIDCoefficients(RobotConstantsV1.turretPIDs.p, RobotConstantsV1.turretPIDs.i, RobotConstantsV1.turretPIDs.d);
        controller = ControlSystem.builder().posPid(coefficients).build();
        controller.setGoal(new KineticState(0));
        encoder = hardwareMap.get(AnalogInput.class, "turretEncoder");
    }

    public void setPower(double power){
        turret.setPower(power);
        turret.setDirection(DcMotorSimple.Direction.FORWARD);
        this.usePower = true;
    }

    public void run(LimelightV1 limelight, double TURRET_OFFSET_ANGLE){
        double power = 0;

        LLResultTypes.FiducialResult latest_result = limelight.getLatestAprilTagResult();
        if(latest_result != null && limelight.limelight.getLatestResult().isValid()) {
            latestTagID = latest_result.getFiducialId();
            cameraTx = latest_result.getTargetYDegrees();
            double error = (cameraTx + TURRET_OFFSET_ANGLE);
            if(Math.abs((cameraTx + TURRET_OFFSET_ANGLE)) < RobotConstantsV1.degreeToleranceCamera) {
                power = error * RobotConstantsV1.turretSlowPower;
                controller.calculate(new KineticState(error));
            }
            else {
                power = controller.calculate(new KineticState(error));
            }
            noResultTimer.reset();
        }
        else {
            if(noResultTimer.seconds() > 1.2) {
                cameraTx = 0;
                double err = 236 - getEncoderPos(); // e.g., 236 - current
                power = err * RobotConstantsV1.turretServoPower;
            }
        }
        if(getEncoderPos() >= RobotConstantsV1.TURRET_MAX_ENCODER_VALUE && power > 0){
            power = 0;
        }

        //if(noResultTimer.seconds() > 1){
        //turret.setPower(0);

        //}
        power = Math.max(-0.5, Math.min(0.5, power));
        power_ = power;
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

    public PIDCoefficients getPIDCoefficients(){
        return coefficients;
    }
}
