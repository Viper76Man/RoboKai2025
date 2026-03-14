package org.firstinspires.ftc.teamcode.Jack.Servos;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.internal.hardware.android.GpioPin;
import org.firstinspires.ftc.teamcode.Jack.Camera.Limelight3A.LimelightV1;
import org.firstinspires.ftc.teamcode.Jack.Drive.Robot;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.Jack.Odometry.Constants;
import org.firstinspires.ftc.teamcode.Jack.Odometry.DecodeFieldLocalizer;
import org.firstinspires.ftc.teamcode.Jack.Odometry.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Jack.Odometry.PinpointV1;

import java.util.Objects;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.ftc.ActiveOpMode;

public class TurretServoCR {
    public CRServo turret;
    public ControlSystem controller, controllerHeading;
    public PIDCoefficients coefficients, coefficients2;
    public double target = 0;
    public double current = 0;
    public double error = 0;
    public ElapsedTime noResultTimer = new ElapsedTime();
    public Position newPose;
    public double heading;
    public ElapsedTime relocalizeTimer = new ElapsedTime();
    public boolean usePower = true;
    public DecodeFieldLocalizer localizer = new DecodeFieldLocalizer();
    public TelemetryManager telemetryM  = PanelsTelemetry.INSTANCE.getTelemetry();
    public AnalogInput encoder;
    public double power_ = 0;

    public double cameraTx, latestTagID;

    public Follower follower;
    public Robot.Mode mode;

    public void init(HardwareMap hardwareMap, Robot.Mode mode){
        turret = hardwareMap.get(CRServo.class, RobotConstantsV1.turretServoName);
        turret.setPower(0);
        this.mode = mode;
        coefficients = new PIDCoefficients(RobotConstantsV1.turretPIDs.p, RobotConstantsV1.turretPIDs.i, RobotConstantsV1.turretPIDs.d);
        controller = ControlSystem.builder().posSquID(coefficients).build();
        coefficients2 = new PIDCoefficients(RobotConstantsV1.turretPIDsHeading.p, RobotConstantsV1.turretPIDsHeading.i, RobotConstantsV1.turretPIDsHeading.d);
        controllerHeading = ControlSystem.builder().posSquID(coefficients2).build();
        controllerHeading.setGoal(new KineticState(Math.toDegrees(localizer.blueGoalCenter.getHeading())));
        encoder = hardwareMap.get(AnalogInput.class, "turretEncoder");
        relocalizeTimer.reset();
    }

    public void setPower(double power){
        turret.setPower(power);
        turret.setDirection(DcMotorSimple.Direction.FORWARD);
        this.usePower = true;
    }

    public void run(LimelightV1 limelight, double TURRET_OFFSET_ANGLE, Robot.Alliance alliance){
        double power = 0;
        LLResultTypes.FiducialResult latest_result = limelight.getLatestAprilTagResult();
        if(latest_result != null && limelight.limelight.getLatestResult().isValid()) {
            latestTagID = latest_result.getFiducialId();
            cameraTx = latest_result.getTargetXDegrees();
            double error = (cameraTx + TURRET_OFFSET_ANGLE);
            if(Math.abs(error) < RobotConstantsV1.degreeToleranceCamera) {
                power = error * RobotConstantsV1.turretSlowPower;
                controller.calculate(new KineticState(error));
            }
            else {
                power = controller.calculate(new KineticState(error));
            }
            power += (RobotConstantsV1.turretKF * Math.signum(error));
            noResultTimer.reset();
        }
        else {
            if(noResultTimer.seconds() > 3) {
                cameraTx = 0;
                double err = 236 - getEncoderPos(); // e.g., 236 - current
                power = err * RobotConstantsV1.turretServoPower;
            }
        }
        if(getEncoderPos() >= RobotConstantsV1.TURRET_MAX_ENCODER_VALUE && power > 0){
            power = 0;
        }
        if(getEncoderPos() <= 120 && power < 0){
            power = 0;
        }

        //if(noResultTimer.seconds() > 1){
        //turret.setPower(0);

        //}
        power = Math.max(-0.6, Math.min(0.6, power));
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
    public double normalizeAngle(double angle) {
        angle %= 360;
        if (angle > 180) {
            angle -= 360;
        }
        if (angle < -180) {
            angle += 360;
        }
        return angle;
    }
    public double normalizeAngle2(double angle) {
        angle %= 360;
        if (angle > 360) {
            angle -= 360;
        }
        if (angle < -360) {
            angle += 360;
        }
        return angle;
    }
    public double getTurretAngle() {
        return getEncoderPos() - 236;
    }

    public PIDCoefficients getPIDCoefficients(){
        return coefficients;
    }
}
