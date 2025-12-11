package org.firstinspires.ftc.teamcode.Jack.Drive;

import android.app.slice.SliceMetrics;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.modernrobotics.comm.ModernRoboticsDatagram;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.code.Attribute;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Jack.Camera.Limelight3A.LimelightV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.ArcShooterV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.IntakeV1;
import org.firstinspires.ftc.teamcode.Jack.Odometry.BlueAutoPathsV1;
import org.firstinspires.ftc.teamcode.Jack.Odometry.Constants;
import org.firstinspires.ftc.teamcode.Jack.Odometry.DecodeFieldLocalizer;
import org.firstinspires.ftc.teamcode.Jack.Odometry.PinpointV1;
import org.firstinspires.ftc.teamcode.Jack.Odometry.RedAutoPathsV1;
import org.firstinspires.ftc.teamcode.Jack.Other.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.Jack.Other.ObeliskPattern;
import org.firstinspires.ftc.teamcode.Jack.Servos.StorageServoV1;

public class Robot {

    public PinpointV1 pinpoint = new PinpointV1();
    public ArcShooterV1 shooter = new ArcShooterV1();
    public MecanumDriveOnly drive = new MecanumDriveOnly();

    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public DecodeFieldLocalizer localizer = new DecodeFieldLocalizer();
    public Mode mode;
    public GamepadV1 gamepad = new GamepadV1();
    public MultipleTelemetry multipleTelemetry;
    public IntakeV1 intake = new IntakeV1();
    public LimelightV1 limelight = new LimelightV1();
    public StorageServoV1 storage = new StorageServoV1();
    public ElapsedTime lockTimer = new ElapsedTime();
    public Alliance alliance = Alliance.TEST;
    public boolean lockOn = false;
    public Follower follower;

    public ObeliskPattern pattern;

    public enum Mode {
        TELEOP,
        AUTONOMOUS
    }

    public enum Alliance {
        BLUE,
        RED,
        TEST
    }

    public void init(Mode mode, Alliance alliance, HardwareMap hardwareMap, MultipleTelemetry multipleTelemetry, Gamepad gamepad1){
        this.mode = mode;
        this.hardwareMap = hardwareMap;
        this.telemetry = multipleTelemetry.telemetry;
        this.alliance = alliance;
        this.multipleTelemetry = multipleTelemetry;
        //TODO: Debug gamepad
        this.gamepad.init(gamepad1, 0.3);
        follower = Constants.createFollower(hardwareMap);
        shooter.init(hardwareMap);
        drive.init(hardwareMap, gamepad1);
        intake.init(hardwareMap);
        limelight.init(hardwareMap, telemetry);
        if(mode == Mode.AUTONOMOUS) {
            setCameraPipeline(LimelightV1.Pipeline.OBELISK);
        }
        else {
            switch (alliance){
                case RED:
                    setCameraPipeline(LimelightV1.Pipeline.RED_GOAL);
                    //TODO: Remove before comps
                    follower.setStartingPose(RedAutoPathsV1.startPose);
                    break;
                case BLUE:
                    setCameraPipeline(LimelightV1.Pipeline.BLUE_GOAL);
                    //TODO: Remove before comps
                    follower.setStartingPose(BlueAutoPathsV1.startPose);
                    break;
            }
        }
    }

    public void systemStatesUpdate(){
        follower.update();
        localizer.drawToPanels(follower);
        arcUpdate();
        runIntake();
        multipleTelemetry.update();
        if(mode == Mode.TELEOP) {
            gamepad.update();
            if (gamepad.triangle && gamepad.isGamepadReady()) {
                switchIntakeDirection();
                gamepad.resetTimer();
            }
            if(gamepad.circle && gamepad.isGamepadReady()){
                lockOn = true;
                lockTimer.reset();
                gamepad.resetTimer();
            }
            if(lockTimer.seconds() > 2) {
                lockOn = false;
            }
            if(lockOn && limelight.getLatestAprilTagResult() == null) {
                drive.driveWithRotationLock(alliance, follower.getPose(), telemetry, false);
            }
            else if(lockOn && limelight.getLatestAprilTagResult() != null){
                drive.driveWithRotationLock(alliance, follower.getPose(), telemetry, true);
                multipleTelemetry.addLine("wat??????????????????????????????????");
            }
            else if (!lockOn) {
                drive();
            }
        }
    }

    public void shootArtifact(int artifact){
        runShooterActive();
    }

    public void switchIntakeDirection(){
        intake.switchDirection();
    }
    public void runIntake(){
        intake.setDirection(RobotConstantsV1.intakeDirection);
        intake.setPower(RobotConstantsV1.INTAKE_POWER);
    }

    public void setTargetObeliskPattern(ObeliskPattern pattern) {
        this.pattern = pattern;
    }

    public void idToPattern(int id) {
    }

    public void runIntakeReverse(){
        if(RobotConstantsV1.intakeDirection == DcMotorSimple.Direction.REVERSE){
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        else {
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        intake.setPower(RobotConstantsV1.INTAKE_POWER);
    }

    public void runShooterIdle(){
        shooter.setTargetRPM(RobotConstantsV1.SHOOTER_IDLE_RPM);
    }

    public void runShooterActive(){
        shooter.setTargetRPM(RobotConstantsV1.SHOOTER_TARGET_RPM);
    }

    public void arcUpdate(){
        if(Math.abs(getDistanceFromLaunchZone()) < RobotConstantsV1.maxLaunchZoneDistance + 12){
            runShooterActive();
        }
        else {
            runShooterIdle();
        }
    }

    public void drive(){
        drive.drive();
    }

    public double getDistanceFromLaunchZone(){
        return localizer.getDistanceFromLaunchZone(follower.getPose());
    }

    public boolean inLaunchZone(){
        return localizer.isRobotInBackLaunchZone(follower.getPose());
    }

    public void setCameraPipeline(LimelightV1.Pipeline pipeline){
        limelight.setPipeline(pipeline);
        limelight.startStreaming();
    }

    public boolean isObeliskResultValid(){
        if(limelight.getLatestAprilTagResult() != null) {
            if (limelight.getLatestAprilTagResult().getFiducialId() < 21 || limelight.getLatestAprilTagResult().getFiducialId() > 23) {
                limelight.setPipeline(LimelightV1.Pipeline.OBELISK);
            }
            return limelight.getLatestResult().isValid() && limelight.getLatestAprilTagResult() != null;
        }
        else {
            return false;
        }
    }
}
