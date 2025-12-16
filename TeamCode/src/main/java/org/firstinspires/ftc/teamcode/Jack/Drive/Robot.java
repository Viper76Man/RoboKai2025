package org.firstinspires.ftc.teamcode.Jack.Drive;

import android.app.slice.SliceMetrics;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
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
import org.firstinspires.ftc.teamcode.Jack.Other.Drawing;
import org.firstinspires.ftc.teamcode.Jack.Other.LoggerV1;
import org.firstinspires.ftc.teamcode.Jack.Other.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.Jack.Other.ObeliskPattern;
import org.firstinspires.ftc.teamcode.Jack.Servos.FlickerServoV1;
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
    public FlickerServoV1 flicker1 = new FlickerServoV1();
    public LimelightV1 limelight = new LimelightV1();
    public LoggerV1 loggerV1 = new LoggerV1();
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

    public void init(Mode mode, Alliance alliance, HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1){
        this.mode = mode;
        this.hardwareMap = hardwareMap;
        this.telemetry = multipleTelemetry.telemetry;
        this.alliance = alliance;
        if(RobotConstantsV1.panelsEnabled) {
            this.multipleTelemetry = new MultipleTelemetry(telemetry, PanelsTelemetry.INSTANCE.getTelemetry());
            Drawing.init();
        }
        this.gamepad.init(gamepad1, 0.3);
        follower = Constants.createFollower(hardwareMap);
        shooter.init(hardwareMap);
        drive.init(hardwareMap, gamepad1);
        intake.init(hardwareMap);
        flicker1.init(hardwareMap, RobotConstantsV1.flickerServoName);
        limelight.init(hardwareMap, telemetry);
        if(mode == Mode.AUTONOMOUS) {
            setCameraPipeline(LimelightV1.Pipeline.OBELISK);
            loggerV1.saveSideToFile(alliance);

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
    public void updatePanelsFieldRotation(){
        if(RobotConstantsV1.panelsEnabled){
            Drawing.setFieldRotation(RobotConstantsV1.panelsFieldRotation);
        }
    }

    public void systemStatesUpdate(){
        follower.update();
        localizer.drawToPanels(follower);
        if(RobotConstantsV1.panelsEnabled){
            Drawing.drawRobot(follower.getPose());
            Drawing.drawPoseHistory(follower.getPoseHistory());
        }
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
            //TODO:Code function to do this to selected ball slot
            if(gamepad.gamepad.right_trigger >= 0.2 && readyToFire()){
                shootBall(getNextBall());
            }
            if(gamepad.right_trigger <= 0.2 && flicker1.timer.seconds() > RobotConstantsV1.ALL_FLICKER_DOWN_DELAY_SECONDS) {
                flickerDown(1);
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


    public void flickerUp(int flicker){
        switch (flicker){
            case 1:
                flicker1.setPosition(RobotConstantsV1.FLICKER_SERVO_UP);
                break;
            case 2:
                break;
        }
    }

    public void flickerDown(int flicker){
        switch (flicker){
            case 1:
                flicker1.setPosition(RobotConstantsV1.FLICKER_SERVO_DOWN);
                break;
            case 2:
                break;
        }
    }

    public void shootBall(int ball){
        if(ball > 3 || ball < 1){
            return;
        }
        switch(ball){
            case 1:
                flicker1.resetTimer();
            case 2:
                break;
        }
        flickerUp(ball);
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

    public int getNextBall(){
        return 1;
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

    public boolean readyToFire(){
       return getDistanceFromBackLaunchZone() < 20 && gamepad.isGamepadReady();
    }

    public void arcUpdate(){
        if(Math.abs(getDistanceFromBackLaunchZone()) < RobotConstantsV1.maxLaunchZoneDistance + 12){
            runShooterActive();
        }
        else {
            runShooterIdle();
        }
    }

    public void drive(){
        drive.drive();
    }

    public double getDistanceFromBackLaunchZone(){
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
