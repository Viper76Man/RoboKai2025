package org.firstinspires.ftc.teamcode.Jack.Drive;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Jack.Camera.Limelight3A.LimelightV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.ArcShooterV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.IntakeV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.SpindexerMotorV1;
import org.firstinspires.ftc.teamcode.Jack.Odometry.Constants;
import org.firstinspires.ftc.teamcode.Jack.Odometry.DecodeFieldLocalizer;
import org.firstinspires.ftc.teamcode.Jack.Other.CustomPath;
import org.firstinspires.ftc.teamcode.Jack.Other.Drawing;
import org.firstinspires.ftc.teamcode.Jack.Other.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.Jack.Servos.FlickerServoV1;

public class RobotV2 {
    //HARDWARE--------------------------------------------------------------------------------------
    public ArcShooterV1 arcShooter = new ArcShooterV1();
    public SpindexerMotorV1 spindexer = new SpindexerMotorV1();
    public IntakeV1 intake = new IntakeV1();
    public LimelightV1 limelight = new LimelightV1();
    public Follower follower;
    public GamepadV1 gamepadV1 = new GamepadV1();
    public MecanumDriveOnly drive = new MecanumDriveOnly();
    public FlickerServoV1 flicker = new FlickerServoV1();
    //VARIABLES-------------------------------------------------------------------------------------
    public enum Mode {
        TELEOP,
        AUTONOMOUS
    }
    public boolean usePinpointForTeleOp = true;
    public boolean shooterOn = true;
    public Robot.Alliance alliance;
    //OTHER-----------------------------------------------------------------------------------------
    public DecodeFieldLocalizer localizer = new DecodeFieldLocalizer();
    //----------------------------------------------------------------------------------------------
    public void init(Mode mode,  Robot.Alliance alliance,  HardwareMap hardwareMap, GamepadV1 gamepadV1, Telemetry telemetry){
        //INITIALIZATION----------------------------------------------------------------------------
        initHardware(hardwareMap, gamepadV1);
        intake.setDirection(RobotConstantsV1.intakeDirection);
        follower = Constants.createFollower(hardwareMap);
        //INIT-STUFF--------------------------------------------------------------------------------
        switch (mode){
            case AUTONOMOUS:
            case TELEOP:
                break;
        }
    }

    public void initHardware(HardwareMap hardwareMap, GamepadV1 gamepadV1){
        arcShooter.init(hardwareMap);
        intake.init(hardwareMap);
        drive.init(hardwareMap, gamepadV1);
        flicker.init(hardwareMap, RobotConstantsV1.flickerServoName);
        limelight.init(hardwareMap);
        this.gamepadV1 = gamepadV1;
    }

    //INTAKE----------------------------------------------------------------------------------------
    public void intakeRun(){
        if(activateIntake()) {
            intake.setPower(RobotConstantsV1.INTAKE_POWER);
        }
    }

    public void intakeSwitchDirection(){
        intake.switchDirection();
    }
    //SHOOTER---------------------------------------------------------------------------------------
    public void shooterSwitchDirection(){
        arcShooter.switchDirection();
    }
    public void setShooterTargetRPM(int rpm){
        arcShooter.setTargetRPM(rpm);
    }
    public void shooterRun(){
        if(activateShooter()) {
            arcShooter.run();
        }
    }
    //FUNCTIONS-------------------------------------------------------------------------------------
    public double getDistanceFromBackLaunchZone(){
        return Math.abs(localizer.getDistanceFromLaunchZone(follower.getPose()));
    }
    public boolean activateIntake(){
        return !activateShooter();
    }

    public boolean activateShooter(){
        if(usePinpointForTeleOp) {
            return getDistanceFromBackLaunchZone() < RobotConstantsV1.maxLaunchZoneArcShooterDistance;
        }
        else {
            return shooterOn;
        }
    }

    public void toggleShooterNoPinpoint(){
        shooterOn = !shooterOn;
    }

    public void shootBall(int ball){
        if(!spindexer.isReady()) {
            switch (ball) {
                case 1:
                    spindexer.setState(SpindexerMotorV1.State.BALL_1_SHOOT);
                    break;
                case 2:
                    spindexer.setState(SpindexerMotorV1.State.BALL_2_SHOOT);
                    break;
                case 3:
                    spindexer.setState(SpindexerMotorV1.State.BALL_3_SHOOT);
                    break;
            }
        }
        else {
            if(flicker.timer.seconds() < 2) {
                flicker.setPosition(RobotConstantsV1.FLICKER_SERVO_UP);
            }
        }
    }

    public int getNextBall(){
        return 1;
    }

    public void drawToPanels(){
        Drawing.drawRobot(follower.getPose());
        Drawing.drawPoseHistory(follower.getPoseHistory());
        Drawing.sendPacket();
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

    public boolean inLaunchZone(){
        return localizer.isRobotInBackLaunchZone(follower.getPose());
    }
    //----------------------------------------------------------------------------------------------
    public void systemStatesUpdate(){
        shooterRun();
        intakeRun();
        gamepadV1.update();
        if(usePinpointForTeleOp){
            if(gamepadV1.isGamepadReady() && gamepadV1.options){
                toggleShooterNoPinpoint();
                gamepadV1.resetTimer();
            }
            else if(gamepadV1.buttonTimer.seconds() > 0.2 && gamepadV1.right_trigger >= 0.15){
                shootBall(getNextBall());
                gamepadV1.resetTimer();
            }
        }
    }

    public void followPath(Path path){
        follower.followPath(path);
        follower.update();
    }

    public void followPath(CustomPath path){

    }

    public void log(Telemetry telemetry){
        MultipleTelemetry telemetry1 = new MultipleTelemetry(telemetry, PanelsTelemetry.INSTANCE.getTelemetry());
        telemetry1.addLine("Alliance: " + alliance.name());
        telemetry1.update();
    }
}
