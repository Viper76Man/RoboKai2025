package org.firstinspires.ftc.teamcode.Jack.Drive;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Jack.Camera.Limelight3A.LimelightV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.ArcShooterV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.IntakeV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.SpindexerMotorV1;
import org.firstinspires.ftc.teamcode.Jack.Odometry.Constants;
import org.firstinspires.ftc.teamcode.Jack.Odometry.CustomFollower;
import org.firstinspires.ftc.teamcode.Jack.Odometry.DecodeFieldLocalizer;
import org.firstinspires.ftc.teamcode.Jack.Odometry.CustomPath;
import org.firstinspires.ftc.teamcode.Jack.Other.ArtifactColor;
import org.firstinspires.ftc.teamcode.Jack.Other.ArtifactSlot;
import org.firstinspires.ftc.teamcode.Jack.Other.Drawing;
import org.firstinspires.ftc.teamcode.Jack.Other.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.Jack.Other.SlotColorSensorV1;
import org.firstinspires.ftc.teamcode.Jack.Servos.FlickerServoV1;

public class RobotV2 {
    //HARDWARE--------------------------------------------------------------------------------------
    public ArcShooterV1 arcShooter = new ArcShooterV1();
    public SpindexerMotorV1 spindexer = new SpindexerMotorV1();
    public IntakeV1 intake = new IntakeV1();
    public LimelightV1 limelight = new LimelightV1();
    public Follower follower;
    public GamepadV1 gamepad = new GamepadV1();
    public MecanumDriveOnly drive = new MecanumDriveOnly();
    public FlickerServoV1 flicker = new FlickerServoV1();
    public CustomFollower customFollower;


    public ElapsedTime timer = new ElapsedTime();
    public SlotColorSensorV1 sensor = new SlotColorSensorV1();

    public ArtifactSlot slot1 = new ArtifactSlot();
    public ArtifactSlot slot2 = new ArtifactSlot();
    public ArtifactSlot slot3 = new ArtifactSlot();

    public ElapsedTime shootTimer = new ElapsedTime();

    //VARIABLES-------------------------------------------------------------------------------------
    public enum Mode {
        TELEOP,
        AUTONOMOUS
    }

    public enum State {
        SHOOT,
        INTAKE
    }

    public int currentBall = 1;
    public boolean usePinpointForTeleOp = true;
    public boolean shooterOn = true;
    public boolean flickerUp = true;
    public Robot.Alliance alliance;
    public State state = State.INTAKE;
    //OTHER-----------------------------------------------------------------------------------------
    public DecodeFieldLocalizer localizer = new DecodeFieldLocalizer();
    //----------------------------------------------------------------------------------------------
    public void init(Mode mode,  Robot.Alliance alliance,  HardwareMap hardwareMap, GamepadV1 gamepad, Telemetry telemetry){
        //INITIALIZATION----------------------------------------------------------------------------
        initHardware(hardwareMap, gamepad);
        initArtifactSlots();
        sensor.init(hardwareMap, RobotConstantsV1.colorSensor1);
        intake.setDirection(RobotConstantsV1.intakeDirection);
        follower = Constants.createFollower(hardwareMap);
        customFollower = new CustomFollower(hardwareMap);
        //INIT-STUFF--------------------------------------------------------------------------------
        switch (mode){
            case AUTONOMOUS:
            case TELEOP:
                break;
        }
    }

    public void start(){
        runSpindexerToIntakeBall(1);
        timer.reset();
    }

    public void initHardware(HardwareMap hardwareMap, GamepadV1 gamepad){
        arcShooter.init(hardwareMap, RobotConstantsV1.arcPIDs);
        intake.init(hardwareMap);
        drive.init(hardwareMap, gamepad);
        flicker.init(hardwareMap, RobotConstantsV1.flickerServoName);
        limelight.init(hardwareMap);
        spindexer.init(hardwareMap, RobotConstantsV1.spindexerPIDs);
        spindexer.setTargetPos(RobotConstantsV1.SPINDEXER_MOTOR_BALL_1_INTAKE, SpindexerMotorV1.EncoderMeasurementMethod.MOTOR);
        this.gamepad = gamepad;
    }

    //INTAKE----------------------------------------------------------------------------------------
    public void intakeRun(){
        if(state == State.INTAKE) {
            intake.setPower(RobotConstantsV1.INTAKE_POWER);
        }
        else {
            intake.setPower(0.4);
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
        if(state == State.INTAKE && shootTimer.seconds() < 1.2){
            arcShooter.setTargetRPM(RobotConstantsV1.SHOOTER_TARGET_RPM);
            arcShooter.run();
        }
        if(state == State.SHOOT) {
            gamepad.update();
            arcShooter.setTargetRPM(RobotConstantsV1.SHOOTER_TARGET_RPM);
            arcShooter.run();
            if(gamepad.dpad_up && gamepad.isGamepadReady()){
                arcShooter.setTargetRPM(arcShooter.getTargetRPM() + RobotConstantsV1.velocityUpStep);
                gamepad.resetTimer();
            }
            else if(gamepad.dpad_down && gamepad.isGamepadReady()) {
                arcShooter.setTargetRPM(arcShooter.getTargetRPM() - RobotConstantsV1.velocityDownStep);
                gamepad.resetTimer();
            }
        }
        else {
            if(shootTimer.seconds() > 1.2) {
                arcShooter.setTargetRPM(500);
                arcShooter.run();
            }
        }
    }
    //FUNCTIONS-------------------------------------------------------------------------------------
    public double getDistanceFromBackLaunchZone(){
        return Math.abs(localizer.getDistanceFromLaunchZone(follower.getPose()));
    }
    public boolean activateIntake(){
        return !activateShooter();
    }

    public void initArtifactSlots(){
        setEmpty(1);
        setEmpty(2);
        setEmpty(3);
    }

    public void colorSensorUpdate(){
        if(state == State.INTAKE) {
            if(!spindexer.isSpindexerReady()){
                sensor.clear();
            }
            else {
                sensor.update(spindexer.state, spindexer.isSpindexerReady());
                if (sensor.getCurrent() == ArtifactColor.GREEN && spindexer.isSpindexerReady()) {
                    setGreen(currentBall);
                    spinRight();
                    sensor.clear();

                } else if (sensor.getCurrent() == ArtifactColor.PURPLE && spindexer.isSpindexerReady()) {
                    setPurple(currentBall);
                    spinRight();
                    sensor.clear();
                }
            }
        }
    }

    public void flickerUpdate(){
        if(state == State.SHOOT) {
            if (flicker.timer.seconds() < RobotConstantsV1.FLICKER_UP_TIME && flicker.getState() == FlickerServoV1.State.DOWN) {
                flicker.setPosition(FlickerServoV1.State.UP);
            }
            if (flicker.timer.seconds() > RobotConstantsV1.FLICKER_UP_TIME) {
                flicker.setPosition(FlickerServoV1.State.DOWN);
            }
            if (flickerUp && flicker.timer.seconds() < 10 && flicker.timer.seconds() > RobotConstantsV1.FLICKER_UP_TIME + 0.1) {
                spinRight();
                flickerUp = false;
            }
        }
    }

    public void setFlickerUp(){
        flicker.resetTimer();
    }

    public boolean activateShooter(){
        if(usePinpointForTeleOp) {
            return getDistanceFromBackLaunchZone() < RobotConstantsV1.maxLaunchZoneArcShooterDistance;
        }
        else {
            return shooterOn;
        }
    }

    public void spindexerRun(){
        switch (state) {
            case SHOOT:
                switch (currentBall) {
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
                break;
            case INTAKE:
                switch (currentBall) {
                    case 1:
                        spindexer.setState(SpindexerMotorV1.State.BALL_1_INTAKE);
                        break;
                    case 2:
                        spindexer.setState(SpindexerMotorV1.State.BALL_2_INTAKE);
                        break;
                    case 3:
                        spindexer.setState(SpindexerMotorV1.State.BALL_3_INTAKE);
                        break;
                }
                break;
        }
        spindexer.run();
    }

    public void toggleShooterNoPinpoint(){
        shooterOn = !shooterOn;
    }

    //TODO: Remember button presses maybe?
    public void shootBall(int ball) {
        shootTimer.reset();
        if(spindexer.isSpindexerReady()){
            if(flicker.getState() != FlickerServoV1.State.UP) {
                setFlickerUp();
                setEmpty(ball);
            }
        }
    }
    public void spinLeft(){
        currentBall = currentBall - 1;
        if(currentBall < 1){
            currentBall = 3;
        }
        if(state == State.INTAKE){
            runSpindexerToIntakeBall(currentBall);
        }
    }

    public void spinRight(){
        currentBall = currentBall + 1;
        if(currentBall > 3 && state == State.INTAKE) {
            state = State.SHOOT;
            currentBall = 1;
        }
        else if(currentBall > 3 && state == State.SHOOT) {
            state = State.INTAKE;
            currentBall = 1;
        }
    }

    public int getNextBall(){
        int next = currentBall + 1;
        if(next > 3){
            next = 1;
        }
        return next;
    }

    public int getPreviousBall(){
        int next = currentBall - 1;
        if(next < 1){
            next = 3;
        }
        return next;
    }

    public void setEmpty(int ball){
        switch (ball){
            case 1:
                slot1.setColor(ArtifactColor.NONE);
                break;
            case 2:
                slot2.setColor(ArtifactColor.NONE);
                break;
            case 3:
                slot3.setColor(ArtifactColor.NONE);
                break;
        }
    }

    public void setGreen(int ball){
        switch (ball){
            case 1:
                slot1.setColor(ArtifactColor.GREEN);
                break;
            case 2:
                slot2.setColor(ArtifactColor.GREEN);
                break;
            case 3:
                slot3.setColor(ArtifactColor.GREEN);
                break;
        }
    }

    public void runSpindexerToIntakeBall(int ball){
        currentBall = ball;
    }

    public void runSpindexerToShootBall(int ball){
        currentBall = ball;
    }

    public void setPurple(int ball){
        switch (ball){
            case 1:
                slot1.setColor(ArtifactColor.PURPLE);
                break;
            case 2:
                slot2.setColor(ArtifactColor.PURPLE);
                break;
            case 3:
                slot3.setColor(ArtifactColor.PURPLE);
                break;
        }
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
        flickerUpdate();
        if(timer.seconds() > 0.8) {
            colorSensorUpdate();
        }
        spindexerRun();
        drive.drive();
        gamepad.update();
        if(gamepad.isGamepadReady() && gamepad.circle){
            currentBall = getNextBall();
            gamepad.resetTimer();
        }
        if(gamepad.isGamepadReady() && gamepad.triangle){
            currentBall = getPreviousBall();
            gamepad.resetTimer();
        }
        if(slot1.getColor() != ArtifactColor.NONE && slot2.getColor() != ArtifactColor.NONE && slot3.getColor() != ArtifactColor.NONE){
            state = State.SHOOT;
        }
        else if(slot1.getColor() == ArtifactColor.NONE && slot2.getColor() == ArtifactColor.NONE && slot3.getColor() == ArtifactColor.NONE){
            state = State.INTAKE;
        }
        if(usePinpointForTeleOp){
            if(gamepad.isGamepadReady() && gamepad.options){
                toggleShooterNoPinpoint();
                gamepad.resetTimer();
            }
            else if(gamepad.buttonTimer.seconds() > 0.2 && gamepad.right_trigger >= 0.15){
                shootBall(currentBall);
                gamepad.resetTimer();
            }
        }
    }

    public void followPath(Path path){
        follower.followPath(path);
        follower.update();
    }

    public void followPath(CustomPath path){
        customFollower.setCurrentPath(path);
    }

    public void switchStates(){
        if(state == State.INTAKE){
            state = State.SHOOT;
        }
        else {
            state = State.INTAKE;
        }
    }

    public void log(TelemetryManager telemetryM, Telemetry telemetry){
        if(RobotConstantsV1.panelsEnabled) {
            telemetryM.addLine("Current Ball: " + currentBall);
            telemetryM.addLine("Slot 1: " + slot1.getColor().name());
            telemetryM.addLine("Slot 2: " + slot2.getColor().name());
            telemetryM.addLine("Slot 3: " + slot3.getColor().name());
        }
        else {
            telemetry.addLine("Alliance: " + alliance.name());
            telemetry.addLine("Slot 1: " + slot1.getColor().name());
            telemetry.addLine("Slot 2: " + slot2.getColor().name());
            telemetry.addLine("Slot 3: " + slot3.getColor().name());
            arcShooter.graph(telemetry);
        }
    }
}
