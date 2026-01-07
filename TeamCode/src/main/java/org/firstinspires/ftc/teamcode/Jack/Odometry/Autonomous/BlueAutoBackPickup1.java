package org.firstinspires.ftc.teamcode.Jack.Odometry.Autonomous;

import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Jack.Camera.Limelight3A.LimelightV1;
import org.firstinspires.ftc.teamcode.Jack.Drive.GamepadV1;
import org.firstinspires.ftc.teamcode.Jack.Drive.Robot;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotV3;
import org.firstinspires.ftc.teamcode.Jack.Motors.ArcShooterV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.IntakeV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.PIDController;
import org.firstinspires.ftc.teamcode.Jack.Motors.SpindexerMotorV1;
import org.firstinspires.ftc.teamcode.Jack.Odometry.BlueAutoPathsV2;
import org.firstinspires.ftc.teamcode.Jack.Odometry.CustomFollower;
import org.firstinspires.ftc.teamcode.Jack.Odometry.DecodeFieldLocalizer;
import org.firstinspires.ftc.teamcode.Jack.Odometry.RedAutoPathsV2;
import org.firstinspires.ftc.teamcode.Jack.Other.ArtifactColor;
import org.firstinspires.ftc.teamcode.Jack.Other.ArtifactSlot;
import org.firstinspires.ftc.teamcode.Jack.Other.DecodeAprilTag;
import org.firstinspires.ftc.teamcode.Jack.Other.Drawing;
import org.firstinspires.ftc.teamcode.Jack.Other.Range;
import org.firstinspires.ftc.teamcode.Jack.Other.SlotColorSensorV1;
import org.firstinspires.ftc.teamcode.Jack.Servos.FlickerServoV2;
import org.firstinspires.ftc.teamcode.Jack.Servos.TurretServoCR;

import java.util.Objects;

@Autonomous
public class BlueAutoBackPickup1 extends LinearOpMode {
    public CustomFollower follower;
    public BlueAutoPathsV2 pathsV2 = new BlueAutoPathsV2();
    public DecodeAprilTag obeliskTag;
    public ElapsedTime ballTimer = new ElapsedTime();
    public ElapsedTime noResultTimer = new ElapsedTime();
    public PIDController controller;
    //HARDWARE--------------------------------------------------------------------------------------
    public ArcShooterV1 arcShooter = new ArcShooterV1();
    public SpindexerMotorV1 spindexer = new SpindexerMotorV1();
    public IntakeV1 intake = new IntakeV1();
    public FlickerServoV2 flicker = new FlickerServoV2();
    public TurretServoCR turret = new TurretServoCR();
    public LimelightV1 limelight = new LimelightV1();
    public SlotColorSensorV1 sensor = new SlotColorSensorV1();
    //-------------------------------------------------------------------------------------
    public double cameraTx = 0;
    public int ballsFired = 0;
    public boolean fire = false;
    public boolean firedAlready = false;
    public boolean turretReady = false;
    public boolean clearedForIntake = false;
    public ArtifactSlot slot1 = new ArtifactSlot();
    public ArtifactSlot slot2 = new ArtifactSlot();
    public ArtifactSlot slot3 = new ArtifactSlot();
    public double currentBall = 1;
    public enum PathStates {
        START,
        TO_SHOOT,
        SHOOT_SET_1,
        TO_PICKUP_1,
        TURN_TO_PICKUP_1,
        PICKUP_1,
        BACK_TO_SHOOT_1,
        SHOOT_SET_2,
        IDLE
    }

    public enum State {
        INTAKE_BALL_1,
        INTAKE_BALL_2,
        INTAKE_BALL_3,
        SHOOT_BALL_1,
        SHOOT_BALL_2,
        SHOOT_BALL_3
    }

    public State actionState = State.INTAKE_BALL_1;

    public PathStates pathState;
    public boolean actionIsSet = false;

    @Override
    public void runOpMode() {
        initHardware();
        pathState = PathStates.START;
        follower.setStartingPose(BlueAutoPathsV2.startPoseFar);
        if (obeliskTag != null) {
            telemetry.addData("Latest tag: ", obeliskTag.name());
        }
        waitForStart();
        while (opModeIsActive()) {
            log();
            autoPathUpdate();
            systemStatesUpdate();
            if (RobotConstantsV1.panelsEnabled) {
                draw();
            }
        }
    }

    public void initHardware() {
        follower = new CustomFollower(hardwareMap);
        pathsV2.buildPaths();
        sensor.init(hardwareMap, RobotConstantsV1.colorSensor1);
        arcShooter.init(hardwareMap, RobotConstantsV1.arcPIDsAuto);
        spindexer.init(hardwareMap, RobotConstantsV1.spindexerPIDs);
        spindexer.resetEncoder();
        flicker.init(hardwareMap, RobotConstantsV1.flickerServoName);
        turret.init(hardwareMap);
        spindexer.setTargetPos(RobotConstantsV1.SPINDEXER_MOTOR_BALL_1_INTAKE, SpindexerMotorV1.EncoderMeasurementMethod.MOTOR);
        intake.init(hardwareMap);
        intake.setDirection(RobotConstantsV1.intakeDirection);
        limelight.init(hardwareMap);
        limelight.setPipeline(LimelightV1.Pipeline.BLUE_GOAL);
        limelight.startStreaming();
        controller = new PIDController(RobotConstantsV1.turretPIDs.p, RobotConstantsV1.turretPIDs.i, RobotConstantsV1.turretPIDs.d);
        setEmpty(1);
        setEmpty(2);
        setEmpty(3);
        sensor.sensor.setGain(10);
    }

    public void autoPathUpdate() {
        follower.update(telemetry);
        telemetry.addData("Pose: ", follower.follower.getPose());
        switch (pathState) {
            case START:
                setPathState(PathStates.TO_SHOOT);
                if (!actionIsSet) {
                    setActionState(State.INTAKE_BALL_1);
                    actionIsSet = true;
                }
                break;
            case TO_SHOOT:
                if (!follower.isBusy()) {
                    follower.setCurrentPath(BlueAutoPathsV2.outOfStartFar);
                    setPathState(PathStates.SHOOT_SET_1);
                    fire = true;
                    break;
                }
                break;
            case SHOOT_SET_1:
                if (follower.follower.getCurrentTValue() >= 1 && actionState == State.INTAKE_BALL_1 || actionState == State.INTAKE_BALL_2 || actionState == State.INTAKE_BALL_3 && fire) {
                    if (ballTimer.seconds() > 0.5) {
                        setActionState(State.SHOOT_BALL_1);
                        clearedForIntake = false;
                    }
                }
                if (follower.follower.getCurrentTValue() < 1 && Objects.equals(follower.path.name, "outOfStartFar")) {
                    ballTimer.reset();
                }
                if (!fire) {
                    setPathState(PathStates.TO_PICKUP_1);
                    clearedForIntake = true;
                    ballsFired = 0;
                }
                break;
            case TO_PICKUP_1:
                if(!follower.isBusy()){
                    follower.setCurrentPath(BlueAutoPathsV2.toFirstArtifacts);
                    setPathState(PathStates.PICKUP_1);
                }
                break;
            case PICKUP_1:
                if(!follower.isBusy()){
                    setActionState(State.INTAKE_BALL_1);
                    follower.setCurrentPath(BlueAutoPathsV2.pickup1);
                    setPathState(PathStates.BACK_TO_SHOOT_1);
                }
                break;
            case BACK_TO_SHOOT_1:
                if(isLastPathName(BlueAutoPathsV2.pickup1.getName()) && !follower.isBusy()){
                    follower.setCurrentPath(BlueAutoPathsV2.overdriveBack1);
                }
                else if(isLastPathName(BlueAutoPathsV2.overdriveBack1.getName()) && follower.follower.getCurrentTValue() > BlueAutoPathsV2.backToShoot1OverdriveTValue){
                    follower.setCurrentPath(BlueAutoPathsV2.backToShoot1);
                    setPathState(PathStates.SHOOT_SET_2);
                }
                break;
            case SHOOT_SET_2:
                if (follower.follower.getCurrentTValue() >= 1 && actionState == State.INTAKE_BALL_3 && fire) {
                    if (ballTimer.seconds() > 0.5) {
                        setActionState(State.SHOOT_BALL_1);
                        clearedForIntake = false;
                    }
                }
                if (follower.follower.getCurrentTValue() < 1 && Objects.equals(follower.path.name, "backToShoot1")) {
                    ballTimer.reset();
                }
                if (!fire) {
                    setPathState(PathStates.IDLE);
                    clearedForIntake = true;
                    ballsFired = 0;
                }
                break;
        }
    }

    public void systemStatesUpdate() {
        turretUpdate();
        spindexerRun();
        arcShooter.run();
        flicker.update(spindexer.isSpindexerReady());
        spindexer.update();
        switch (actionState) {
            case SHOOT_BALL_1:
                sensor.clear();
                currentBall = 1;
                arcShooter.setTargetRPM(RobotConstantsV1.SHOOTER_TARGET_RPM);
                spindexer.setState(SpindexerMotorV1.State.BALL_1_SHOOT);
                intake.setPower(0.2);
                if(fire) {
                    if (flicker.state == FlickerServoV2.State.IDLE && !firedAlready && new Range((RobotConstantsV1.SHOOTER_TARGET_RPM_AUTO), 10).isInRange(arcShooter.getVelocityRPM()) && spindexer.isSpindexerReady() && turretReady) {
                        flicker.setState(FlickerServoV2.State.DOWN);
                    }
                    else if (flicker.state == FlickerServoV2.State.IDLE && !firedAlready && (arcShooter.getVelocityRPM() > RobotConstantsV1.SHOOTER_TARGET_RPM_AUTO + 10 && arcShooter.getVelocityRPM() < RobotConstantsV1.SHOOTER_TARGET_RPM_AUTO + 20)  && spindexer.isSpindexerReady() && turretReady) {
                        flicker.setState(FlickerServoV2.State.DOWN);
                    }
                    if (flicker.state != FlickerServoV2.State.IDLE && !firedAlready) {
                        firedAlready = true;
                    }
                    if (flicker.state == FlickerServoV2.State.IDLE && firedAlready) {
                        setEmpty(1);
                        setActionState(State.SHOOT_BALL_2);
                        firedAlready = false;
                    }
                }
                break;
            case SHOOT_BALL_2:
                sensor.clear();
                currentBall = 2;
                arcShooter.setTargetRPM(RobotConstantsV1.SHOOTER_TARGET_RPM);
                spindexer.setState(SpindexerMotorV1.State.BALL_2_SHOOT);
                intake.setPower(0.2);
                if(fire) {
                    if (flicker.state == FlickerServoV2.State.IDLE && !firedAlready && new Range((RobotConstantsV1.SHOOTER_TARGET_RPM_AUTO), 10).isInRange(arcShooter.getVelocityRPM()) && spindexer.isSpindexerReady() && turretReady) {
                        flicker.setState(FlickerServoV2.State.DOWN);
                    }
                    else if (flicker.state == FlickerServoV2.State.IDLE && !firedAlready && (arcShooter.getVelocityRPM() > RobotConstantsV1.SHOOTER_TARGET_RPM_AUTO + 10 && arcShooter.getVelocityRPM() < RobotConstantsV1.SHOOTER_TARGET_RPM_AUTO + 20)  && spindexer.isSpindexerReady() && turretReady) {
                        flicker.setState(FlickerServoV2.State.DOWN);
                    }
                    if(flicker.state != FlickerServoV2.State.IDLE && !firedAlready){
                        firedAlready = true;
                    }
                    if (flicker.state == FlickerServoV2.State.IDLE && firedAlready) {
                        setEmpty(2);
                        setActionState(State.SHOOT_BALL_3);
                        firedAlready = false;
                    }
                }
                break;
            case SHOOT_BALL_3:
                sensor.clear();
                currentBall = 3;
                arcShooter.setTargetRPM(RobotConstantsV1.SHOOTER_TARGET_RPM);
                spindexer.setState(SpindexerMotorV1.State.BALL_3_SHOOT);
                intake.setPower(0.2);
                if(fire) {
                    if (flicker.state == FlickerServoV2.State.IDLE && !firedAlready && new Range((RobotConstantsV1.SHOOTER_TARGET_RPM_AUTO), 10).isInRange(arcShooter.getVelocityRPM()) && spindexer.isSpindexerReady() && turretReady) {
                        flicker.setState(FlickerServoV2.State.DOWN);
                    }
                    else if (flicker.state == FlickerServoV2.State.IDLE && !firedAlready && (arcShooter.getVelocityRPM() > RobotConstantsV1.SHOOTER_TARGET_RPM_AUTO + 10 && arcShooter.getVelocityRPM() < RobotConstantsV1.SHOOTER_TARGET_RPM_AUTO + 20)  && spindexer.isSpindexerReady() && turretReady) {
                        flicker.setState(FlickerServoV2.State.DOWN);
                    }
                    if(flicker.state != FlickerServoV2.State.IDLE && !firedAlready){
                        firedAlready = true;
                    }
                    if (flicker.state == FlickerServoV2.State.IDLE && firedAlready) {
                        clearedForIntake = true;
                        setEmpty(3);
                        firedAlready = false;
                        fire = false;
                    }
                }
                break;
            case INTAKE_BALL_1:
                currentBall = 1;
                arcShooter.setTargetRPM(RobotConstantsV1.SHOOTER_IDLE_RPM);
                if(ballCollectUpdate(1) && clearedForIntake && spindexer.isSpindexerReady() ) {
                    setActionState(State.INTAKE_BALL_2);
                    currentBall = 2;
                    sensor.clear();
                }
                else {
                    spindexer.setState(SpindexerMotorV1.State.BALL_1_INTAKE);
                }
                intake.setPower(RobotConstantsV1.INTAKE_POWER);
                break;
            case INTAKE_BALL_2:
                arcShooter.setTargetRPM(RobotConstantsV1.SHOOTER_IDLE_RPM);
                //TODO: If that doesn't work, try adding these conditions to ballCollectUpdate() :)
                if(ballCollectUpdate(2) && clearedForIntake && spindexer.isSpindexerReady()) {
                    setActionState(State.INTAKE_BALL_3);
                    sensor.clear();
                    currentBall = 3;
                }
                else {
                    spindexer.setState(SpindexerMotorV1.State.BALL_2_INTAKE);
                }
                intake.setPower(RobotConstantsV1.INTAKE_POWER);
                break;
            case INTAKE_BALL_3:
                arcShooter.setTargetRPM(RobotConstantsV1.SHOOTER_IDLE_RPM);
                if(ballCollectUpdate(3) && clearedForIntake && spindexer.isSpindexerReady()) {
                    spindexer.setState(SpindexerMotorV1.State.BALL_1_SHOOT);
                    sensor.clear();
                    clearedForIntake = false;
                }
                else {
                    spindexer.setState(SpindexerMotorV1.State.BALL_3_INTAKE);
                }
                intake.setPower(RobotConstantsV1.INTAKE_POWER);
                break;
        }
    }

    public void draw() {
        Drawing.drawRobot(follower.follower.getPose());
        Drawing.drawPoseHistory(follower.follower.getPoseHistory());
        Drawing.sendPacket();
    }

    public void fireBall() {
        ballsFired += 1;
        fire = true;
    }

    public void setPathState(PathStates pathState) {
        this.pathState = pathState;
    }

    public void setActionState(State actionState) {
        this.actionState = actionState;
        actionIsSet = false;
    }

    public void log() {
        telemetry.addLine("State: " + pathState.name());
        telemetry.addLine("Action: " + actionState.name());
        telemetry.addLine("Balls fired: " + ballsFired);
        telemetry.addLine("T-value: " + follower.follower.getCurrentTValue());
        telemetry.addLine("Busy? " + follower.isBusy());
        telemetry.addLine("RPM: " + arcShooter.getVelocityRPM());
        telemetry.addLine("Sensor State: " + sensor.getCurrent().name());
        telemetry.addLine("Ready to intake? :" + clearedForIntake);
        sensor.log(PanelsTelemetry.INSTANCE.getTelemetry(), telemetry);
        if(RobotConstantsV1.panelsEnabled){
            PanelsTelemetry.INSTANCE.getTelemetry().update(telemetry);
        }
    }

    public void turretUpdate() {
        double power;
        LLResultTypes.FiducialResult latest_result = limelight.getLatestAprilTagResult();
        if (latest_result != null) {
            double latestTagID = latest_result.getFiducialId();
            cameraTx = latest_result.getTargetXDegreesNoCrosshair();
            noResultTimer.reset();
            power = -controller.getOutput(cameraTx + RobotConstantsV1.TURRET_OFFSET_ANGLE_BLUE_AUTO);
        } else {
            cameraTx = 0;
            power = -controller.getOutput((int) turret.getEncoderPos(), 236);
        }
        if (turret.getEncoderPos() >= RobotConstantsV1.TURRET_MAX_ENCODER_VALUE && power < 0) {
            power = 0;
        }
        if (Math.abs((cameraTx + RobotConstantsV1.TURRET_OFFSET_ANGLE_BLUE_AUTO)) < RobotConstantsV1.degreeToleranceCameraAuto) {
            power = power / 2;
            turretReady = true;
        }
        else {
            turretReady = false;
        }
        turret.setPower(power);
    }

    public void setEmpty(int ball){
        sensor.clear();
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
    public boolean ballCollectUpdate(int ball){
        sensor.update(spindexer.state, spindexer.isSpindexerReady());
        if (sensor.getCurrent() == ArtifactColor.GREEN && currentBall == ball && isEmpty(currentBall) && (sensor.sensor.getNormalizedColors().green / sensor.sensor.getNormalizedColors().alpha) < 0.15 && sensor.sensor.getNormalizedColors().green > 0.03){
            setGreen(ball);
            return true;
        } else if (sensor.getCurrent() == ArtifactColor.PURPLE && currentBall == ball) {
            setPurple(ball);
            return true;
        }
        return false;
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
    public boolean isEmpty(int ball){
        switch (ball){
            case 1:
                return slot1.getColor() == ArtifactColor.NONE;
            case 2:
                return slot2.getColor() == ArtifactColor.NONE;
            case 3:
                return slot3.getColor() == ArtifactColor.NONE;
        }
        return true;
    }

    public boolean isEmpty(double ball){
        return isEmpty((int) ball);
    }

    public State getNextState(State state){
        switch (state){
            case INTAKE_BALL_1:
                if(isEmpty(2)){
                    return State.INTAKE_BALL_2;
                }
                else if(isEmpty(3)){
                    return State.INTAKE_BALL_3;
                }
                return State.SHOOT_BALL_1;
            case INTAKE_BALL_2:
                if(isEmpty(1)){
                    return State.INTAKE_BALL_1;
                }
                else if(isEmpty(3)){
                    return State.INTAKE_BALL_3;
                }
                return State.SHOOT_BALL_1;
            case INTAKE_BALL_3:
                if(isEmpty(1)){
                    return State.INTAKE_BALL_1;
                }
                else if(isEmpty(2)){
                    return State.INTAKE_BALL_2;
                }
                return State.SHOOT_BALL_1;
            case SHOOT_BALL_1:
            case SHOOT_BALL_2:
            case SHOOT_BALL_3:
                if(!isEmpty(1)){
                    return State.SHOOT_BALL_1;
                }
                else if(!isEmpty(2)){
                    return State.SHOOT_BALL_2;
                }
                else if(!isEmpty(3)){
                    return State.SHOOT_BALL_3;
                }
                return State.INTAKE_BALL_1;
            default:
                return State.INTAKE_BALL_1;
        }
    }

    public void spindexerRun(){
        switch (actionState) {
            case SHOOT_BALL_1:
                spindexer.setState(SpindexerMotorV1.State.BALL_1_SHOOT);
                break;
            case SHOOT_BALL_2:
                spindexer.setState(SpindexerMotorV1.State.BALL_2_SHOOT);
                break;
            case SHOOT_BALL_3:
                spindexer.setState(SpindexerMotorV1.State.BALL_3_SHOOT);
                break;
            case INTAKE_BALL_1:
                spindexer.setState(SpindexerMotorV1.State.BALL_1_INTAKE);
                break;
            case INTAKE_BALL_2:
                spindexer.setState(SpindexerMotorV1.State.BALL_2_INTAKE);
                break;
            case INTAKE_BALL_3:
                spindexer.setState(SpindexerMotorV1.State.BALL_3_INTAKE);
                break;
        }
        spindexer.run();
    }
    public boolean isLastPathName(String name){
        return Objects.equals(follower.lastPathName, name);
    }

}
