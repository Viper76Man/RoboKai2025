package org.firstinspires.ftc.teamcode.Jack.Odometry.Autonomous.NextFTC;

import androidx.annotation.AnimatorRes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Jack.Camera.Limelight3A.LimelightV1;
import org.firstinspires.ftc.teamcode.Jack.Drive.GamepadV1;
import org.firstinspires.ftc.teamcode.Jack.Drive.Robot;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotV4;
import org.firstinspires.ftc.teamcode.Jack.Odometry.Autonomous.Other.BlueAutoBackPickup1;
import org.firstinspires.ftc.teamcode.Jack.Odometry.BlueAutoPathsV2;
import org.firstinspires.ftc.teamcode.Jack.Odometry.CustomFollower;
import org.firstinspires.ftc.teamcode.Jack.Other.BallManager;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.AdjustableHoodV1;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.ArcMotorsV2;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.ColorSensorV3;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.DriveMotorsV2;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.FiringManager;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.FlickerSubsystem;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.IntakeMotorV2;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.SpindexerV2;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.Objects;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous
public class BlueAutoBackPickup1V2 extends NextFTCOpMode {
    public ParallelGroup intakeCommand, shootCommand, fireCommand, fireSingleCommand;
    public LED left1, right1, left2, right2;
    public IntakeMotorV2 intake = new IntakeMotorV2();
    public ColorSensorV3 sensor = new ColorSensorV3();
    public FlickerSubsystem flicker = new FlickerSubsystem();
    public SpindexerV2 spindexer = new SpindexerV2();
    public ElapsedTime matchTimer = new ElapsedTime();
    public LimelightSubsystem ll = new LimelightSubsystem();
    public AdjustableHoodV1 hood = new AdjustableHoodV1();
    public FiringManager firingManager = new FiringManager();
    public ArcMotorsV2 arcMotorsV2 = new ArcMotorsV2();
    public BallManager manager = new BallManager();

    public CustomFollower follower;
    public PathStates pathState = PathStates.START;
    public BlueAutoPathsV2 pathsV2 = new BlueAutoPathsV2();

    public boolean firedAlready = false;
    public boolean firedAlreadyPathing = false;
    public boolean firstLoop = true;

    public boolean shouldFire = false;
    public double OFFSET_ANGLE;
    public enum PathStates {
        START,
        TO_SHOOT,
        SHOOT_SET_1,
        TO_PICKUP_1,
        TURN_TO_PICKUP_1,
        PICKUP_1,
        BACK_TO_SHOOT_1,
        SHOOT_SET_2,
        OUT_OF_ZONE,
        IDLE
    }

    public enum SystemStates {
        START,
        BALL_1_INTAKE,
        BALL_2_INTAKE,
        BALL_3_INTAKE,
        SHOOT_ALL,
        SHOOT_SINGLE
    }

    public SystemStates state = SystemStates.START;
    public Robot.Mode mode = Robot.Mode.AUTONOMOUS;

    public void init(HardwareMap hardwareMap, Robot.Mode mode, Robot.Alliance alliance){
        follower = new CustomFollower(hardwareMap);
        this.mode = mode;
        intake = new IntakeMotorV2();
        intake.init(hardwareMap);
        spindexer.init(manager);
        spindexer.spindexer.resetEncoder();
        hood.init();
        ll.init();
        flicker.init(spindexer.spindexer);
        sensor.init(hardwareMap, manager, spindexer.spindexer);
        arcMotorsV2.init(hardwareMap, Robot.Mode.AUTONOMOUS);
        left1 = hardwareMap.get(LED.class, "left");
        right1 = hardwareMap.get(LED.class, "right");
        left2 = hardwareMap.get(LED.class, "left2");
        right2 = hardwareMap.get(LED.class, "right2");
        switch (alliance){
            case RED:
                OFFSET_ANGLE = RobotConstantsV1.TURRET_OFFSET_ANGLE_RED;
                ll.limelight.setPipeline(LimelightV1.Pipeline.RED_GOAL);
                break;
            case BLUE:
            case TEST:
                ll.limelight.setPipeline(LimelightV1.Pipeline.BLUE_GOAL);
                OFFSET_ANGLE = RobotConstantsV1.TURRET_OFFSET_ANGLE_BLUE;
                break;
        }
        ll.limelight.startStreaming();
    }

    public void buildCommands(){
        pathsV2.buildPaths();
        intakeCommand = new ParallelGroup(
                spindexer.spindexerRun(),
                sensor.update(),
                arcMotorsV2.spinUpIdle());
        firingManager.init(manager, flicker, spindexer.spindexer);
        shootCommand = new ParallelGroup(
                spindexer.spindexerRun(),
                arcMotorsV2.spinActive()
        );
        fireCommand = new ParallelGroup(firingManager.fireTriple(Robot.Mode.AUTONOMOUS, arcMotorsV2));
        fireSingleCommand = new ParallelGroup(firingManager.fireSingle(arcMotorsV2));
    }



    @Override
    public void onInit() {
        init(hardwareMap, Robot.Mode.AUTONOMOUS, Robot.Alliance.BLUE);
        buildCommands();

    }

    @Override
    public void onStartButtonPressed(){
        pathState = PathStates.START;
        follower.setStartingPose(BlueAutoPathsV2.startPoseFar);
    }

    public void onUpdate(){
        hood.hoodServo.setPos(0.2);
        if(isStopRequested()){
            return;
        }
        systemStatesUpdate();
        autoPathUpdate();
    }

    public void systemStatesUpdate(){
        log();
        ll.turret.run(ll.limelight, OFFSET_ANGLE);
        if(firstLoop){
            flicker.fire().run();
            firstLoop = false;
        }
        switch (state){
            case START:
                redLED();
                firedAlreadyPathing = true;
                manager.setCurrentBall(1);
                manager.setMode(BallManager.State.INTAKE);
                setSystemState(SystemStates.BALL_1_INTAKE);
                intakeCommand.schedule();
                break;
            case BALL_1_INTAKE:
                if ((sensor.isPurple() || sensor.isGreen()) && sensor.sensor.getNormalizedRGB().green >= 0.03) {
                    manager.setBall1(sensor.sensor.getCurrent());
                    manager.setCurrentBall(2);
                    sensor.clear();
                    setSystemState(SystemStates.BALL_2_INTAKE);
                }
                break;
            case BALL_2_INTAKE:
                if ((sensor.isPurple() || sensor.isGreen()) && sensor.sensor.getNormalizedRGB().green >= 0.03) {
                    manager.setBall2(sensor.sensor.getCurrent());
                    manager.setCurrentBall(3);
                    sensor.clear();
                    setSystemState(SystemStates.BALL_3_INTAKE);
                }
                break;
            case BALL_3_INTAKE:
                if ((sensor.isPurple() || sensor.isGreen()) && sensor.sensor.getNormalizedRGB().green >= 0.03) {
                    manager.setBall3(sensor.sensor.getCurrent());
                    manager.next();
                    sensor.clear();
                    shootCommand.schedule();
                    manager.setMode(BallManager.State.SHOOT);
                    setSystemState(SystemStates.SHOOT_ALL);
                    greenLED();
                }
                break;
            case SHOOT_ALL:
                if(readyForTriple()){
                    fireTriple();
                }
                if(manager.mode == BallManager.State.INTAKE && firedAlready && fireCommand.isDone()){
                    setSystemState(SystemStates.START);
                    firedAlready = false;
                    firedAlreadyPathing = true;
                    shouldFire = false;
                }
                break;
        }
    }

    public void autoPathUpdate() {
        follower.update();
        follower.log(telemetry);
        telemetry.addData("Pose: ", follower.follower.getPose());
        if(matchTimer.seconds() > 29){
            setPathState(PathStates.OUT_OF_ZONE);
        }
        switch (pathState) {
            case START:
                setPathState(PathStates.TO_SHOOT);
                intake.intake.setPower(RobotConstantsV1.INTAKE_POWER);
                break;
            case TO_SHOOT:
                if (!follower.isBusy()) {
                    follower.setCurrentPath(BlueAutoPathsV2.outOfStartFar);
                    setPathState(PathStates.SHOOT_SET_1);
                }
                firedAlreadyPathing = false;
                break;
            case SHOOT_SET_1:
                if (!follower.follower.isBusy() && !firedAlreadyPathing && !shouldFire){
                    shouldFire = true;
                }
                if (firedAlreadyPathing) {
                    setPathState(PathStates.TO_PICKUP_1);
                    firedAlreadyPathing = false;
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
                    follower.setCurrentPath(BlueAutoPathsV2.pickup1);
                    setPathState(PathStates.BACK_TO_SHOOT_1);
                }
                break;
            case BACK_TO_SHOOT_1:
                if(isLastPathName(BlueAutoPathsV2.pickup1.getName()) && follower.isBusy() && Math.toDegrees(follower.follower.getPose().getHeading()) > 150){
                    follower.follower.setMaxPower(0.17);
                }
                if(isLastPathName(BlueAutoPathsV2.pickup1.getName()) && !follower.isBusy()){
                    follower.setCurrentPath(BlueAutoPathsV2.overdriveBack1);
                    follower.follower.setMaxPower(1);
                }
                else if(isLastPathName(BlueAutoPathsV2.overdriveBack1.getName()) && follower.follower.getCurrentTValue() > BlueAutoPathsV2.backToShoot1OverdriveTValue){
                    follower.setCurrentPath(BlueAutoPathsV2.backToShoot1);
                    setPathState(PathStates.SHOOT_SET_2);
                }
                firedAlreadyPathing = false;
                break;
            case SHOOT_SET_2:
                if (follower.follower.getCurrentTValue() > 0.7 && !firedAlreadyPathing && !shouldFire) {
                    shouldFire = true;
                }
                if ((firedAlready && manager.mode == BallManager.State.INTAKE)) {
                    setPathState(PathStates.OUT_OF_ZONE);
                }
                break;
            case OUT_OF_ZONE:
                if(!follower.isBusy()){
                    follower.setCurrentPath(BlueAutoPathsV2.leaveShoot);
                    setPathState(PathStates.IDLE);
                }
        }
    }

    public void setPathState(PathStates pathState) {
        this.pathState = pathState;
    }


    public void log(){
        flicker.flicker.log(ActiveOpMode.telemetry());
        arcMotorsV2.arcShooter.log(ActiveOpMode.telemetry());
        ActiveOpMode.telemetry().addLine("Current ball: " + manager.getCurrentBall());
        ActiveOpMode.telemetry().addLine("Mode: "+ manager.mode);
    }

    public void logCurrentCommands(Telemetry telemetry){
        for(String command : CommandManager.INSTANCE.snapshot()){
            telemetry.addLine(command);
        }
    }

    public void logCurrentCommands(TelemetryManager telemetryM){
        for(String command : CommandManager.INSTANCE.snapshot()){
            telemetryM.addLine(command);
        }
    }

    public void setSystemState(SystemStates state){
        this.state = state;
    }


    public void fireTriple(){
        fireCommand.schedule();
        firedAlready = true;
    }

    public boolean readyForTriple(){
        return spindexer.spindexer.isSpindexerReady() && !firedAlready && shouldFire;
    }


    public void redLED(){
        left1.on();
        left2.on();
        right1.off();
        right2.off();
    }

    public void greenLED(){
        left1.off();
        left2.off();
        right1.on();
        right2.on();
    }

    public boolean isLastPathName(String name){
        return Objects.equals(follower.lastPathName, name);
    }

}

