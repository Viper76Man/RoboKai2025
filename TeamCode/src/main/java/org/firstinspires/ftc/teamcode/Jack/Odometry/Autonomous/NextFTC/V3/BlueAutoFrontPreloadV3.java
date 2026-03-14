package org.firstinspires.ftc.teamcode.Jack.Odometry.Autonomous.NextFTC.V3;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Coach.subsystems.Ramp;
import org.firstinspires.ftc.teamcode.Jack.Camera.Limelight3A.LimelightV1;
import org.firstinspires.ftc.teamcode.Jack.Drive.GamepadV1;
import org.firstinspires.ftc.teamcode.Jack.Drive.Robot;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotV4;
import org.firstinspires.ftc.teamcode.Jack.Motors.SpindexerMotorV2;
import org.firstinspires.ftc.teamcode.Jack.Odometry.Autonomous.NextFTC.BlueAutoBackPreloadV2;
import org.firstinspires.ftc.teamcode.Jack.Odometry.BlueAutoPathsV2;
import org.firstinspires.ftc.teamcode.Jack.Odometry.Constants;
import org.firstinspires.ftc.teamcode.Jack.Odometry.CustomFollower;
import org.firstinspires.ftc.teamcode.Jack.Other.ArtifactColor;
import org.firstinspires.ftc.teamcode.Jack.Other.BallManager;
import org.firstinspires.ftc.teamcode.Jack.Other.Sensors;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.AdjustableHoodV1;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.ArcMotorsV2;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.ColorSensorV3;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.DriveMotorsV2;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.FiringManager;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.FlickerSubsystem;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.IntakeMotorV2;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.SpindexerV2;

import java.util.Objects;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.driving.MecanumDriverControlled;

@Autonomous
public class BlueAutoFrontPreloadV3 extends NextFTCOpMode {
    public ParallelGroup driveCommand;
    public Command fireCommand, liftCommand, rampDown, arcSpinUp, driveMotors;

    public double OFFSET_ANGLE;

    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public TelemetryManager telemetryM;
    public GamepadV1 gamepad = new GamepadV1();

    public boolean firstLoop = true;

    public enum SystemStates {
        START,
        BALL_1_INTAKE,
        BALL_2_INTAKE,
        BALL_3_INTAKE,
        SHOOT_ALL,
        SHOOT_SINGLE
    }

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


    public SpindexerMotorV2 spindexer = SpindexerMotorV2.INSTANCE_AUTO;
    public ColorSensorV3 sensor = new ColorSensorV3();
    public IntakeMotorV2 intake = new IntakeMotorV2();
    public AdjustableHoodV1 hood = new AdjustableHoodV1();
    public BlueAutoPathsV2 pathsV2 = new BlueAutoPathsV2();
    public LiftSubsystem lift = new LiftSubsystem();

    public ArcMotorsV2 arc = new ArcMotorsV2();
    public DriveMotorsV2 drive = new DriveMotorsV2();
    public Sensors sensors = new Sensors();
    public LimelightSubsystem ll;

    public Robot.Alliance alliance = Robot.Alliance.BLUE;
    public PathStates pathState = PathStates.START;

    public BallManager manager = new BallManager();
    public SystemStates state = SystemStates.START;
    public CustomFollower follower;

    public boolean shooting = false;
    public boolean firedAlreadyPathing = false;
    public boolean shouldFire = false;
    public ElapsedTime matchTimer = new ElapsedTime();


    public BlueAutoFrontPreloadV3(){
        addComponents(new SubsystemComponent(SpindexerMotorV2.INSTANCE_AUTO));
    }

    private void initialize(){
        hardwareMap = ActiveOpMode.hardwareMap();
        follower = new CustomFollower(hardwareMap);
        pathsV2.buildPaths();
        telemetry = ActiveOpMode.telemetry();
        gamepad.init(ActiveOpMode.gamepad1(), 0.3);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        ll = new LimelightSubsystem(Robot.Mode.TELEOP, alliance);
        ll.init();
        sensors.init(hardwareMap);
        spindexer.init(manager);
        lift.init();
        hood.init(ll.limelight);
        sensor.init(hardwareMap, manager, spindexer);
        arc.init(hardwareMap, Robot.Mode.TELEOP, ll.limelight, sensors);
        intake.init(hardwareMap);
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
        driveCommand = new ParallelGroup(
                sensor.update(),
                spindexer.spindexerUpdate(),
                ll.turretUpdate(OFFSET_ANGLE),
                hood.servoUpdate()
        );
        manager.setMode(BallManager.State.INTAKE);
        fireCommand = buildShot();
        liftCommand = lift.liftUpdate();
        rampDown = Ramp.INSTANCE.rampDown;
        arcSpinUp = arc.spinUpIdle();
        follower.setStartingPose(BlueAutoPathsV2.startPoseClose);
    }

    public Command buildShot(){
        return new ParallelRaceGroup(
                spindexer.Fire(),
                arc.spinActive(),
                new Delay(5)
        );
    }



    @Override
    public void onInit() {
        initialize();
        buildCommands();
    }

    @Override
    public void onWaitForStart(){
        if(isStopRequested()){
            return;
        }
    }

    @Override
    public void onStartButtonPressed(){
    }

    public void onUpdate(){
        if(isStopRequested()){
            return;
        }
        systemStatesUpdate();
        autoPathUpdate();
    }

    public void systemStatesUpdate(){
        switch (state){
            case START:
                manager.setMode(BallManager.State.INTAKE);
                manager.setCurrentBall(1);
                if (Objects.equals(fireCommand, null)) {
                    fireCommand = buildShot();
                }
                if(!shooting){
                    fireCommand.cancel();
                    rampDown.cancel();
                    arcSpinUp.cancel();
                    arcSpinUp = arc.spinUpIdle();
                    driveCommand.cancel();
                    driveCommand.schedule();
                    rampDown.schedule();
                    manager.setMode(BallManager.State.INTAKE);
                    spindexer.setState(SpindexerMotorV2.SpindexerState.INTAKE_BALL_1);
                    setSystemState(SystemStates.BALL_1_INTAKE);
                    shooting = false;
                    arcSpinUp.schedule();
                }
                if(fireCommand.isDone() && shooting) {
                    fireCommand.cancel();
                    rampDown.cancel();
                    arcSpinUp.cancel();
                    arcSpinUp = arc.spinUpIdle();
                    driveCommand.cancel();
                    for(Command cmd : driveCommand.getCommands()){
                        cmd.cancel();
                    }
                    rampDown.schedule();
                    setSystemState(SystemStates.BALL_1_INTAKE);
                    spindexer.setState(SpindexerMotorV2.SpindexerState.INTAKE_BALL_1);
                    manager.setCurrentBall(1);
                    shooting = false;
                    arcSpinUp.schedule();
                }
                break;
            case BALL_1_INTAKE:
                if(!driveCommand.isScheduled()){
                    driveCommand.schedule();
                }
                rampDown.cancel();
                manager.setCurrentBall(1);
                if(sensor.hasBall() && !spindexer.isActive2()){
                    spindexer.next();
                    setSystemState(SystemStates.BALL_2_INTAKE);
                    sensor.clear();
                }
                break;
            case BALL_2_INTAKE:
                manager.setCurrentBall(2);
                if(sensor.hasBall() && !spindexer.isActive2()){
                    spindexer.next();
                    setSystemState(SystemStates.BALL_3_INTAKE);
                    sensor.clear();
                }
                break;
            case BALL_3_INTAKE:
                manager.setCurrentBall(3);
                if(sensor.hasBall() && !spindexer.isActive2()){
                    manager.setCurrentBall(4);
                    manager.setMode(BallManager.State.SHOOT);
                    setSystemState(SystemStates.SHOOT_ALL);
                    sensor.clear();
                }
                break;
            case SHOOT_ALL:
                if(!fireCommand.isScheduled() && !firedAlreadyPathing && !shooting && shouldFire) {
                    shoot();
                    firedAlreadyPathing = true;
                    gamepad.resetTimer();
                }
                break;
        }

    }
    public void shoot(){
        fireCommand.cancel();
        fireCommand = buildShot();
        fireCommand.schedule();
        setSystemState(SystemStates.START);
        shooting = true;
    }

    public void autoPathUpdate() {
        follower.update();
        telemetry.addData("Pose: ", follower.follower.getPose());
        if(matchTimer.seconds() > 28.5){
            setPathState(PathStates.OUT_OF_ZONE);
        }
        switch (pathState) {
            case START:
                setPathState(PathStates.TO_SHOOT);
                intake.intake.setPower(RobotConstantsV1.INTAKE_POWER);
                break;
            case TO_SHOOT:
                if (!follower.isBusy()) {
                    follower.setCurrentPath(BlueAutoPathsV2.outOfStartClose);
                    manager.setBall1(ArtifactColor.GREEN);
                    manager.setBall2(ArtifactColor.PURPLE);
                    manager.setCurrentBall(3);
                    setSystemState(SystemStates.BALL_3_INTAKE);
                    setPathState(PathStates.SHOOT_SET_1);
                }
                firedAlreadyPathing = false;
                break;
            case SHOOT_SET_1:
                if (!follower.follower.isBusy() && !firedAlreadyPathing && !shouldFire){
                    shouldFire = true;
                }
                if (firedAlreadyPathing) {
                    setPathState(PathStates.OUT_OF_ZONE);
                    firedAlreadyPathing = false;
                }
                break;
            case OUT_OF_ZONE:
                if(!follower.isBusy()){
                    follower.setCurrentPath(BlueAutoPathsV2.leaveShoot);
                    setPathState(PathStates.IDLE);
                }
                break;
            case IDLE:
                if(!follower.isBusy()){
                    requestOpModeStop();
                }
                break;
        }
    }

    public void setPathState(PathStates pathState) {
        this.pathState = pathState;
    }


    public void log(){

    }

    public void setSystemState(SystemStates state){
        this.state = state;
    }

}

