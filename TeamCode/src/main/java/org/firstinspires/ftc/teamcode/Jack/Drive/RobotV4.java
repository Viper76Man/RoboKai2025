package org.firstinspires.ftc.teamcode.Jack.Drive;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Coach.subsystems.Ramp;
import org.firstinspires.ftc.teamcode.Jack.Camera.Limelight3A.LimelightV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.SpindexerMotorV2;
import org.firstinspires.ftc.teamcode.Jack.Other.BallManager;
import org.firstinspires.ftc.teamcode.Jack.Other.Sensors;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.AdjustableHoodV1;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.ArcMotorsV2;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.ColorSensorV3;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.DriveMotorsV2;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.IntakeMotorV2;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.LimelightSubsystem;

import java.util.Objects;
import java.util.logging.Handler;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

public class RobotV4 implements Subsystem {
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
        SHOOT_SINGLE,
        LIFT
    }


    public SpindexerMotorV2 spindexer = SpindexerMotorV2.INSTANCE;
    public ColorSensorV3 sensor = new ColorSensorV3();
    public IntakeMotorV2 intake = new IntakeMotorV2();
    public AdjustableHoodV1 hood = new AdjustableHoodV1();
    public LiftSubsystem lift = new LiftSubsystem();

    public ArcMotorsV2 arc = new ArcMotorsV2();
    public DriveMotorsV2 drive = new DriveMotorsV2();
    public Sensors sensors = new Sensors();
    public LimelightSubsystem ll;

    public Robot.Alliance alliance;

    public BallManager manager = new BallManager();
    public SystemStates state = SystemStates.START;

    public boolean shooting = false;

    private final MotorEx frontLeftMotor = new MotorEx("fl").reversed();
    private final MotorEx frontRightMotor = new MotorEx("fr");
    private final MotorEx backLeftMotor = new MotorEx("bl").reversed();
    private final MotorEx backRightMotor = new MotorEx("br");


    private void init(){
        hardwareMap = ActiveOpMode.hardwareMap();
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

    public RobotV4(Robot.Alliance alliance){
        this.alliance = alliance;
    }

    public void initialize(){
        init();
    }

    public void buildCommands(){
        driveCommand = new ParallelGroup(
                sensor.update(),
                spindexer.spindexerUpdate(),
                ll.turretUpdate(OFFSET_ANGLE),
                hood.servoUpdate()
        );
        driveMotors = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX()
        );
        manager.setMode(BallManager.State.INTAKE);
        fireCommand = buildShot();
        liftCommand = lift.liftUpdate();
        rampDown = Ramp.INSTANCE.rampDown;
        arcSpinUp = arc.spinUpIdle();
    }


    public Command buildShot(){
        return new ParallelRaceGroup(
                spindexer.Fire(),
                arc.spinActive(),
                new Delay(5)
        );
    }

    public void systemStatesUpdate(){
        if(firstLoop){
            firstLoop = false;
            return;
        }
        else if(!driveMotors.isScheduled()) {
            driveMotors.update();
            driveMotors.schedule();
        }
        if(gamepad.left_trigger >= 0.15 && state != SystemStates.LIFT){
            intake.intake.setDirection(intake.intake.invertDirection(RobotConstantsV1.intakeDirection));
            intake.intake.setPower(RobotConstantsV1.INTAKE_POWER);
        }
        else if(state != SystemStates.LIFT) {
            intake.intake.setDirection(RobotConstantsV1.intakeDirection);
            intake.intake.setPower(RobotConstantsV1.INTAKE_POWER);
        }
        sensors.update();
        gamepad.update();
        if(gamepad.right_trigger > 0.15 && !fireCommand.isScheduled() && gamepad.isGamepadReady()){
            shoot();
        }
        if(gamepad.cross && gamepad.isGamepadReady()){
            setSystemState(SystemStates.LIFT);
            gamepad.resetTimer();
        }
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
                if(!fireCommand.isScheduled() && gamepad.isGamepadReady() && gamepad.right_trigger > 0.15 && !shooting) {
                    shoot();
                    gamepad.resetTimer();
                }
                break;
            case LIFT:
                if(driveCommand.isScheduled()){
                    driveCommand.cancel();
                }
                intake.setPower(0, RobotConstantsV1.intakeDirection);
                arc.disablePIDs();
                arc.arcShooter.setMotorPower(0);
                if(!liftCommand.isScheduled()){
                    liftCommand.schedule();
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

    public void setSystemState(SystemStates state){
        this.state = state;
    }

    public void log(){
        telemetryM.addLine("Commands running: " + CommandManager.INSTANCE.snapshot().size());
        telemetryM.addLine("Spindexer State: " + spindexer.currentSpindexerState.name());
        telemetryM.addLine("Arc velocity: " + arc.arcShooter.getVelocityRPM());
        telemetryM.addLine("Arc target: " + arc.arcShooter.getTargetRPM());
        LLResult result = ll.limelight.getLatestResult();
        if(result != null && result.isValid()) {
            telemetryM.addLine("LL distance: " + ll.limelight.getTargetDistance());
            telemetryM.addLine("Target X: " + result.getTx());
        }
        telemetryM.addData("Lift powers: (" + lift.left.getPower()+ ", " + lift.left2.getPower(), + lift.right.getPower() + ", " + lift.right2.getPower());
        telemetryM.update(telemetry);
    }
}
