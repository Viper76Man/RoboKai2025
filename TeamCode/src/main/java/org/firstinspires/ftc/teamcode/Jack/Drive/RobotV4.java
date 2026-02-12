package org.firstinspires.ftc.teamcode.Jack.Drive;

import android.provider.Settings;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Jack.Camera.Limelight3A.LimelightV1;
import org.firstinspires.ftc.teamcode.Jack.Other.BallManager;
import org.firstinspires.ftc.teamcode.Jack.Other.BulkReadsTest;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.AdjustableHoodV1;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.ArcMotorsV2;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.ColorSensorV3;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.DriveMotorsV2;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.FiringManager;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.FlickerSubsystem;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.IntakeMotorV2;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.IntakeV2;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.SpindexerV2;
import org.firstinspires.ftc.teamcode.R;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class RobotV4 implements Subsystem { ;
    public ParallelGroup intakeCommand, shootCommand, fireCommand, fireSingleCommand;
    public LED left1, right1, left2, right2;
    public IntakeMotorV2 intake = new IntakeMotorV2();
    public ColorSensorV3 sensor = new ColorSensorV3();
    public FlickerSubsystem flicker = new FlickerSubsystem();
    public SpindexerV2 spindexer = new SpindexerV2();
    public LimelightSubsystem ll = new LimelightSubsystem();
    public AdjustableHoodV1 hood = new AdjustableHoodV1();
    public FiringManager firingManager = new FiringManager();
    public ArcMotorsV2 arcMotorsV2 = new ArcMotorsV2();
    public BallManager manager = new BallManager();
    public GamepadV1 gamepad = new GamepadV1();
    public DriveMotorsV2 drive = new DriveMotorsV2();

    public boolean firedAlready = false;
    public boolean firstLoop = true;

    public double OFFSET_ANGLE;

    public enum SystemStates {
        START,
        BALL_1_INTAKE,
        BALL_2_INTAKE,
        BALL_3_INTAKE,
        SHOOT_ALL,
        SHOOT_SINGLE
    }

    public SystemStates state = SystemStates.START;
    public Robot.Mode mode = Robot.Mode.TELEOP;


    public void init(HardwareMap hardwareMap, GamepadV1 gamepadV1, Robot.Mode mode, Robot.Alliance alliance){
        this.mode = mode;
        this.gamepad = gamepadV1;
        intake = new IntakeMotorV2();
        drive = new DriveMotorsV2();
        intake.init(hardwareMap);
        spindexer.init(manager);
        drive.init(hardwareMap, gamepadV1);
        hood.init();
        ll.init();
        flicker.init(spindexer.spindexer);
        sensor.init(hardwareMap, manager, spindexer.spindexer);
        arcMotorsV2.init(hardwareMap, Robot.Mode.TELEOP);
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
        intakeCommand = new ParallelGroup(
                drive.drive(),
                spindexer.spindexerRun(),
                sensor.update(),
                arcMotorsV2.spinUpIdle(),
                ll.turretUpdate(OFFSET_ANGLE),
                hood.servoUpdate()
        );
        firingManager.init(manager, flicker, spindexer.spindexer);
        shootCommand = new ParallelGroup(
                arcMotorsV2.spinActive()
        );
        fireCommand = new ParallelGroup(firingManager.fireTriple(mode, arcMotorsV2));
        fireSingleCommand = new ParallelGroup(firingManager.fireSingle(arcMotorsV2));
    }


    public void systemStatesUpdate(){
        if(firstLoop){
            flicker.fire().run();
            firstLoop = false;
        }
        gamepad.update();
        if(gamepad.left_trigger > 0.15) {
            intake.setPower(RobotConstantsV1.INTAKE_POWER, intake.intake.invertDirection(RobotConstantsV1.intakeDirection)).schedule();
        }
        else {
            intake.setPower(RobotConstantsV1.INTAKE_POWER, RobotConstantsV1.intakeDirection).schedule();
        }
        switch (state){
            case START:
                redLED();
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
                if(gamepad.isGamepadReady() && gamepad.circle && mode == Robot.Mode.TELEOP){
                    gamepad.resetTimer();
                    manager.setCurrentBall(1);
                    setSystemState(SystemStates.SHOOT_SINGLE);
                }
                break;
            case BALL_2_INTAKE:
                if ((sensor.isPurple() || sensor.isGreen()) && sensor.sensor.getNormalizedRGB().green >= 0.03) {
                    manager.setBall2(sensor.sensor.getCurrent());
                    manager.setCurrentBall(3);
                    sensor.clear();
                    setSystemState(SystemStates.BALL_3_INTAKE);
                }
                if(gamepad.isGamepadReady() && gamepad.circle && mode == Robot.Mode.TELEOP){
                    gamepad.resetTimer();
                    manager.setCurrentBall(1);
                    setSystemState(SystemStates.SHOOT_SINGLE);
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
                if(readyForTriple() && gamepad.right_trigger >= 0.1 && gamepad.isGamepadReady() && mode == Robot.Mode.TELEOP){
                    fireTriple();
                }
                //TODO: auto using command system
                if(manager.mode == BallManager.State.INTAKE && firedAlready && fireCommand.isDone()){
                    setSystemState(SystemStates.START);
                    firedAlready = false;
                }
                break;
            case SHOOT_SINGLE:
                if(spindexer.spindexer.isSpindexerReady() && !firedAlready && gamepad.right_trigger > 0.15 && gamepad.isGamepadReady()){
                    fireSingleCommand.schedule();
                    firedAlready = true;
                    gamepad.resetTimer();
                }
                if(manager.mode == BallManager.State.INTAKE && firedAlready && fireSingleCommand.isDone()){
                    firedAlready = false;
                }
                break;
        }
    }


    public void log(){
        if(RobotConstantsV1.panelsEnabled){
            ActiveOpMode.telemetry().setMsTransmissionInterval(150);
            hood.hoodServo.log(PanelsTelemetry.INSTANCE.getTelemetry());
            PanelsTelemetry.INSTANCE.getTelemetry().addLine("Target Shooter RPM: " + arcMotorsV2.arcShooter.getTargetRPM());
            PanelsTelemetry.INSTANCE.getTelemetry().addLine("Shooter Current RPM: " + arcMotorsV2.arcShooter.getVelocityRPM());
            PanelsTelemetry.INSTANCE.getTelemetry().addLine("Current ball: " + manager.getCurrentBall());
            PanelsTelemetry.INSTANCE.getTelemetry().addLine("Turret error: " + (OFFSET_ANGLE + ll.turret.cameraTx));
            PanelsTelemetry.INSTANCE.getTelemetry().addLine("Turret power: " + ll.turret.power_);
            PanelsTelemetry.INSTANCE.getTelemetry().update(ActiveOpMode.telemetry());
        }
        else {
            arcMotorsV2.arcShooter.log(ActiveOpMode.telemetry());
            ActiveOpMode.telemetry().addLine("Current ball: " + manager.getCurrentBall());
            ActiveOpMode.telemetry().addLine("Mode: "+ manager.mode);
        }
    }

    public void setSystemState(SystemStates state){
        this.state = state;
    }


    public void fireTriple(){
        fireCommand.schedule();
        firedAlready = true;
        gamepad.resetTimer();
    }

    public boolean readyForTriple(){
        return spindexer.spindexer.isSpindexerReady() && !firedAlready;
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

}