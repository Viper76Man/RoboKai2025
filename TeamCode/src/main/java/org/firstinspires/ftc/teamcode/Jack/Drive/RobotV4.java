package org.firstinspires.ftc.teamcode.Jack.Drive;

import android.provider.Settings;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Jack.Other.BallManager;
import org.firstinspires.ftc.teamcode.Jack.Other.BulkReadsTest;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.ArcMotorsV2;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.ColorSensorV3;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.DriveMotorsV2;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.FiringManager;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.FlickerSubsystem;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.IntakeMotorV2;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.IntakeV2;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.SpindexerV2;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class RobotV4 implements Subsystem {
    public DriveMotorsV2 drive = new DriveMotorsV2();
    public ParallelGroup intakeCommand, shootCommand, fireCommand, fireSingleCommand;
    public LED left1, right1, left2, right2;
    public IntakeMotorV2 intake = new IntakeMotorV2();
    public ColorSensorV3 sensor = new ColorSensorV3();
    public FlickerSubsystem flicker = new FlickerSubsystem();
    public SpindexerV2 spindexer = new SpindexerV2();
    public FiringManager firingManager = new FiringManager();
    public ArcMotorsV2 arcMotorsV2 = new ArcMotorsV2();
    public BallManager manager = new BallManager();
    public GamepadV1 gamepad = new GamepadV1();

    public boolean firedAlready = false;

    public enum SystemStates {
        START,
        BALL_1_INTAKE,
        BALL_2_INTAKE,
        BALL_3_INTAKE,
        SHOOT_ALL,
        SHOOT_SINGLE
    }

    public SystemStates state = SystemStates.START;


    public void init(HardwareMap hardwareMap, GamepadV1 gamepadV1){
        this.gamepad = gamepadV1;
        drive = new DriveMotorsV2();
        intake = new IntakeMotorV2();
        drive.init(hardwareMap, gamepadV1);
        intake.init(hardwareMap);
        spindexer.init(manager);
        flicker.init(spindexer.spindexer);
        sensor.init(hardwareMap, manager, spindexer.spindexer);
        arcMotorsV2.init(hardwareMap, Robot.Mode.TELEOP);
        left1 = hardwareMap.get(LED.class, "left");
        right1 = hardwareMap.get(LED.class, "right");
        left2 = hardwareMap.get(LED.class, "left2");
        right2 = hardwareMap.get(LED.class, "right2");
    }

    public void buildCommands(){
        intakeCommand = new ParallelGroup(drive.drive(),
                spindexer.spindexerRun(),
                sensor.update(),
                arcMotorsV2.spinUpIdle());
        firingManager.init(manager, flicker, spindexer.spindexer);
        shootCommand = new ParallelGroup(drive.drive());
        shootCommand.and(
                spindexer.spindexerRun(),
                arcMotorsV2.spinActive()
        );
        fireCommand = new ParallelGroup(firingManager.fireTriple());
        fireSingleCommand = new ParallelGroup(firingManager.fireSingle());
    }

    public void systemStatesUpdate(){
        gamepad.update();
        intake.setPower(RobotConstantsV1.INTAKE_POWER, RobotConstantsV1.intakeDirection).schedule();
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
                if(gamepad.isGamepadReady() && gamepad.circle){
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
                if(gamepad.isGamepadReady() && gamepad.circle){
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
                rpmUpdate();
                if(spindexer.spindexer.isSpindexerReady() && !firedAlready && gamepad.right_trigger >= 0.1 && gamepad.isGamepadReady()){
                    fireCommand.schedule();
                    firedAlready = true;
                    gamepad.resetTimer();
                }
                if(manager.mode == BallManager.State.INTAKE && firedAlready && fireCommand.isDone()){
                    setSystemState(SystemStates.START);
                    firedAlready = false;
                }
                break;
            case SHOOT_SINGLE:
                rpmUpdate();
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
            logCurrentCommands(PanelsTelemetry.INSTANCE.getTelemetry());
            flicker.flicker.log(PanelsTelemetry.INSTANCE.getTelemetry());
            PanelsTelemetry.INSTANCE.getTelemetry().addLine("Current ball: " + manager.getCurrentBall());
            PanelsTelemetry.INSTANCE.getTelemetry().addLine("Mode: "+ manager.mode);
            PanelsTelemetry.INSTANCE.getTelemetry().update(ActiveOpMode.telemetry());
        }
        else {
            logCurrentCommands(ActiveOpMode.telemetry());
        }
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


    public void rpmUpdate(){
        if(gamepad.right_bumper && gamepad.isGamepadReady()){
            arcMotorsV2.setTargetRPM(RobotConstantsV1.SHOOTER_TARGET_RPM);
            gamepad.resetTimer();
        }
        if(gamepad.left_bumper && gamepad.isGamepadReady()){
            arcMotorsV2.setTargetRPM(RobotConstantsV1.SHOOTER_FRONT_RPM);
            gamepad.resetTimer();
        }

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