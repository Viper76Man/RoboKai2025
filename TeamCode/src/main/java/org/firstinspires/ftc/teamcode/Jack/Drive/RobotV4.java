package org.firstinspires.ftc.teamcode.Jack.Drive;

import android.provider.Settings;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    public ParallelGroup intakeCommand, shootCommand, flickerCommand, fireCommand;
    public IntakeMotorV2 intake = new IntakeMotorV2();
    public ColorSensorV3 sensor = new ColorSensorV3();
    public FlickerSubsystem flicker = new FlickerSubsystem();
    public SpindexerV2 spindexer = new SpindexerV2();
    public FiringManager firingManager = new FiringManager();
    public ArcMotorsV2 arcMotorsV2 = new ArcMotorsV2();
    public BallManager manager = new BallManager();

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
    public int lastFiredBall = 0;



    public void init(HardwareMap hardwareMap, GamepadV1 gamepadV1){
        drive = new DriveMotorsV2();
        intake = new IntakeMotorV2();
        drive.init(hardwareMap, gamepadV1);
        intake.init(hardwareMap);
        flicker.init();
        spindexer.init(manager);
        sensor.init(hardwareMap, manager, spindexer.spindexer);
        arcMotorsV2.init(hardwareMap, Robot.Mode.TELEOP);
    }

    public void buildCommands(){
        intakeCommand = new ParallelGroup(drive.drive(),
                spindexer.spindexerRun(),
                sensor.update(),
                arcMotorsV2.spinUpIdle());
        flickerCommand = new ParallelGroup(flicker.fire());
        firingManager.init(manager, flickerCommand, spindexer.spindexer);
        shootCommand = new ParallelGroup(drive.drive());
        shootCommand.and(
                spindexer.spindexerRun(),
                arcMotorsV2.spinActive()
        );
        fireCommand = new ParallelGroup(firingManager.fire());
    }

    public void systemStatesUpdate(){
        PanelsTelemetry.INSTANCE.getTelemetry().addLine("Current ball: " + manager.getCurrentBall());
        PanelsTelemetry.INSTANCE.getTelemetry().addLine("Mode: "+ manager.mode);
        PanelsTelemetry.INSTANCE.getTelemetry().update(ActiveOpMode.telemetry());
        intake.setPower(RobotConstantsV1.INTAKE_POWER, RobotConstantsV1.intakeDirection).schedule();
        switch (state){
            case START:
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
                }
                break;
            case SHOOT_ALL:
                if(spindexer.spindexer.isSpindexerReady() && !firedAlready && ActiveOpMode.gamepad1().right_trigger >= 0.15){
                    fireCommand.schedule();
                    firedAlready = true;
                }
                if(manager.mode == BallManager.State.INTAKE){
                    setSystemState(SystemStates.BALL_1_INTAKE);
                }
                break;

        }
    }


    public void log(){
        if(RobotConstantsV1.panelsEnabled){
            logCurrentCommands(PanelsTelemetry.INSTANCE.getTelemetry());
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

}