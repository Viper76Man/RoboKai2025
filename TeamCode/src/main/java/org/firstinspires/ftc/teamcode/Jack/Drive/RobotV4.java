package org.firstinspires.ftc.teamcode.Jack.Drive;

import android.provider.Settings;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Jack.Other.BallManager;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.ColorSensorV3;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.DriveMotorsV2;
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
    public ParallelGroup master;
    public IntakeMotorV2 intake = new IntakeMotorV2();
    public ColorSensorV3 sensor = new ColorSensorV3();
    public SpindexerV2 spindexer = new SpindexerV2();
    public BallManager manager = new BallManager();
    public void init(HardwareMap hardwareMap, GamepadV1 gamepadV1){
        drive = new DriveMotorsV2();
        intake = new IntakeMotorV2();
        drive.init(hardwareMap, gamepadV1);
        intake.init(hardwareMap);
        spindexer.init(manager);
        sensor.init(hardwareMap, manager);
    }

    public void buildCommand(){
        master = new ParallelGroup(drive.drive());
        master.and(
                spindexer.spindexerRun(),
                sensor.update(),
                intake.setPower(RobotConstantsV1.INTAKE_POWER, RobotConstantsV1.intakeDirection)
        );
    }

    public void systemStatesUpdate(){
        PanelsTelemetry.INSTANCE.getTelemetry().update(ActiveOpMode.telemetry());
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

}