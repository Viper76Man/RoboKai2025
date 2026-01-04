package org.firstinspires.ftc.teamcode.Jack.Drive;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Jack.Other.LoggerV1;

@TeleOp(name = "Blue TeleOpV2 [EXPERIMENTAL]")
//TODO: Test this!!!!
public class BlueTeleOpV2 extends OpMode {
    public RobotV3 robotv3 = new RobotV3();
    public LoggerV1 logger = new LoggerV1();
    public GamepadV1 gamepad = new GamepadV1();
    public TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    @Override
    public void init() {
        logger.init(telemetry);
        gamepad.init(gamepad1, 0.3);
        robotv3.init(hardwareMap, gamepad, Robot.Mode.TELEOP, Robot.Alliance.BLUE);
        telemetry.addLine("Alliance: " + Robot.Alliance.BLUE.name());
    }

    @Override
    public void loop() {
        robotv3.systemStatesUpdate();
        robotv3.log(telemetryM, telemetry);
        robotv3.sensor.log(telemetryM, telemetry);
        telemetryM.update(telemetry);
    }
}