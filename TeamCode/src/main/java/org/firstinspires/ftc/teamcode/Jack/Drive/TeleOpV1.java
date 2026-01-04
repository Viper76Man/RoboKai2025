package org.firstinspires.ftc.teamcode.Jack.Drive;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Jack.Other.LoggerV1;
import org.firstinspires.ftc.teamcode.Jack.Other.PanelsToolkit;

@Disabled
//TODO: Test this!!!!
public class TeleOpV1 extends OpMode {
    public RobotV2 robotv2 = new RobotV2();
    public LoggerV1 logger = new LoggerV1();
    public GamepadV1 gamepad = new GamepadV1();
    public TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    @Override
    public void init() {
        logger.init(telemetry);
        gamepad.init(gamepad1, 0.3);
        Robot.Alliance alliance = logger.readSideFromFile();

        if(alliance != null) {
            robotv2.init(RobotV2.Mode.TELEOP, alliance, hardwareMap, gamepad, telemetry);
            telemetry.addLine("Alliance: " + alliance.name());
        }
        else {
            robotv2.init(RobotV2.Mode.TELEOP, Robot.Alliance.TEST, hardwareMap, gamepad, telemetry);
            telemetry.addLine("Alliance: TEST");
        }
    }

    @Override
    public void loop() {
        robotv2.systemStatesUpdate();
        robotv2.log(telemetryM, telemetry);
        robotv2.sensor.log(telemetryM, telemetry);
        telemetryM.update(telemetry);

    }
}