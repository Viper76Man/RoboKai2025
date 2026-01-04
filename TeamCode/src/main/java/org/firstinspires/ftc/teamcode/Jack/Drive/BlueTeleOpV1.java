package org.firstinspires.ftc.teamcode.Jack.Drive;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Jack.Other.MultipleTelemetry;

@Disabled
public class BlueTeleOpV1 extends OpMode {
    public Robot robot = new Robot();
    public MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, PanelsTelemetry.INSTANCE.getTelemetry());
    @Override
    public void init() {
        robot.init(Robot.Mode.TELEOP, Robot.Alliance.BLUE, hardwareMap, telemetry, gamepad1);
    }

    @Override
    public void loop() {
        robot.systemStatesUpdate();
    }
}
