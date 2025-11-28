package org.firstinspires.ftc.teamcode.Jack.Drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class BlueTeleOpV1 extends OpMode {
    public Robot robot = new Robot();
    @Override
    public void init() {
        robot.init(Robot.Mode.TELEOP, Robot.Alliance.BLUE, hardwareMap, telemetry, gamepad1);
    }

    @Override
    public void loop() {
        robot.systemStatesUpdate();
    }
}
