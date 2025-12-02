package org.firstinspires.ftc.teamcode.Jack.Drive;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class RedTeleOpV1 extends OpMode {
    public Robot robot = new Robot();
    @Override
    public void init() {
        robot.init(Robot.Mode.TELEOP, Robot.Alliance.RED, hardwareMap, telemetry, gamepad1);
        robot.follower.setPose(new Pose(72, 72, Math.toRadians(90)));
    }

    @Override
    public void loop() {
        robot.systemStatesUpdate();
    }
}
