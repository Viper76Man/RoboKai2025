package org.firstinspires.ftc.teamcode.Jack.Drive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Jack.Other.LoggerV1;

@TeleOp(name = "TeleOpV1 [EXPERIMENTAL]")
//TODO: Run gradle update
public class TeleOpV1 extends OpMode {
    public Robot robot = new Robot();
    public LoggerV1 logger = new LoggerV1();
    @Override
    public void init() {
        logger.init(telemetry);
        Robot.Alliance alliance = logger.readSideFromFile();
        if(alliance != null) {
            robot.init(Robot.Mode.TELEOP, alliance, hardwareMap, telemetry, gamepad1);
            telemetry.addLine("Alliance: " + alliance.name());
        }
        else {
            robot.init(Robot.Mode.TELEOP, Robot.Alliance.TEST, hardwareMap, telemetry, gamepad1);
            telemetry.addLine("Alliance: TEST");
        }
    }

    @Override
    public void loop() {
        robot.systemStatesUpdate();
    }
}