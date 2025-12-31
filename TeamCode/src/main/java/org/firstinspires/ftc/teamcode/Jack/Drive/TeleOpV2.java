package org.firstinspires.ftc.teamcode.Jack.Drive;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware;
import org.firstinspires.ftc.teamcode.Jack.Other.LoggerV1;

@TeleOp(name = "TeleOpV2 [EXPERIMENTAL]")
//TODO: Test this!!!!
public class TeleOpV2 extends OpMode {
    public RobotV3 robotv3 = new RobotV3();
    public LoggerV1 logger = new LoggerV1();
    public GamepadV1 gamepad = new GamepadV1();
    public TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    @Override
    public void init() {
        logger.init(telemetry);
        gamepad.init(gamepad1, 0.3);
        Robot.Alliance alliance = logger.readSideFromFile();

        if(alliance != null) {
            robotv3.init(hardwareMap, gamepad, Robot.Mode.TELEOP, alliance);
            telemetry.addLine("Alliance: " + alliance.name());
        }
        else {
            robotv3.init(hardwareMap, gamepad, Robot.Mode.TELEOP, Robot.Alliance.TEST);
            telemetry.addLine("Alliance: TEST");
        }
    }

    @Override
    public void loop() {
        robotv3.systemStatesUpdate();
        robotv3.log(telemetryM, telemetry);
        robotv3.sensor.log(telemetryM, telemetry);
        telemetryM.update(telemetry);

    }
}