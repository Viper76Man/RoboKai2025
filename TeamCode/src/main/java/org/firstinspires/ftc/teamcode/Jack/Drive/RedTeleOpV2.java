package org.firstinspires.ftc.teamcode.Jack.Drive;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Red TeleOpV2 [EXPERIMENTAL]")
//TODO: Test this!!!!
public class RedTeleOpV2 extends OpMode {
    public RobotV3 robotv3 = new RobotV3();
    public GamepadV1 gamepad = new GamepadV1();
    public TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    @Override
    public void init() {
        gamepad.init(gamepad1, 0.3);
        robotv3.init(hardwareMap, gamepad, Robot.Mode.TELEOP, Robot.Alliance.RED);
        telemetry.addLine("Alliance: " + Robot.Alliance.RED.name());
    }

    @Override
    public void loop() {
        robotv3.systemStatesUpdate();
        robotv3.log(telemetryM, telemetry);
        robotv3.sensor.log(telemetryM, telemetry);
        telemetryM.update(telemetry);
    }
}