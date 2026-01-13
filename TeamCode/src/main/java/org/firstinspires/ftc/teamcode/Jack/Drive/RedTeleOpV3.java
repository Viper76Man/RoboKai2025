
package org.firstinspires.ftc.teamcode.Jack.Drive;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Red TeleOpV3 [EXPERIMENTAL]")
public class RedTeleOpV3 extends OpMode {
    public RobotV4 robotv4 = new RobotV4();
    public GamepadV1 gamepad = new GamepadV1();
    public TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    @Override
    public void init() {
        gamepad.init(gamepad1, 0.3);
        robotv4.init(hardwareMap, gamepad, Robot.Mode.TELEOP, Robot.Alliance.RED);
        telemetry.addLine("Alliance: " + Robot.Alliance.RED.name());
    }

    @Override
    public void loop() {
        robotv4.systemStatesUpdate();
        //robotv4.log(telemetryM, telemetry);
        robotv4.sensor.log(telemetryM, telemetry);
        telemetryM.update(telemetry);
    }
}