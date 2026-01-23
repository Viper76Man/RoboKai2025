package org.firstinspires.ftc.teamcode.Jack.Drive;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp(name = "RedTeleOpV4 [EXPERIMENTAL]")
public class RedTeleOpV4 extends NextFTCOpMode {
    public RobotV4 robotV4 = RobotV4.INSTANCE;
    public MecanumDriveOnly drive = new MecanumDriveOnly();
    public GamepadV1 gamepadv1 = new GamepadV1();

    @Override
    public void onInit(){
        addComponents(
                new SubsystemComponent(robotV4)
        );
        robotV4.init(hardwareMap, RobotV4.Period.TELEOP);
        gamepadv1.init(gamepad1, 0.3);
        drive.init(hardwareMap, gamepadv1);
    }

    @Override
    public void onUpdate(){
        PanelsTelemetry.INSTANCE.getTelemetry().update(telemetry);
        robotV4.log(telemetry);
        gamepadv1.update();
        drive.drive();
        robotV4.systemStatesUpdate();
    }
}
