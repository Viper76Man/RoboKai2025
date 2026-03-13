package org.firstinspires.ftc.teamcode.Jack.Drive;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Jack.Motors.SpindexerMotorV2;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.DriveMotorsV2;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "BlueTeleOpV4 [EXPERIMENTAL]")
public class BlueTeleOpV4 extends NextFTCOpMode {
    public RobotV4 robotV4 = new RobotV4(Robot.Alliance.BLUE);
    public GamepadV1 gamepad = new GamepadV1();
    public DriveMotorsV2 drive = new DriveMotorsV2();
    public ElapsedTime loopTimer = new ElapsedTime();

    public double cycles = 0;

    public BlueTeleOpV4(){
        addComponents(
                BulkReadComponent.INSTANCE,
                new SubsystemComponent(SpindexerMotorV2.INSTANCE),
                new SubsystemComponent(robotV4),
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onInit(){
        gamepad.init(gamepad1, 0.3);
        robotV4.buildCommands();
    }


    @Override
    public void onUpdate() {
        gamepad.update();
        robotV4.log();
        robotV4.systemStatesUpdate();
        if (RobotConstantsV1.panelsEnabled) {
            PanelsTelemetry.INSTANCE.getTelemetry().addLine("Loop time: " + loopTimer.milliseconds());
        } else {
            telemetry.addLine("Loop time: " + loopTimer.milliseconds());
        }
        loopTimer.reset();
    }


}