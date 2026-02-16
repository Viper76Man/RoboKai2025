package org.firstinspires.ftc.teamcode.Jack.Drive;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Jack.Subsystems.DriveMotorsV2;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.List;

import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp(name = "BlueTeleOpV4 [EXPERIMENTAL]")
public class BlueTeleOpV4 extends NextFTCOpMode {
    public RobotV4 robotV4 = new RobotV4();
    public GamepadV1 gamepad = new GamepadV1();
    public DriveMotorsV2 drive = new DriveMotorsV2();
    public ElapsedTime loopTimer = new ElapsedTime();

    public double cycles = 0;

    @Override
    public void onInit(){
        gamepad.init(gamepad1, 0.3);
        robotV4.init(hardwareMap, gamepad, Robot.Mode.TELEOP, Robot.Alliance.BLUE);
    }

    @Override
    public void onStartButtonPressed(){
        gamepad.update();
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