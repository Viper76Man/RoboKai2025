package org.firstinspires.ftc.teamcode.Jack.Drive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Jack.Subsystems.DriveMotorsV2;

import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp(name = "BlueTeleOpV4 [EXPERIMENTAL]")
public class BlueTeleOpV4 extends NextFTCOpMode {
    public RobotV4 robotV4 = new RobotV4();
    public GamepadV1 gamepad = new GamepadV1();
    public DriveMotorsV2 drive = new DriveMotorsV2();

    @Override
    public void onInit(){
        gamepad.init(gamepad1, 0.3);
        robotV4.init(hardwareMap, gamepad, Robot.Mode.TELEOP);
        drive = new DriveMotorsV2();
    }

    @Override
    public void onStartButtonPressed(){
        drive.drive().run();
        robotV4.buildCommands();
    }

    @Override
    public void onUpdate(){
        robotV4.log();
        robotV4.systemStatesUpdate();
    }


}