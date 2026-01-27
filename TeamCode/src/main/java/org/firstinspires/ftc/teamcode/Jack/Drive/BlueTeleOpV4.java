package org.firstinspires.ftc.teamcode.Jack.Drive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp(name = "BlueTeleOpV4 [EXPERIMENTAL]")
public class BlueTeleOpV4 extends NextFTCOpMode {
    public RobotV4 robotV4 = new RobotV4();
    public GamepadV1 gamepad = new GamepadV1();

    @Override
    public void onInit(){
        gamepad.init(gamepad1, 0.3);
        robotV4.init(hardwareMap, gamepad);
    }

    @Override
    public void onStartButtonPressed(){
        robotV4.buildCommands();
    }

    @Override
    public void onUpdate(){
        robotV4.systemStatesUpdate();
    }


}