package org.firstinspires.ftc.teamcode.Jack.Drive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp
public class BlueTeleOpV4 extends NextFTCOpMode {
    public RobotV4 robotV4 = new RobotV4();

    @Override
    public void onInit(){
        robotV4.init(hardwareMap);
    }

    @Override
    public void onUpdate(){
        robotV4.systemStatesUpdate();
    }
}
