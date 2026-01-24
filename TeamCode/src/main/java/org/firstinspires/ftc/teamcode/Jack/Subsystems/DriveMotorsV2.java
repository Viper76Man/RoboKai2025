package org.firstinspires.ftc.teamcode.Jack.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Jack.Drive.GamepadV1;
import org.firstinspires.ftc.teamcode.Jack.Drive.MecanumDriveOnly;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class DriveMotorsV2 implements Subsystem {
    public MecanumDriveOnly drive_ = new MecanumDriveOnly();


    public void init(HardwareMap hardwareMap, GamepadV1 gamepadV1){
        drive_.init(hardwareMap, gamepadV1);
    }


    public Drive drive(){
        return new Drive();
    }

    public class Drive extends Command{
        @Override
        public void update(){
            drive_.drive();
        }

        @Override
        public boolean isDone() {
            return ActiveOpMode.isStopRequested();
        }
    }
}
