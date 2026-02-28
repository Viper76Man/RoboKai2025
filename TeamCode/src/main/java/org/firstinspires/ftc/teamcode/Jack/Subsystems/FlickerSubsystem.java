package org.firstinspires.ftc.teamcode.Jack.Subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.SpindexerMotorV1;
import org.firstinspires.ftc.teamcode.Jack.Servos.FlickerServoV2;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class FlickerSubsystem implements Subsystem {
    public FlickerServoV2 flicker = new FlickerServoV2();
    public SpindexerMotorV1 spindexer;

    public void init(SpindexerMotorV1 spindexer){
        flicker.init(ActiveOpMode.hardwareMap(), RobotConstantsV1.flickerServoName);
        this.spindexer = spindexer;
    }

    public FlickUp fire(){
        return new FlickUp();
    }


    public class FlickUp extends Command {
        public boolean done = false;

        @Override
        public void update(){
            flicker.flicker.setPosition(RobotConstantsV1.FLICKER_SERVO_UP);
            done = true;
        }

        @Override
        public boolean isDone() {
            return done;
        }
    }
}
