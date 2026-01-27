package org.firstinspires.ftc.teamcode.Jack.Subsystems;

import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.Jack.Servos.FlickerServoV2;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class FlickerSubsystem implements Subsystem {
    public FlickerServoV2 flicker = new FlickerServoV2();

    public void init(){
        flicker.init(ActiveOpMode.hardwareMap(), RobotConstantsV1.flickerServoName);
    }

    public FlickUp fire(){
        return new FlickUp();
    }


    public class FlickUp extends Command {
        public boolean done = false;
        public boolean flickedAlready = false;

        @Override
        public void update(){
            if(!flickedAlready){
                flicker.setState(FlickerServoV2.State.DOWN);
                flickedAlready = true;
            }
            else if(flicker.getState() == FlickerServoV2.State.DOWN){
                done = true;
            }
        }

        @Override
        public boolean isDone() {
            return done;
        }
    }
}
