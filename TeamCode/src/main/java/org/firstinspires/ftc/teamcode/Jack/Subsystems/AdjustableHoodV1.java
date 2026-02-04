package org.firstinspires.ftc.teamcode.Jack.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Jack.Servos.AdjustableHoodServo;


import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class AdjustableHoodV1 implements Subsystem {
    public AdjustableHoodServo hoodServo = new AdjustableHoodServo();
    public void init(){
        hoodServo.init(ActiveOpMode.hardwareMap());
    }

    public class setPosition extends Command{
        public setPosition(double pos){
            hoodServo.setPos(pos);
        }


        @Override
        public boolean isDone() {
            return false;
        }
    }
}
