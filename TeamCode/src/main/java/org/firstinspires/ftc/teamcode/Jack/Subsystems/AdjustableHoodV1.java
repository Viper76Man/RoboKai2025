package org.firstinspires.ftc.teamcode.Jack.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    public updateServo servoUpdate(){
        return new updateServo();
    }

    public class updateServo extends Command {

        public Gamepad gamepad = ActiveOpMode.gamepad1();
        public ElapsedTime buttonTimer = new ElapsedTime();
        @Override
        public void update(){

            if(buttonTimer.seconds() > 0.3 && gamepad.dpad_left){
                hoodServo.goToDeg(hoodServo.currentDeg - 1);
                buttonTimer.reset();
            }
            if(buttonTimer.seconds() > 0.3 && gamepad.dpad_right){
                hoodServo.goToDeg(hoodServo.currentDeg + 1);
                buttonTimer.reset();
            }
        }

        @Override
        public boolean isDone() {
            return false;
        }
    }

}
