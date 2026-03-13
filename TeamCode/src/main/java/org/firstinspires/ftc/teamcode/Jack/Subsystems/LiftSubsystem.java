package org.firstinspires.ftc.teamcode.Jack.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class LiftSubsystem implements Subsystem {
    public CRServo left, right, left2, right2;
    public enum state {
        INACTIVE,
        LIFT,
        HOLD
    }
    public state liftState = state.INACTIVE;
    public ElapsedTime timer = new ElapsedTime();

    public void init(){
        left = ActiveOpMode.hardwareMap().get(CRServo.class, "leftLift1");
        left2 = ActiveOpMode.hardwareMap().get(CRServo.class, "leftLift2"); //back
        right = ActiveOpMode.hardwareMap().get(CRServo.class, "rightLift");
        right2 = ActiveOpMode.hardwareMap().get(CRServo.class, "rightLift2");
    }

    public void updateLift(){
        if(ActiveOpMode.gamepad1().dpad_left){
            left.setPower(-1);
            left2.setPower(1);
        }
        else {
            left.setPower(0);
            left2.setPower(0);
        }
        if(ActiveOpMode.gamepad1().dpad_right){
            right.setPower(1);
            right2.setPower(-1);
        }
        else {
            right.setPower(0);
            right2.setPower(0);
        }
    }

    public Command liftUpdate(){
        return new Command() {
            public void update(){
                updateLift();
            }

            @Override
            public boolean isDone() {
                return false;
            }
        };
    }
    public void setPowers(double power){
        left.setPower(-power); //right
        left2.setPower(power); //LEFT
        right2.setPower(-power); // RIGHT SIDE
        right.setPower(power); //right
    }

    public void setState(state newState){
        this.liftState = newState;
    }

}
