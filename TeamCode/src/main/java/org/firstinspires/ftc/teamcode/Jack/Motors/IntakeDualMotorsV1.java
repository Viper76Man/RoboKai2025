package org.firstinspires.ftc.teamcode.Jack.Motors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeDualMotorsV1 {
    public HardwareMap hardwareMap;
    public DcMotor left;
    public DcMotor right;
    public void init(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        left = hardwareMap.get(DcMotor.class, "leftIntake");
        right = hardwareMap.get(DcMotor.class, "rightIntake");
        right.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void setPowers(double power) {
        left.setPower(power);
        right.setPower(power);
    }

    public void switchDirections(){
        if(left.getDirection() == DcMotorSimple.Direction.FORWARD) {
            left.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else {
            left.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        if(right.getDirection() == DcMotorSimple.Direction.FORWARD) {
            right.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else {
            right.setDirection(DcMotorSimple.Direction.FORWARD);
        }

    }
}
