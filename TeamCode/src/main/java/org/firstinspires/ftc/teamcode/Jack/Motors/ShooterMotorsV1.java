package org.firstinspires.ftc.teamcode.Jack.Motors;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterMotorsV1 {
    public HardwareMap hardwareMap;
    public DcMotor leftMotor;
    public DcMotor rightMotor;

    public void init(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        leftMotor = this.hardwareMap.get(DcMotor.class, "leftShooter");
        rightMotor = this.hardwareMap.get(DcMotor.class, "rightShooter");
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setLeftMotorPower(double power){
        leftMotor.setPower(power);
    }

    public void setRightMotorPower(double power){
        rightMotor.setPower(power);
    }
}
