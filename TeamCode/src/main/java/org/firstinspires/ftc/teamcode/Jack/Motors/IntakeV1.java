package org.firstinspires.ftc.teamcode.Jack.Motors;

import android.app.backup.BackupAgent;

import androidx.core.text.util.LocalePreferences;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.Jack.Other.MultipleTelemetry;

public class IntakeV1{
    public HardwareMap hardwareMap;
    public DcMotor motor;

    public enum IntakeState {
        IDLE,
        FORWARD,
        REVERSE
    }

    public void init(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        motor = this.hardwareMap.get(DcMotor.class, RobotConstantsV1.intakeMotorName);
        motor.setDirection(RobotConstantsV1.intakeDirection);
    }

    public void setPower(double power){
        motor.setPower(power);
    }

    public void setDirection(DcMotorSimple.Direction direction){
        motor.setDirection(direction);
    }

    public void switchDirection(){
        if(motor.getDirection() == DcMotorSimple.Direction.FORWARD){
            setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else {
            setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    public DcMotorSimple.Direction getDirection(){
        return motor.getDirection();
    }

    public void log(Telemetry telemetry){
        telemetry.addData("Intake Power: ", motor.getPower());
        telemetry.addData("Intake Motor Position: ", motor.getCurrentPosition());
        telemetry.addData("Intake Direction: " , motor.getDirection());
        telemetry.update();
    }

    public void log(MultipleTelemetry telemetry){
        telemetry.addData("Intake Power: ", motor.getPower());
        telemetry.addData("Intake Motor Position: ", motor.getCurrentPosition());
        telemetry.addData("Intake Direction: " , motor.getDirection());
        telemetry.update();
    }

    public double getCurrent(){
        return ((DcMotorEx) motor).getCurrent(CurrentUnit.AMPS);
    }

    public void setState(IntakeState state){
        switch (state){
            case IDLE:
                setPower(0.4);
                setDirection(RobotConstantsV1.intakeDirection);
                break;
            case REVERSE:
                setPower(0.85);
                setDirection(inverse(RobotConstantsV1.intakeDirection));
                break;
            case FORWARD:
                setPower(RobotConstantsV1.INTAKE_POWER);
                setDirection(RobotConstantsV1.intakeDirection);
                break;
        }

    }

    public static DcMotorSimple.Direction inverse(DcMotorSimple.Direction original){
        if(original == DcMotorSimple.Direction.FORWARD){
            return DcMotorSimple.Direction.REVERSE;
        }
        else {
            return DcMotorSimple.Direction.FORWARD;
        }
    }

}
