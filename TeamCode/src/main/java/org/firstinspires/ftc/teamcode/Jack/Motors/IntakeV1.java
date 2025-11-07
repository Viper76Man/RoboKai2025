package org.firstinspires.ftc.teamcode.Jack.Motors;

import android.app.backup.BackupAgent;

import androidx.core.text.util.LocalePreferences;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;

public class IntakeV1{
    public HardwareMap hardwareMap;
    public DcMotor motor;

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

    public void log(Telemetry telemetry){
        telemetry.addData("Intake Power: ", motor.getPower());
        telemetry.addData("Intake Motor Position: ", motor.getCurrentPosition());
        telemetry.addData("Intake Direction: " , motor.getDirection());
        telemetry.update();
    }

}
