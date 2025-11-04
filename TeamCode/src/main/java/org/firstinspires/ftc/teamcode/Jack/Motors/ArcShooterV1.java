package org.firstinspires.ftc.teamcode.Jack.Motors;

import android.app.backup.BackupAgent;

import androidx.core.text.util.LocalePreferences;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;

public class ArcShooterV1 {
    public HardwareMap hardwareMap;
    public DcMotor motor;
    public DcMotorEx shooter;
    public double velocity = 0;


    public void init(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        motor = this.hardwareMap.get(DcMotor.class, RobotConstantsV1.arcShooterName);
        shooter = (DcMotorEx) motor;
        setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setMotorPower(double power){
        motor.setPower(power);
    }

    public void setDirection(DcMotorSimple.Direction direction){
        shooter.setDirection(direction);
    }

    public void switchDirection(){
        if(shooter.getDirection() == DcMotorSimple.Direction.FORWARD){
            setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else {
            setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    public void setTargetVelocity(double velocity_){
        velocity = velocity_;
        updateVelocity();
    }

    public double getTargetVelocity(){
        return velocity;
    }
    public double getVelocity(){
        return shooter.getVelocity();
    }

    public boolean ready(){
        return getVelocity() >= RobotConstantsV1.SHOOTER_TARGET_VELOCITY;
    }

    private void updateVelocity(){
        shooter.setVelocity(velocity);
    }

    public void log(Telemetry telemetry){
        telemetry.addData("Arc Motor Velocity: ", velocity);
        telemetry.addData("Arc Motor Position: ", shooter.getCurrentPosition());
        telemetry.addData("Ready? : ", ready());
        telemetry.update();
    }

}
