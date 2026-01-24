package org.firstinspires.ftc.teamcode.Jack.Subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Jack.Motors.IntakeV1;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
public class IntakeMotorV2 implements Subsystem {
    public IntakeV1 intake = new IntakeV1();

    public void init(HardwareMap hardwareMap){
        intake.init(hardwareMap);
    }

    public setPowerAndDirection setPower(double power, DcMotorSimple.Direction direction){
        return new setPowerAndDirection(power, direction);
    }

    public class setPowerAndDirection extends Command {
        public double targetPower;

        public setPowerAndDirection(double power, DcMotorSimple.Direction direction){
            intake.setDirection(direction);
            intake.setPower(power);
            targetPower = power;
        }

        @Override
        public boolean isDone() {
            return intake.motor.getPower() == targetPower;
        }
    }
}