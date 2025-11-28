package org.firstinspires.ftc.teamcode.Jack.Other;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robocol.RobocolConfig;

import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;

@TeleOp
public class MathTest extends OpMode {
    public AngleDegAndRPMtoDistanceFeet calculator = new AngleDegAndRPMtoDistanceFeet();
    @Override
    public void init() {
    }

    @Override
    public void loop() {
        double rpm = RobotConstantsV1.SHOOTER_TARGET_RPM;
        double distance =  calculator.calculate(rpm, telemetry);
        telemetry.addLine("Distance with " + rpm + ": " + distance);
    }
}
