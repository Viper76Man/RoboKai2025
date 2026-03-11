package org.firstinspires.ftc.teamcode.Coach.mechanisims;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class TestBench {
    private double ticksPerRev; // revolution

    private IMU imu;

    public void init(HardwareMap hwMap) {
        imu = hwMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

        imu.initialize(new IMU.Parameters(RevOrientation));

    }

    public YawPitchRollAngles getOrientation(){
        return imu.getRobotYawPitchRollAngles();
    }

    public double getDistanceFromTag(double ta) {
        if (ta <= 0) {
            return Double.POSITIVE_INFINITY; // invalid result
        }

        double scale = 30665.95; // calculated from table
        double distance = Math.sqrt(scale / ta); //simplified
        return distance; // in cm
    }

}