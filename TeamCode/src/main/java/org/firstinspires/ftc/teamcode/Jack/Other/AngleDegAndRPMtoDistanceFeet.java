package org.firstinspires.ftc.teamcode.Jack.Other;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;

public class AngleDegAndRPMtoDistanceFeet {
    public double vi, vx, vy, tTotal, distance = 0;
    public double calculate(double rpm, Telemetry telemetry){
        double angleDeg = RobotConstantsV1.SHOOTER_ANGLE_DEG;
        double diameterMeters = 0.096; //m
        double metersPerRotation = Math.PI * diameterMeters;//m
        double rotationsPerSec = rpm / 60.0;
        telemetry.addData("RPS: ", rotationsPerSec);
        double metersPerSec = rotationsPerSec * metersPerRotation;
        telemetry.addData("MPS: ", metersPerSec);
        //11.87 in per rotation = 0.99 ft
        vi = metersPerSec;
        telemetry.addData("Vi: ", vi);
        vx = vi * Math.cos(angleDeg);
        telemetry.addData("Vx: ", vx);
        vy = vi * Math.sin(angleDeg);
        telemetry.addData("Vy: ", vy);
        tTotal = (2 * vy) / -9.81;
        telemetry.addData("T-Total: ", tTotal);
        distance = (vx * tTotal) * 3.281;
        return distance;
    }
}
