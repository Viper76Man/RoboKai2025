package org.firstinspires.ftc.teamcode.Jack.Drive;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RobotConstantsV1 {
    //Drive motors
    public static String frontLeft = "fl";
    public static String frontRight = "fr";
    public static String backLeft = "bl";
    public static String backRight = "br";
    public static DcMotorSimple.Direction frontLeftDirection = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction backLeftDirection = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction frontRightDirection = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction backRightDirection = DcMotorSimple.Direction.FORWARD;

    //Config names
    public static String pinpointName = "pinpoint";

    //AUTONOMOUS-------------------------------------------------------------------------------------
    //Auto, needs measuring, in inches
    public static double forwardPodY = -5;
    public static double strafePodX = 0.5;
    public static DistanceUnit podsMeasurementUnit = DistanceUnit.INCH;

    public static double xVelocity = 49.88;
    public static double yVelocity = 49.88;

    //Tuning values
    public static double robotMassKG = 10;
    public static double forwardZeroPowerAcceleration = -10;
    public static double lateralZeroPowerAcceleration = 10;
    public static double centripetalScaling = 0.005;

    //PIDs
    public static PIDFCoefficients translationalPIDCoefficients = new PIDFCoefficients(0.3, 0, 0.01, 0);
    public static PIDFCoefficients headingPIDFCoefficients = new PIDFCoefficients(2, 0, 0.01, 0);
    public static FilteredPIDFCoefficients drivePIDCoefficients = new FilteredPIDFCoefficients(0.025, 0, 0, 0.6, 0);
}
