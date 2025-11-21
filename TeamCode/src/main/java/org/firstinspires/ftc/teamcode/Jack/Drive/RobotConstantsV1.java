package org.firstinspires.ftc.teamcode.Jack.Drive;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class RobotConstantsV1 {
    //Drive motors
    public static String frontLeft = "fl"; //Control Hub Port 0
    public static String frontRight = "fr"; //Control Hub Port 1
    public static String backLeft = "bl"; //Control Hub Port 2
    public static String backRight = "br"; //Control Hub Port 3
    public static DcMotorSimple.Direction frontLeftDirection = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction backLeftDirection = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction frontRightDirection = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction backRightDirection = DcMotorSimple.Direction.FORWARD;
    public static boolean useBrakeInTeleOp = true;
    //Config names
    public static String pinpointName = "pinpoint";
    //HARDWARE--------------------------------------------------------------------------------------
    public static String arcShooterName = "arcMotor"; //Expansion Hub Port 1
    public static String intakeMotorName = "intake"; //Expansion Hub Port 0
    public static String flickerServoName = "flicker"; //Control Hub Servos Port 1
    public static String storageServoName = "storageServo"; //Control Hub Servos Port 0

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

    //TELEOP-------------------------------------------------------------------------------------------------------------------------
    public static PIDCoefficients arcPIDs = new PIDCoefficients(0.00035, 0, 0.00000003);
    public static double SHOOTER_TARGET_RPM = 6000;
    public static double INTAKE_POWER = 1;
    public static DcMotorSimple.Direction intakeDirection = DcMotorSimple.Direction.REVERSE;
    public static int SHOOTER_PPR = 28;
    public static double SHOOTER_UPDATE_TIME_SECONDS = 0.15;
    public static double FLICKER_SERVO_UP = 1;
    public static double FLICKER_SERVO_DOWN = 0;

    public static double STORAGE_BALL_1 = 0.1;
    public static double STORAGE_BALL_2 = 0.45;
    public static double STORAGE_BALL_3 = 0.85;
    //TUNERS---------------------------------------------------------------------------------------------------------------------------
    public static double defaultShooterRPM = SHOOTER_TARGET_RPM;
    public static double velocityUpStep = 0.1;
    public static double velocityDownStep = 0.1;

    public static double storageServoStep = 0.05;
}
