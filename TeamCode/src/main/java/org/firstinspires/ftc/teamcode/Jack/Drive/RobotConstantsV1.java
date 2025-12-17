package org.firstinspires.ftc.teamcode.Jack.Drive;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.CanvasRotation;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Jack.Other.RGB;

@Configurable
public class RobotConstantsV1 {
    //Drive motors
    public static String frontLeft = "fl"; //Control Hub Port 0
    public static String frontRight = "fr"; //Control Hub Port 1
    public static String backLeft = "bl"; //Control Hub Port 2
    public static String backRight = "br"; //Control Hub Port 3
    public static DcMotorSimple.Direction frontLeftDirection = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction backLeftDirection = DcMotorSimple.Direction.FORWARD;
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
    public static String colorSensor1 = "colorSensor1"; //I2C port 0
    public static String spindexerMotorName = "spindexer"; //Expansion Hub Port 2

    //AUTONOMOUS-------------------------------------------------------------------------------------
    //Auto, needs measuring, in inches
    public static double forwardPodY = 7;
    public static double strafePodX = 4;
    public static DistanceUnit podsMeasurementUnit = DistanceUnit.INCH;

    public static int degreeToleranceCamera = 5;
    public static double shotAngleBlueDegrees = 70;
    public static double shotAngleRedDegrees = 110;

    public static GoBildaPinpointDriver.EncoderDirection forwardPodDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection lateralPodDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;

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
    public static PIDFCoefficients arcPIDs = new PIDFCoefficients(0.0000001, 0, 0, 0.000197);
    public static PIDCoefficients rotationalPIDs = new PIDCoefficients(0.0502, 0.00065,0.00001);
    public static PIDCoefficients rotationalPIDs2 = new PIDCoefficients(0.0102, 0.00065,0.00001);
    public static double INTAKE_POWER = 1;
    public static DcMotorSimple.Direction intakeDirection = DcMotorSimple.Direction.REVERSE;

    public static double SHOOTER_TARGET_RPM = 5000;
    public static double SHOOTER_IDLE_RPM = 120;
    public static int SHOOTER_PPR = 28;
    public static double SHOOTER_ANGULAR_VELOCITY = 0;
    public static double SHOOTER_UPDATE_TIME_SECONDS = 0.15;
    public static double SHOOTER_ANGLE_DEG = 50;

    public static double LIMELIGHT_HEIGHT_FROM_GROUND_INCHES = 12.5;


    public static double FLICKER_SERVO_UP = 0.4;
    public static double FLICKER_SERVO_DOWN = 0.1;
    public static double ALL_FLICKER_DOWN_DELAY_SECONDS = 1;

    public static double maxLaunchZoneDistance = 15; //inches

    public static double maxLaunchZoneArcShooterDistance = 24; //inches


    public static RGB greenRGB = new RGB(0, 255, 0);
    public static RGB purpleRGB = new RGB(128, 0, 128);
    public static double STORAGE_BALL_1 = 0.1;
    public static double STORAGE_BALL_2 = 0.45;
    public static double STORAGE_BALL_3 = 0.85;


    //TODO: Get these values
    public static int SPINDEXER_MOTOR_BALL_1_INTAKE = 0;
    public static int SPINDEXER_MOTOR_BALL_1_SHOOT = 0;
    public static int SPINDEXER_MOTOR_BALL_2_INTAKE = 0;
    public static int SPINDEXER_MOTOR_BALL_2_SHOOT = 0;
    public static int SPINDEXER_MOTOR_BALL_3_INTAKE = 0;
    public static int SPINDEXER_MOTOR_BALL_3_SHOOT = 0;
    //TUNERS---------------------------------------------------------------------------------------------------------------------------
    //TODO: TURN OFF BEFORE COMPS
    public static boolean panelsDrawingEnabled = true;
    public static boolean panelsEnabled = true;

    public static CanvasRotation panelsFieldRotation = CanvasRotation.DEG_270;

    public static double defaultShooterRPM = SHOOTER_TARGET_RPM;
    public static double velocityUpStep = 0.1;
    public static double velocityDownStep = 0.1;

    public static double storageServoStep = 0.05;
}
