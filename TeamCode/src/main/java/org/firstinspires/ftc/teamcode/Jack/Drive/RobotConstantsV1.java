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

    public static DcMotorSimple.Direction frontLeftDirectionAuto = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction backLeftDirectionAuto = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction frontRightDirectionAuto = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction backRightDirectionAuto = DcMotorSimple.Direction.FORWARD;
    public static boolean useBrakeInTeleOp = true;
    //Config names
    public static String pinpointName = "pinpoint";
    //HARDWARE--------------------------------------------------------------------------------------
    public static String arcShooterName = "arcMotor"; //Expansion Hub Port 1
    public static String intakeMotorName = "intake"; //Expansion Hub Port 0
    public static String flickerServoName = "flicker"; //Control Hub Servos Port 1
    public static String storageServoName = "storageServo"; //Control Hub Servos Port 0
    public static String colorSensor1 = "colorSensor1"; //Expansion Hub I2C port 0
    public static String spindexerMotorName = "spindexer"; //Expansion Hub Port 2
    public static String turretServoName = "turretServo";

    //AUTONOMOUS-------------------------------------------------------------------------------------
    //Auto, needs measuring, in inches
    public static double forwardPodY = 2;
    public static double strafePodX = -6;
    public static DistanceUnit podsMeasurementUnit = DistanceUnit.INCH;

    public static int degreeToleranceCamera = 2;
    public static int degreeToleranceCameraAuto = 5;
    public static double shotAngleBlueDegrees = 70;
    public static double shotAngleRedDegrees = 110;

    public static GoBildaPinpointDriver.EncoderDirection forwardPodDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection lateralPodDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;

    public static double xVelocity = 65.42;
    public static double yVelocity = 50.83;

    //Tuning values
    public static double robotMassKG = 16.09;
    public static double forwardZeroPowerAcceleration = -31.31;
    public static double lateralZeroPowerAcceleration = -59.31;
    public static double centripetalScaling = 0.005;

    //PIDs
    public static PIDFCoefficients translationalPIDCoefficients = new PIDFCoefficients(0.06, 0, 0.001, 0);
    public static PIDFCoefficients headingPIDFCoefficients = new PIDFCoefficients(2, 0, 0.01, 0);
    public static FilteredPIDFCoefficients drivePIDCoefficients = new FilteredPIDFCoefficients(0.015, 0, 0, 0.6, 0);

    //TELEOP-------------------------------------------------------------------------------------------------------------------------
    public static PIDFCoefficients arcPIDs = new PIDFCoefficients(0.000101, 0, 0, 0.0001965);
    public static PIDFCoefficients arcPIDsAuto = new PIDFCoefficients(0.000101, 0, 0, 0.0001967);

    public static PIDCoefficients rotationalPIDs = new PIDCoefficients(0.0502, 0.00065,0.00001);
    public static PIDCoefficients rotationalPIDs2 = new PIDCoefficients(0.0102, 0.00065,0.00001);
    public static PIDCoefficients spindexerPIDs = new PIDCoefficients(0.005, 0, 0.000001);
    public static PIDCoefficients turretPIDs = new PIDCoefficients(-0.013, -0.0002, -0.00025);

    public static double brakingStrength = 1.25;
    public static double timeoutConstantMS = 600;
    public static double brakingStart = 1;
    public static double tValueConstraint = 1;


    public static double INTAKE_POWER = 0.9;
    public static double turretServoPower = 0.01;
    public static DcMotorSimple.Direction intakeDirection = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction leftShooterDirection = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction rightShooterDirection = DcMotorSimple.Direction.FORWARD;
    public static boolean useBothArcMotors = false;

    public static double TURRET_INPUT_VALUE = 255;
    public static double TURRET_MAX_ENCODER_VALUE = 320;

    public static double TURRET_SERVO_MAX_ANGLE_DEG = TURRET_INPUT_VALUE * ((double) 355 / 255);

    public static double SHOOTER_TARGET_RPM = 3800;

    public static double SHOOTER_TARGET_RPM_AUTO = 3850;
    public static double SHOOTER_FRONT_RPM = 3200;
    public static double SHOOTER_IDLE_RPM = 120;
    public static int SHOOTER_PPR = 28;
    public static double SHOOTER_ANGULAR_VELOCITY = 0;
    public static double SHOOTER_UPDATE_TIME_SECONDS = 0.15;
    public static double SHOOTER_ANGLE_DEG = 50;

    public static double TURRET_OFFSET_ANGLE_BLUE = -3;
    public static double TURRET_OFFSET_ANGLE_RED = 5;

    public static double LIMELIGHT_HEIGHT_FROM_GROUND_INCHES = 12.5;

    public static int SPINDEXER_TOLERANCE = 1000;

    public static double MIN_G_VALUE_COLOR_SENSOR = 165;
    public static double MAX_DISTANCE_COLOR_SENSOR = 32;
    public static double MIN_DISTANCE_COLOR_SENSOR = 13;

    public static double FLICKER_SERVO_UP = 0.7;
    public static double FLICKER_SERVO_DOWN = 1;
    public static double FLICKER_UP_TIME = 0.35;

    public static double maxLaunchZoneDistance = 15; //inches

    public static double maxLaunchZoneArcShooterDistance = 24; //inches


    public static RGB greenRGB = new RGB(0, 255, 0);
    public static RGB purpleRGB = new RGB(128, 0, 128);
    public static double STORAGE_BALL_1 = 0.1;
    public static double STORAGE_BALL_2 = 0.45;
    public static double STORAGE_BALL_3 = 0.85;


    //TODO: Get these values
    public static int SPINDEXER_ENCODER_BALL_1_INTAKE = 2424;
    public static int SPINDEXER_ENCODER_BALL_1_SHOOT = 518;
    public static int SPINDEXER_ENCODER_BALL_2_INTAKE = 3650;
    public static int SPINDEXER_ENCODER_BALL_2_SHOOT = 3000;
    public static int SPINDEXER_ENCODER_BALL_3_INTAKE = 1102;
    public static int SPINDEXER_ENCODER_BALL_3_SHOOT = 3255;

    public static int SPINDEXER_MOTOR_BALL_1_INTAKE = 0;
    public static int SPINDEXER_MOTOR_BALL_1_SHOOT = 380;
    public static int SPINDEXER_MOTOR_BALL_2_INTAKE = 240;
    public static int SPINDEXER_MOTOR_BALL_2_SHOOT = 632;
    public static int SPINDEXER_MOTOR_BALL_3_INTAKE = 485;
    public static int SPINDEXER_MOTOR_BALL_3_SHOOT = 122;
    //TUNERS---------------------------------------------------------------------------------------------------------------------------
    //TODO: TURN OFF BEFORE COMPS
    public static boolean panelsDrawingEnabled = true;
    public static boolean panelsEnabled = true;

    public static CanvasRotation panelsFieldRotation = CanvasRotation.DEG_90;

    public static double defaultShooterRPM = SHOOTER_TARGET_RPM;
    public static double velocityUpStep = 0.1;
    public static double velocityDownStep = 0.1;

    public static double storageServoStep = 0.05;
}
