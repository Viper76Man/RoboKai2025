package org.firstinspires.ftc.teamcode.Jack.Drive;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Jack.Camera.Limelight3A.LimelightV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.PIDController;
import org.firstinspires.ftc.teamcode.Jack.Odometry.DecodeFieldLocalizer;
import org.firstinspires.ftc.teamcode.Jack.Other.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.R;

public class MecanumDriveOnly {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    public HardwareMap hardwareMap;
    public GamepadV1 gamepad1 = new GamepadV1();
    public DecodeFieldLocalizer localizer = new DecodeFieldLocalizer();
    public LimelightV1 limelight = new LimelightV1();
    public boolean firstIteration = true;
    public PIDController controller;
    public double rotationalError;

    public void init(HardwareMap hardwareMap, Gamepad gamepad) {
        this.hardwareMap = hardwareMap;
        this.gamepad1.init(gamepad, 0.3);
        controller = new PIDController(RobotConstantsV1.rotationalPIDs.p, RobotConstantsV1.rotationalPIDs.i, RobotConstantsV1.rotationalPIDs.d);
        frontLeftMotor = hardwareMap.get(DcMotor.class, RobotConstantsV1.frontLeft);
        frontRightMotor = hardwareMap.get(DcMotor.class, RobotConstantsV1.frontRight);
        backLeftMotor = hardwareMap.get(DcMotor.class, RobotConstantsV1.backLeft);
        backRightMotor = hardwareMap.get(DcMotor.class, RobotConstantsV1.backRight);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setDirection(RobotConstantsV1.frontLeftDirection);
        backLeftMotor.setDirection(RobotConstantsV1.backLeftDirection);
        frontRightMotor.setDirection(RobotConstantsV1.frontRightDirection);
        backRightMotor.setDirection(RobotConstantsV1.backRightDirection);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void init(HardwareMap hardwareMap, GamepadV1 gamepad) {
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad;
        frontLeftMotor = hardwareMap.get(DcMotor.class, RobotConstantsV1.frontLeft);
        frontRightMotor = hardwareMap.get(DcMotor.class, RobotConstantsV1.frontRight);
        backLeftMotor = hardwareMap.get(DcMotor.class, RobotConstantsV1.backLeft);
        backRightMotor = hardwareMap.get(DcMotor.class, RobotConstantsV1.backRight);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setDirection(RobotConstantsV1.frontLeftDirection);
        backLeftMotor.setDirection(RobotConstantsV1.backLeftDirection);
        frontRightMotor.setDirection(RobotConstantsV1.frontRightDirection);
        backRightMotor.setDirection(RobotConstantsV1.backRightDirection);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setToBrake(){
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }



    public void drive() {
        double y = -gamepad1.gamepad.left_stick_y; // Remember, this is reversed!
        double x = -gamepad1.gamepad.left_stick_x; // Counteract imperfect strafing, if the back motors are facing downwards this should be negative
        double rx = -gamepad1.gamepad.right_stick_x; //This is reversed for our turning
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (-y + x + rx) / denominator;
        double backLeftPower = (-y - x + rx) / denominator;
        double frontRightPower = (y + x + rx) / denominator;
        double backRightPower = (y - x + rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    public void drive(boolean slowmode) {
        double y = -gamepad1.gamepad.left_stick_y; // Remember, this is reversed!
        double x = -gamepad1.gamepad.left_stick_x; // Counteract imperfect strafing, if the back motors are facing downwards this should be negative
        double rx = -gamepad1.gamepad.right_stick_x; //This is reversed for our turning
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (-y + x + rx) / denominator;
        double backLeftPower = (-y - x + rx) / denominator;
        double frontRightPower = (y + x + rx) / denominator;
        double backRightPower = (y - x + rx) / denominator;
        if(!slowmode) {
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
        else {
            frontLeftMotor.setPower(frontLeftPower * 0.8);
            backLeftMotor.setPower(backLeftPower * 0.8);
            frontRightMotor.setPower(frontRightPower * 0.8);
            backRightMotor.setPower(backRightPower * 0.8);
        }
    }

    public void driveSlowmode(double slowPower) {
        slowPower = Math.abs(slowPower);
        double y = -gamepad1.gamepad.left_stick_y; // Remember, this is reversed!
        double x = -gamepad1.gamepad.left_stick_x; // Counteract imperfect strafing, if the back motors are facing downwards this should be negative
        double rx = -gamepad1.gamepad.right_stick_x; //This is reversed for our turning
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (-y + x + rx) / denominator;
        double backLeftPower = (-y - x + rx) / denominator;
        double frontRightPower = (y + x + rx) / denominator;
        double backRightPower = (y - x + rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower * slowPower);
        backLeftMotor.setPower(backLeftPower * slowPower);
        frontRightMotor.setPower(frontRightPower * slowPower);
        backRightMotor.setPower(backRightPower * slowPower);
    }


    public void drive(double y, double x, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (-y + x + rx) / denominator;
        double backLeftPower = (-y - x + rx) / denominator;
        double frontRightPower = (y + x + rx) / denominator;
        double backRightPower = (y - x + rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    public void driveWithRotationLock(Robot.Alliance team, Pose pose, Telemetry telemetry, boolean useCamera){
        gamepad1.update();
        if(firstIteration && useCamera){
            controller.setConstants(RobotConstantsV1.rotationalPIDs);
            limelight.init(hardwareMap);
            firstIteration = false;
        }
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = -gamepad1.left_stick_x; // Counteract imperfect strafing, if the back motors are facing downwards this should be negative
        double rx = -gamepad1.right_stick_x;
        if(!useCamera) {
            switch (team) {
                case BLUE:
                    rx = -controller.kP * localizer.getHeadingErrorBlue(pose);
                    break;
                case RED:
                    rx = -controller.kP * localizer.getHeadingErrorRed(pose);
                    break;
                case TEST:
                    rx = -controller.kP * localizer.getHeadingErrorFromGoalDegrees(pose);
                    break;
                }
            telemetry.addLine("rx: " + rx);
            drive(y,x, rx);
        }
        else {
            LLResultTypes.FiducialResult result = limelight.getLatestAprilTagResult();
            if (result != null) {
                telemetry.addLine("All clear.");
                rotationalError = result.getTargetXDegrees();
                rx = -RobotConstantsV1.rotationalPIDs2.p * rotationalError;
                rx = Math.min(Math.max(rx, -1), 1);
                telemetry.addLine("rx: " + rx);
                drive(y, x, rx);
            }
            else{
                telemetry.addData("Distance: ", localizer.getDistanceFromLaunchZone(pose));
                telemetry.addData("Apriltag Result is null?", limelight.getLatestAprilTagResult() == null);
                telemetry.addLine("rx: " + rx);
                drive(y, x, rx);
            }
        }
    }


    public void log(Telemetry telemetry){
        telemetry.addLine("Front Left Power: " + frontLeftMotor.getPower());
        telemetry.addLine("Front Right Power: " + frontRightMotor.getPower());
        telemetry.addLine("Back Left Power: " + backLeftMotor.getPower());
        telemetry.addLine("Back Right Power: " + backRightMotor.getPower());
        telemetry.addLine("Front Left Position: " + frontLeftMotor.getCurrentPosition());
        telemetry.addLine("Front Right Position: " + frontRightMotor.getCurrentPosition());
        telemetry.addLine("Back Left Position: " + backLeftMotor.getCurrentPosition());
        telemetry.addLine("Back Right Position: " + backRightMotor.getCurrentPosition());
        telemetry.addLine("FL Mode: " + frontLeftMotor.getZeroPowerBehavior());
        telemetry.addLine("BL Mode: " + backLeftMotor.getZeroPowerBehavior());
        telemetry.addLine("FR Mode: " + frontRightMotor.getZeroPowerBehavior());
        telemetry.addLine("BR Mode: " + backRightMotor.getZeroPowerBehavior());
    }
    public void log(MultipleTelemetry telemetry){
        telemetry.addLine("Front Left Power: " + frontLeftMotor.getPower());
        telemetry.addLine("Front Right Power: " + frontRightMotor.getPower());
        telemetry.addLine("Back Left Power: " + backLeftMotor.getPower());
        telemetry.addLine("Back Right Power: " + backRightMotor.getPower());
        telemetry.addLine("Front Left Position: " + frontLeftMotor.getCurrentPosition());
        telemetry.addLine("Front Right Position: " + frontRightMotor.getCurrentPosition());
        telemetry.addLine("Back Left Position: " + backLeftMotor.getCurrentPosition());
        telemetry.addLine("Back Right Position: " + backRightMotor.getCurrentPosition());
        telemetry.addLine("FL Mode: " + frontLeftMotor.getZeroPowerBehavior());
        telemetry.addLine("BL Mode: " + backLeftMotor.getZeroPowerBehavior());
        telemetry.addLine("FR Mode: " + frontRightMotor.getZeroPowerBehavior());
        telemetry.addLine("BR Mode: " + backRightMotor.getZeroPowerBehavior());
        telemetry.addLine("Error: " + rotationalError);
    }


}
