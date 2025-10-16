package org.firstinspires.ftc.teamcode.Jack.Drive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumDriveOnly {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    public RobotConstantsV1 constants = new RobotConstantsV1();
    public GamepadV1 gamepad1 = new GamepadV1();

    public void init(HardwareMap hardwareMap, Gamepad gamepad) {
        gamepad1.init(gamepad, 0.3);
        frontLeftMotor = hardwareMap.get(DcMotor.class, constants.frontLeft);
        frontRightMotor = hardwareMap.get(DcMotor.class, constants.frontRight);
        backLeftMotor = hardwareMap.get(DcMotor.class, constants.backLeft);
        backRightMotor = hardwareMap.get(DcMotor.class, constants.backRight);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void init(HardwareMap hardwareMap, GamepadV1 gamepad) {
        this.gamepad1 = gamepad;
        frontLeftMotor = hardwareMap.get(DcMotor.class, constants.frontLeft);
        frontRightMotor = hardwareMap.get(DcMotor.class, constants.frontRight);
        backLeftMotor = hardwareMap.get(DcMotor.class, constants.backLeft);
        backRightMotor = hardwareMap.get(DcMotor.class, constants.backRight);

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

    public void log(Telemetry telemetry){
        telemetry.addLine("Front Left Power: " + frontLeftMotor.getPower());
        telemetry.addLine("Front Right Power: " + frontRightMotor.getPower());
        telemetry.addLine("Back Left Power: " + backLeftMotor.getPower());
        telemetry.addLine("Back Right Power: " + backRightMotor.getPower());
        telemetry.addLine("Front Left Position: " + frontLeftMotor.getCurrentPosition());
        telemetry.addLine("Front Right Position: " + frontRightMotor.getCurrentPosition());
        telemetry.addLine("Back Left Position: " + backLeftMotor.getCurrentPosition());
        telemetry.addLine("Back Right Position: " + backRightMotor.getCurrentPosition());
    }
}
