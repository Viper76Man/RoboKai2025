package org.firstinspires.ftc.teamcode.Coach;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="ThroughBoreEncoderTest", group="Test")

public class ThroughBoreEncoderTest extends LinearOpMode{

    private DcMotorEx motor; // Use DcMotorEx for more features
    private static final double CPR = 8192; // REV Through Bore Encoder is 8192 CPR (Counts Per Revolution)

    @Override
    public void runOpMode() {
        // Initialize motor (ensure "motor_name" matches your hardware map)
        motor = hardwareMap.get(DcMotorEx.class, "spindexer");
        motor.setDirection(DcMotorSimple.Direction.REVERSE); // Adjust as needed

        // --- Initialization ---
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset encoder at start
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Run with encoder feedback

        // For angle control:
        motor.setDirection(DcMotorSimple.Direction.FORWARD); // Or REVERSE
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // For moving to a target position
        motor.setTargetPositionTolerance(50); // Tolerance in encoder ticks

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- Reading Encoder Values ---
            int currentPosition = motor.getCurrentPosition(); // Ticks
            double revolutions = (double) currentPosition / CPR;
            double degrees = revolutions * 360;
            double tickTarget = 90 / 360.0 * CPR;

            telemetry.addData("Position Ticks", currentPosition);
            telemetry.addData("Revolutions", revolutions);
            telemetry.addData("Degrees", degrees);
            telemetry.addData("Target Ticks", tickTarget);

            // Go to 90 degrees
            motor.setTargetPosition((int)tickTarget); // Target position in ticks
            motor.setPower(0.8); // Set power to start moving
            telemetry.update();
            sleep(50); // Small delay
        }
    }
}
