package org.firstinspires.ftc.teamcode.Coach;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name="Encoder Test")

public class EncoderTest extends LinearOpMode {

    //Hardware
    DcMotorEx motor;
    //Variables
    final double TICKS_PER_REV = 8192; // Typical for REV Through Bore (check encoder specs)

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motor = hardwareMap.get(DcMotorEx.class, "spindexer");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset encoder at start
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Use encoder for control
        motor.setDirection(DcMotor.Direction.REVERSE);
        // Wait for the game to start (driver presses START)
        waitForStart();
        encoderSpin();


    }



    public void encoderSpin(){
        double targetAngle = 90.0;
        double ticksForAngle = (targetAngle / 360.0) * TICKS_PER_REV;
        motor.setTargetPosition((int) ticksForAngle);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.8); // Move
        // Check if it's close to the target
        while (opModeIsActive() && motor.isBusy()) {
            // Optional: Telemetry for debugging
            telemetry.addData("Current Position", motor.getCurrentPosition());
            telemetry.addData("Ticks For Angle", ticksForAngle);
            telemetry.update();
        }
        motor.setPower(0.0); // Stop after reaching target
        sleep(8000);  // pause to display final telemetry message.
    }

}
