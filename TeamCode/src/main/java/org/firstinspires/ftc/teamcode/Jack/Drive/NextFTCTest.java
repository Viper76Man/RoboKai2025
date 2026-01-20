package org.firstinspires.ftc.teamcode.Jack.Drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Jack.Motors.SpindexerMotorV1;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.IntakeSubsystemV1;

@TeleOp
public class NextFTCTest extends OpMode {
    public IntakeSubsystemV1 intakeSub = new IntakeSubsystemV1();

    @Override
    public void init(){
        intakeSub.spindexerRun(RobotConstantsV1.SPINDEXER_MOTOR_BALL_1_INTAKE, SpindexerMotorV1.EncoderMeasurementMethod.MOTOR);
    }

    @Override
    public void loop() {
        telemetry.addLine("Target: " + intakeSub.spindexer.targetPositionEncoder);
        telemetry.addLine("Pos: " + intakeSub.spindexer.getCurrentPosition());
    }
}
