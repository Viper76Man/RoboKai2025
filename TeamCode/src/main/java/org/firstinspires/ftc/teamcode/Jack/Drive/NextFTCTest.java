package org.firstinspires.ftc.teamcode.Jack.Drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Jack.Motors.SpindexerMotorV1;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.IntakeSubsystemV1;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp
public class NextFTCTest extends NextFTCOpMode {
    public IntakeSubsystemV1 intakeSub = new IntakeSubsystemV1();
    public IntakeSubsystemV1.SpindexerRunToPos command;
    @Override
    public void onInit(){
        intakeSub.init(hardwareMap);
        command = intakeSub.spindexerRun(RobotConstantsV1.SPINDEXER_MOTOR_BALL_1_INTAKE, SpindexerMotorV1.EncoderMeasurementMethod.MOTOR);
    }

    @Override
    public void onUpdate() {
        telemetry.addLine("Target: " + intakeSub.spindexer.targetPositionEncoder);
        telemetry.addLine("Pos: " + intakeSub.spindexer.getCurrentPosition());
        telemetry.addLine("Is done? " + command.isDone());
        telemetry.update();
    }
}
