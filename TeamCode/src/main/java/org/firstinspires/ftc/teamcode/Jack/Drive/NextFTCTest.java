package org.firstinspires.ftc.teamcode.Jack.Drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Jack.Motors.SpindexerMotorV1;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.CustomCommand;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.CustomParallelGroup;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.IntakeSubsystemV1;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp
public class NextFTCTest extends NextFTCOpMode {
    public IntakeSubsystemV1 intakeSub = new IntakeSubsystemV1();
    public CustomCommand command, command2;
    public ParallelGroup group;
    public enum State{
        START,
        DONE
    }
    public State state = State.START;
    @Override
    public void onInit(){
        intakeSub.init(hardwareMap);
        command = intakeSub.spindexerRun(RobotConstantsV1.SPINDEXER_MOTOR_BALL_1_INTAKE, SpindexerMotorV1.EncoderMeasurementMethod.MOTOR);
        command2 = intakeSub.setIntakePower(RobotConstantsV1.INTAKE_POWER, RobotConstantsV1.intakeDirection);
        group = new ParallelGroup(command.actualCommand, command2.actualCommand);
    }

    @Override
    public void onUpdate() {
        switch (state){
            case START:
                group.schedule();
                state = State.DONE;
                break;
        }
        telemetry.addLine("Target: " + intakeSub.spindexer.targetPositionEncoder);
        telemetry.addLine("Pos: " + intakeSub.spindexer.getCurrentPosition());
        telemetry.addLine("Is done? " + command.isDone());
        telemetry.update();
    }
}
