package org.firstinspires.ftc.teamcode.Jack.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.SpindexerMotorV1;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class IntakeSubsystemV1 implements Subsystem {
    public SpindexerMotorV1 spindexer = new SpindexerMotorV1();

    @Override
    public void initialize() {
        init(ActiveOpMode.hardwareMap());
    }

    @Override
    public void periodic() {
        spindexer.run();
    }

    private void init(HardwareMap hardwareMap){
        spindexer.init(hardwareMap, RobotConstantsV1.spindexerPIDs);
    }

    public void spindexerRun(double pos, SpindexerMotorV1.EncoderMeasurementMethod method){
        if(method == null){
            method = SpindexerMotorV1.EncoderMeasurementMethod.MOTOR;
        }
        new SpindexerRunToPos(pos, method).schedule();
    }

    public class SpindexerRunToPos extends Command {
        public ElapsedTime spinTimer = new ElapsedTime();

        public SpindexerRunToPos(double pos, SpindexerMotorV1.EncoderMeasurementMethod method){
            spindexer.setTargetPos(pos, method);
        }

        @Override
        public void start() {
            spinTimer.reset();
        }

        @Override
        public void update() {
            spindexer.run();
        }


        @Override
        public boolean isDone() {
            return spindexer.isSpindexerReady() && spinTimer.seconds() > 0.2;
        }
    }

}
