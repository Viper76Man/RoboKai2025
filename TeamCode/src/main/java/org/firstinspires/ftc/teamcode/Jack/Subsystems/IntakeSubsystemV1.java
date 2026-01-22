package org.firstinspires.ftc.teamcode.Jack.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.IntakeV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.SpindexerMotorV1;
import org.firstinspires.ftc.teamcode.Jack.Other.SlotColorSensorV1;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class IntakeSubsystemV1 implements Subsystem {
    public SpindexerMotorV1 spindexer = new SpindexerMotorV1();
    public IntakeV1 intake  = new IntakeV1();
    public SlotColorSensorV1 sensor = new SlotColorSensorV1();
    public HardwareMap hardwareMap;
    public boolean ballDetected = false;
    public boolean searching = false;

    public void init(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        intake.init(hardwareMap);
        spindexer.init(hardwareMap, RobotConstantsV1.spindexerPIDs);
        sensor.init(hardwareMap, RobotConstantsV1.colorSensor1);
        spindexer.setTargetPos(RobotConstantsV1.SPINDEXER_MOTOR_BALL_1_INTAKE, SpindexerMotorV1.EncoderMeasurementMethod.MOTOR);
    }

    public CustomCommand spindexerRun(double pos, SpindexerMotorV1.EncoderMeasurementMethod method){
        if(method == null){
            method = SpindexerMotorV1.EncoderMeasurementMethod.MOTOR;
        }
        return new CustomCommand(new SpindexerRunToPos(pos, method));
    }

    public CustomCommand setIntakePower(double power, DcMotorSimple.Direction direction){
        return new CustomCommand(new setIntakePower(power, direction));
    }

    public CustomCommand ballUpdate(){
        return new CustomCommand(new BallUpdate());
    }

    //SPINDEXER-------------------------------------------------------------------------------------
    public class SpindexerRunToPos extends Command {
        public ElapsedTime spinTimer = new ElapsedTime();

        public SpindexerRunToPos(double pos, SpindexerMotorV1.EncoderMeasurementMethod method){
            setName("Spindexer Run to " + pos + "with mode " + method.name());
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
            return (spindexer.isSpindexerReady() && spinTimer.seconds() > 0.2) || ActiveOpMode.isStopRequested();
        }

        @Override
        public void stop(boolean interrupted){
            spindexer.spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            spindexer.setMotorPower(0);
        }
    }
    //----------------------------------------------------------------------------------------------
    public class BallUpdate extends Command {

        @Override
        public void start(){
            ballDetected = false;
            searching = true;
            sensor.clear();
        }

        @Override
        public void update() {
            sensor.update(spindexer.state, spindexer.isSpindexerReady());
        }

        @Override
        public boolean isDone() {
            return (sensor.isGreen() || sensor.isPurple()) || ActiveOpMode.isStopRequested();
        }

        @Override
        public void stop(boolean interrupted){
            ballDetected = true;
            searching = false;
        }
    }

    public class setIntakePower extends Command {
        public double power;
        public DcMotorSimple.Direction dir;
        public setIntakePower(double power, DcMotorSimple.Direction direction){
            this.power = power;
            this.dir = direction;
        }

        @Override
        public void update(){
            intake.setPower(power);
            intake.setDirection(dir);
        }

        @Override
        public boolean isDone() {
            return intake.motor.getPower() == power;
        }
    }


}
