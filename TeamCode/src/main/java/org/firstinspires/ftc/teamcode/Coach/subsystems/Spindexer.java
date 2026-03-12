package org.firstinspires.ftc.teamcode.Coach.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class Spindexer implements Subsystem {
    public static final Spindexer INSTANCE = new Spindexer();
    private final double firstPos = 0;  //0 degrees
    private final double secondPos = 250.6; //-120 degrees
    private final double thirdPos = 501.2;  //-240 degrees
    private final double fourthPos = 751.8;  //360 degrees
    private final double fifthPos = 1002.4;  //480 degrees
    private final double sixthPos = 1253; //600 degrees

    private Spindexer() {
    }

    private final MotorEx motor = new MotorEx("spindexer").reversed();



    private final ControlSystem controlSystem = ControlSystem.builder()
            .posPid(0.005, 0, 0)
            .build();

    public Command toFirstPos = new RunToPosition(controlSystem, firstPos).requires(this);
    public Command toSecondPOS = new RunToPosition(controlSystem, secondPos).requires(this);
    public Command toThirdPos = new RunToPosition(controlSystem, thirdPos).requires(this);
    public Command toFourthPos = new RunToPosition(controlSystem, fourthPos).requires(this);
    public Command toFifthPos = new RunToPosition(controlSystem, fifthPos).requires(this);
    public Command toSixthPos = new RunToPosition(controlSystem, sixthPos).requires(this);


    @Override
    public void initialize() {
        //reset encoder
        motor.zero();
    }

    @Override
    public void periodic() {
        motor.setPower(controlSystem.calculate(motor.getState()));
    }

}
