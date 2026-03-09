package org.firstinspires.ftc.teamcode.Coach.servos;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class FlickerServo implements Subsystem {
    public static final FlickerServo INSTANCE = new FlickerServo();
    private FlickerServo() { }

    private ServoEx servo = new ServoEx("flicker");

    public Command rest = new SetPosition(servo, .2).requires(this);
    public Command flick = new SetPosition(servo, .4).requires(this);
}
