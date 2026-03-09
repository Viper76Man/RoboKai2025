package org.firstinspires.ftc.teamcode.Coach;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Coach.servos.FlickerServo;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name = "Servo Test")
public class ServoTest extends NextFTCOpMode {
    public ServoTest() {
        addComponents(
                new SubsystemComponent(FlickerServo.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }


/**
 * Test Region
 */
    @Override
    public void onStartButtonPressed(){
        Gamepads.gamepad1().rightBumper()
        .whenBecomesTrue(
                new SequentialGroup(
                        FlickerServo.INSTANCE.flick,
                        new Delay(.25),
                        FlickerServo.INSTANCE.rest,
                        new Delay(.25),
                        FlickerServo.INSTANCE.flick,
                        new Delay(.25),
                        FlickerServo.INSTANCE.rest,
                        new Delay(.25),
                        FlickerServo.INSTANCE.flick,
                        new Delay(.25),
                        FlickerServo.INSTANCE.rest
                )
        );
    }

}
