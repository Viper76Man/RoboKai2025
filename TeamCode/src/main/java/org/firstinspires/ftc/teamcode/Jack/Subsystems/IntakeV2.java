package org.firstinspires.ftc.teamcode.Jack.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotV4;
import org.firstinspires.ftc.teamcode.Jack.Other.BallManager;
import org.firstinspires.ftc.teamcode.Jack.Other.SlotColorSensorV1;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.subsystems.SubsystemGroup;
import dev.nextftc.ftc.ActiveOpMode;

public class IntakeV2 extends SubsystemGroup {
    public IntakeMotorV2 intakeMotor = new IntakeMotorV2();
    public SpindexerV2 spindexer = new SpindexerV2();
    public BallManager manager;

    public void init(HardwareMap hardwareMap, BallManager manager){
        this.manager = manager;
        intakeMotor.init(hardwareMap);
        spindexer.init(manager);
    }



}
