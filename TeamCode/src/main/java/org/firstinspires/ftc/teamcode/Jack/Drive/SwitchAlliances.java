package org.firstinspires.ftc.teamcode.Jack.Drive;


import com.bylazar.configurables.PanelsConfigurables;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Jack.Other.LoggerV1;
import org.firstinspires.ftc.teamcode.Jack.Other.PanelsToolkit;

@TeleOp(name = "Alliance Switcher", group = "Experimental")
public class SwitchAlliances extends OpMode {
    public LoggerV1 logger = new LoggerV1();
    public Robot.Alliance alliance;
    public PanelsToolkit toolkit = new PanelsToolkit(telemetry);
    @Override
    public void init() {
        logger.init(telemetry);
        PanelsConfigurables.INSTANCE.refreshClass(RobotConstantsV1.class);
        toolkit.setPanelsEnabled(RobotConstantsV1.panelsEnabled);
        if(logger.readSideFromFile() != null) {
            switch (logger.readSideFromFile()) {
                case RED:
                    logger.saveSideToFile(Robot.Alliance.BLUE);
                    alliance = Robot.Alliance.BLUE;
                    break;
                case BLUE:
                    logger.saveSideToFile(Robot.Alliance.RED);
                    alliance = Robot.Alliance.RED;
                    break;
            }
        }
        else {
            logger.saveSideToFile(Robot.Alliance.RED);
            alliance = Robot.Alliance.RED;
        }
    }

    @Override
    public void loop() {
        telemetry.addData("Side switched. Current side: ", alliance.name());
    }
}
