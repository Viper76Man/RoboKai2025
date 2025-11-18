package org.firstinspires.ftc.teamcode.Jack.Other;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MultipleTelemetry {
    public Telemetry telemetry;
    public TelemetryManager panels;
    public MultipleTelemetry(Telemetry telemetry, TelemetryManager panels){
        this.panels = panels;
        this.telemetry = telemetry;
    }
    public void addData(String key, Object value){
        panels.addData(key, value);
    }

    public void addLine(String line){
        panels.addLine(line);
        telemetry.addLine(line);
    }

    public void update(){
        panels.update(telemetry);
        telemetry.update();
    }
}
