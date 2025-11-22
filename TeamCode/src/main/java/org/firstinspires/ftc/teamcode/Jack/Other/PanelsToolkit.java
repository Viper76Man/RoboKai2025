package org.firstinspires.ftc.teamcode.Jack.Other;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.panels.Panels;
import com.bylazar.panels.json.PanelsWidget;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Jack.Drive.Robot;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.Jack.Odometry.Tuning;
import org.firstinspires.ftc.vision.VisionProcessor;

public class PanelsToolkit {
    public Telemetry telemetry;
    public int mode = -1;
    public boolean enabled = RobotConstantsV1.panelsDrawingEnabled;
    public MultipleTelemetry multipleTelemetry;
    public TelemetryManager panels;
    public PanelsToolkit(Telemetry telemetry){
        this.telemetry = telemetry;
        this.mode = 1;
    }
    public PanelsToolkit(MultipleTelemetry telemetry){
        this.multipleTelemetry = telemetry;
        this.mode = 2;
    }
    public PanelsToolkit(Telemetry telemetry, TelemetryManager panels){
        this.multipleTelemetry = new MultipleTelemetry(telemetry, panels);
        this.mode = 3;
    }
    public PanelsToolkit(TelemetryManager panels){
        this.panels = panels;
    }
    //Drawing------------------------------------------------------------------------------
    public static final double ROBOT_RADIUS = 9; // woah
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();

    private static final Style robotLook = new Style(
            "", "#3F51B5", 0.75
    );
    private static final Style historyLook = new Style(
            "", "#4CAF50", 0.75
    );
    //---------------------------------------------------------------------------------------
    public void drawRobot(Pose pose, Style style){
        if(!enabled){
            return;
        }
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            return;
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(ROBOT_RADIUS);

        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
        double x1 = pose.getX() + v.getXComponent() / 2, y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent(), y2 = pose.getY() + v.getYComponent();

        panelsField.setStyle(style);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }
    public void graph(String caption, Object value){
        switch (mode){
            case 1:
                telemetry.addData(caption, value);
                break;
            case 2:
            case 3:
                multipleTelemetry.addData(caption, value);
                break;
            case 4:
                panels.addData(caption, value);
                break;
        }
    }
    public void toggleDrawing(){
        setDrawingEnabled(!enabled);
    }
    public void setDrawingEnabled(boolean enabled){
        this.enabled = enabled;
    }
}
