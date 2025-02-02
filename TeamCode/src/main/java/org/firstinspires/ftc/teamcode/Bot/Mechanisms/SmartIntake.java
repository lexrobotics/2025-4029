package org.firstinspires.ftc.teamcode.Bot.Mechanisms;

import static org.firstinspires.ftc.teamcode.Bot.Setup.telemetry;

import org.firstinspires.ftc.teamcode.Bot.States.ActionSequences;
import org.firstinspires.ftc.teamcode.Bot.Mechanisms.Sensors;

public class SmartIntake {
    private ActionSequences actions;
    private boolean isBlue;
    private Sensors sensors;
    public enum Color{
        YELLOW,
        RED,
        BLUE,
        NOTHING
    }
    public SmartIntake(ActionSequences actionSequences, boolean isBlue, Sensors sensors){
        actions = actionSequences;
        this.isBlue = isBlue;
        this.sensors = sensors;
    }

    /***
     * @param bePicky
     */
    public Color intake(boolean bePicky, boolean move){
        if(!sensors.getTouchStatus(0)) {
            telemetry.addLine("Booped (SmartIntake)");
            return assignColor(sensors.getColor(0));
        } else {
            telemetry.addLine("Not booped (SmartIntake)");
            actions.intake(1);
            return Color.NOTHING;
        }
    }
    public Color assignColor(double[] colors){
        if(1.5*colors[2] > colors[1] && colors[2] > colors[3]) {
            return Color.YELLOW;
        } else if(colors[1] > colors[2] && colors[1] > colors[3]){
            return Color.RED;
        } else{
            return Color.BLUE;
        }
//        else if(colors[3] > colors[1] && colors[3] > colors[2]){
//            return Color.BLUE;
//        }
    }
}
