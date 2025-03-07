package org.firstinspires.ftc.teamcode.Bot2.Mechanisms;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Bot2.Mechanisms.AbstractMechanisms.ServoMechanism;
@Config
public class mIntakeWrist extends ServoMechanism {
    public mIntakeWrist() {
        super("IntakeWrist");
    }
    public static double INIT = 0.514;
    public static double TRANSFER = 1;

    public static final double MAX = 1;
    public static final double MIN = 0;

    public static final double INTAKE = 0.661;
    public static final double INTAKE_PREP = 0.661;

    public static final double SPECIMEN = 0.395;
    public static final double REST = 0.298;
    public static final double BUCKET = 0.964;

}
