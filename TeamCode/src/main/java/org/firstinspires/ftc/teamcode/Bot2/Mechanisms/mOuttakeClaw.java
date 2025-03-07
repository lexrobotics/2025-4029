package org.firstinspires.ftc.teamcode.Bot2.Mechanisms;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Bot2.Mechanisms.AbstractMechanisms.ServoMechanism;
@Config
public class mOuttakeClaw extends ServoMechanism {
    public mOuttakeClaw() {
        super("OuttakeClaw");
    }
    public static double MAX = 0;
    public static double MIN = 0;
    public static double INTAKE = 0;
    public static double INIT = 0.831;
    public static double REST = 0.0393;
    public static double CLOSE = 0.831;
    public static double OPEN = 0.0393;
    public static double TRANSFER = CLOSE;
    public static double TRANSFER_PREP = OPEN;
}
