package org.firstinspires.ftc.teamcode.Bot2.Mechanisms;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Bot2.Mechanisms.AbstractMechanisms.ServoMechanism;
@Config
public class mTurret extends ServoMechanism {
    public mTurret(){ super("Turret"); }
    public static double INIT = 0.55;
    public static double REST = 0.5;
    public static double MAX = 1;
    public static double MIN = 0;
    public static double HORIZONTAL = 0.5;
    public static double VERTICAL = 0.7;
    public static double DIAGONAL_LEFT = 0.8;
    public static double DIAGONAL_RIGHT = 0.2;
    public static double TRANSFER = 0.3;
    public static double TRANSFER_PREP = 0.3;

}
