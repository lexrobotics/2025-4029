package org.firstinspires.ftc.teamcode.Bot.Mechanisms;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Bot.Mechanisms.AbstractMechanisms.RunToPosMotorMechanism;
import org.firstinspires.ftc.teamcode.Bot.Mechanisms.AbstractMechanisms.ServoMechanism;
@Config

public class IntakeSlides extends RunToPosMotorMechanism {
    public IntakeSlides() {
        super("IntakeSlides");
    }
    public static  double INIT = 0;

    public static final double MIN = 0;
    public static final double MAX = 75;
    public static final double POS1 = 100;
    public static final double POS2 = 200;
    public static final double RST = 0;
    public static final double TRANSFER = 4;
}
