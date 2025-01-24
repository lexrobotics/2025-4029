package org.firstinspires.ftc.teamcode.PedroPathing.constants;


import org.firstinspires.ftc.teamcode.PedroPathing.localization.Encoder;
import org.firstinspires.ftc.teamcode.PedroPathing.localization.ThreeWheelLocalizer;


public class LConstants {
    static {
        ThreeWheelLocalizer.FORWARD_TICKS_TO_INCHES = 0.00052; //lower it is, lower the telemetry value will be
        ThreeWheelLocalizer.STRAFE_TICKS_TO_INCHES = 0.00052;
        ThreeWheelLocalizer.TURN_TICKS_TO_RADIANS = 0.00052;
        ThreeWheelLocalizer.leftY = 6.125;
        ThreeWheelConstants.rightY = -6.125;
        ThreeWheelConstants.strafeX = -4.4;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "rightFront";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "leftRear";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "rightRear";
        ThreeWheelLocalizer.leftEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.rightEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
    }
}




