package org.firstinspires.ftc.teamcode.Bot.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Bot.Sensors.SensorSwitch;
import org.firstinspires.ftc.teamcode.Bot.Old.Setup1;

public class OuttakeSlidesSmart extends OuttakeSlides {
    private SensorSwitch sensorSwitch;
    private boolean lastState = false;
    public OuttakeSlidesSmart(){super();}

    @Override
    public void init(double target, HardwareMap hwm){
        super.init(target);
        sensorSwitch = new SensorSwitch("SlidesSwitch", true, hwm);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    @Override
    public void init(double target){
        super.init(target);
        sensorSwitch = new SensorSwitch("SlidesSwitch", true);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void telemetry(){
        super.telemetry();
        Setup1.telemetry.addData(name + " SwitchState", sensorSwitch.getStatus());
        Setup1.telemetry.addData(name + " OSlidesPos", motor.getCurrentPosition());
    }

    @Override
    public void update(){
        boolean state = sensorSwitch.getStatus();
        Setup1.telemetry.addData(name + " SwitchState", sensorSwitch.getStatus());
        if (state && !lastState) {
            reset();
        }
        if(state){
            currentPos = 0;
        }
        lastState = state;

        currentPos = motor.getCurrentPosition();
        if (targetPos == 0 && currentPos < margin && !state){
            motor.setTargetPosition((int)(currentPos - margin*1.5));
        }else {
            motor.setTargetPosition((int)targetPos);
        }
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(velocity);
    }
}
