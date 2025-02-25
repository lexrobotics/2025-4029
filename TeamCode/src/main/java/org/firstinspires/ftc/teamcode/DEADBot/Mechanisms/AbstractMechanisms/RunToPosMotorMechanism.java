package org.firstinspires.ftc.teamcode.DEADBot.Mechanisms.AbstractMechanisms;

import org.firstinspires.ftc.teamcode.DEADBot.Old.Setup1;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public abstract class RunToPosMotorMechanism extends Mechanism{
    protected DcMotorEx motor;
    protected double margin = 50;
    public RunToPosMotorMechanism(String name) {
        super(name);
        velocity = 1;
    }

    @Override
    public void init(double target) {
        setTarget(target);
        motor = Setup1.hardwareMap.get(DcMotorEx.class, name);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void init(double target, DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        setTarget(target);
        motor = Setup1.hardwareMap.get(DcMotorEx.class, name);
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }
    @Override
    public void reverse(boolean isReversed){
        if(isReversed){
            motor.setDirection(DcMotorEx.Direction.REVERSE);
        }
    }
    @Override
    public void reset(){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void update() {
        currentPos = motor.getCurrentPosition();
        motor.setTargetPosition((int) targetPos);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(velocity);
    }

    @Override
    public boolean isBusy() {
        return Math.abs(targetPos - currentPos) >= margin;
    }
}
