package org.firstinspires.ftc.teamcode.Bot2.Sensors;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.teamcode.Bot2.Setup;

public class SensorColor {
    private ColorSensor sensor;
    public SensorColor(String name) {
        sensor = Setup.hardwareMap.get(ColorSensor.class, name);
    }

    public double[] getColor(){
        return new double[]{sensor.alpha(),sensor.red(),sensor.green(),sensor.blue()};
    }

    public void setI2cAddress(I2cAddr newAddress){
        sensor.setI2cAddress(newAddress);
    }

    public void enableLed(boolean isOn){
        sensor.enableLed(isOn);
    }
}
