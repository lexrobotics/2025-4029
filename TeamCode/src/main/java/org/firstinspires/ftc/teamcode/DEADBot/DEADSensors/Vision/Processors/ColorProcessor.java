package org.firstinspires.ftc.teamcode.DEADBot.DEADSensors.Vision.Processors;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;

public class ColorProcessor implements VisionProcessor {
    @Override
    public void init(int width, int height, CameraCalibration calibration){

    }
    @Override
    public Mat processFrame (Mat frame, long captureTimeNanos){
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpToCanvasPx, float scaleCanvasDensity, Object userContext){

    }
}
