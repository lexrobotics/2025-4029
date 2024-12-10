package org.firstinspires.ftc.teamcode.Bot.Sensors.Vision.Pipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class AnythingElse extends OpenCvPipeline {

    public static double theta = 0.0;

    private boolean isBlue;

    private Mat HSV;
    private Mat red;
    private Mat red2;
    private Mat blue;

    private Mat dummy;

    private Scalar upperRed = new Scalar(15,255,255);
    private Scalar lowerRed = new Scalar(0,50,50);
    private Scalar upperRed2 = new Scalar(180,255,255);
    private Scalar lowerRed2 = new Scalar(170,50,100);

    private Scalar outlines = new Scalar(0, 0, 255);

    private Point centerPoint = new Point(640,360);
    private double minBoxSize = 300.0;

    private int currentArrayPos;
    private int biggestBox;
    private double biggestBoxSize;
    private double currentArea;

    public void init(){
        HSV = new Mat();

        red = new Mat();
        red2 = new Mat();
        dummy = new Mat();
    }
    @Override
    public Mat processFrame(Mat input) {

        Core.inRange(HSV, lowerRed, upperRed, red);
        Core.inRange(HSV, lowerRed2, upperRed2, red2);
        Core.add(red, red2, red);



        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(red, contours, dummy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        if(!contours.isEmpty()) {
            currentArrayPos = 0;
            biggestBox = 0;
            biggestBoxSize = -1;
            currentArea = -1;
            for (MatOfPoint contour : contours) {
                currentArea = Imgproc.contourArea(contour);
                if (currentArea > biggestBoxSize && currentArea > minBoxSize) {
                    biggestBox = currentArrayPos;
                    biggestBoxSize = currentArea;
                }
                currentArrayPos++;
            }
            if(biggestBoxSize != -1) {
                MatOfPoint2f points2f = null;
                points2f.fromArray(contours.get(biggestBox).toArray());
                RotatedRect fitBound = Imgproc.minAreaRect(points2f);
                theta = fitBound.angle;
                Rect boundingRect = Imgproc.boundingRect(contours.get(biggestBox));
                Imgproc.putText(HSV, "Detected Area: " + biggestBoxSize, new Point(boundingRect.tl().x, boundingRect.tl().y + boundingRect.height), 2, 1, outlines);
                Imgproc.rectangle(HSV, boundingRect.tl(), boundingRect.br(), outlines, 2);
                centerPoint = new Point(boundingRect.tl().x + boundingRect.width / 2, boundingRect.tl().y + boundingRect.height / 2);
                Imgproc.circle(HSV, centerPoint, 10, outlines, 2);

                Imgproc.cvtColor(HSV, input, Imgproc.COLOR_HSV2RGB);
            } else {
                centerPoint = new Point(-1, -1);
            }

        } else {
            centerPoint = new Point(-1, -1);
        }
        return input;
    }
    /**
     * Returns the centerpoint of the largest contour that is detected by the camera.
     * @return Point centerPoint
     * */
    public Point getCenterPoint(){return centerPoint;}
}
