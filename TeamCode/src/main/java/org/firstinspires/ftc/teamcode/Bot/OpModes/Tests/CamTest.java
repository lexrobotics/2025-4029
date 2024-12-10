package org.firstinspires.ftc.teamcode.Bot.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Bot.Sensors.Vision.Pipelines.AnythingElse;
import org.firstinspires.ftc.teamcode.Bot.Sensors.Vision.Camera;
import org.firstinspires.ftc.teamcode.Bot.Sensors.Vision.Pipelines.Contour;

@Autonomous
public class CamTest extends LinearOpMode {
    private final boolean IS_IN_ADVANCED_CS = true;
    @Override
    public void runOpMode() throws InterruptedException {
        Contour pipeline = new Contour();
        pipeline.init(true);
        Camera cam = new Camera("webcam","Blue", hardwareMap);
        if(IS_IN_ADVANCED_CS) {
            AnythingElse pip = new AnythingElse();
            pip.init();
            cam.setPipeline(pip);
        } else {
            cam.setPipeline(Camera.basePipelines.Contour);
        }
        cam.openCamera();
        waitForStart();
    }
}
