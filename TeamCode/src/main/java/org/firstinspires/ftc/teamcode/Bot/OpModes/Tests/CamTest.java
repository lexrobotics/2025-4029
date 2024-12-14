package org.firstinspires.ftc.teamcode.Bot.OpModes.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Bot.Sensors.Vision.Pipelines.AnythingElse;
import org.firstinspires.ftc.teamcode.Bot.Sensors.Vision.Camera;
import org.firstinspires.ftc.teamcode.Bot.Sensors.Vision.Pipelines.Contour;
@Config
@Autonomous
public class CamTest extends LinearOpMode {
    private static boolean IS_IN_ADVANCED_CS = false;
    @Override
    public void runOpMode() throws InterruptedException {
        Contour pipeline = new Contour();
        pipeline.init(false);
        Camera cam = new Camera("webcam","Red", hardwareMap);
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
