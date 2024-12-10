//package org.firstinspires.ftc.teamcode.Bot.OpModes.Tests;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.Bot.Bot;
//import org.firstinspires.ftc.teamcode.Bot.Drivetrain.Drivetrain;
//import org.firstinspires.ftc.teamcode.Bot.Mechanisms.SmartClaw;
//import org.firstinspires.ftc.teamcode.Bot.Sensors.Vision.Camera;
//import org.firstinspires.ftc.teamcode.Bot.Sensors.Vision.Pipelines.Contour;
//import org.firstinspires.ftc.teamcode.Bot.Setup;
//import org.firstinspires.ftc.teamcode.PedroPathing.localization.Pose;
//
//@Autonomous(name = "Claw test for vis")
//public class ClawTest extends LinearOpMode {
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        Drivetrain drivetrain = new Drivetrain();
//        Setup setup = new Setup(hardwareMap, telemetry, true, this, Setup.OpModeType.TELEOP, Setup.Team.Q1);
//        drivetrain.init(new Pose());
//        Bot bot = new Bot(Setup.mechStates, Setup.sensorStates);
//        Camera cam = new Camera("Red", hardwareMap);
//        cam.setPipeline(Camera.basePipelines.Contour);
//        cam.openCamera();
//        SmartClaw claw = new SmartClaw();
//        while(opModeInInit()){
//            if(gamepad1.a){
//                bot.clawRot.setTarget(claw.rotateTarget());
//                telemetry.addLine("Moving Servo");
//            }
//            telemetry.addLine("Theta: " + Contour.theta);
//            telemetry.addLine("Servo Target: " + claw.rotateTarget());
//            telemetry.update();
//        }
//    }
//}
