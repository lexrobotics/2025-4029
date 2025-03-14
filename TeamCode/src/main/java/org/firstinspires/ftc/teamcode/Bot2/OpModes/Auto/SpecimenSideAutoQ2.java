//package org.firstinspires.ftc.teamcode.Bot2.OpModes.Auto;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.Bot2.Old.Bot1;
//import org.firstinspires.ftc.teamcode.Bot2.Old.Setup1;
//import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
//
///*
// * This is a simple routine to test translational drive capabilities.
// */
//@Config
//@Autonomous(group = "0")
//public class SpecimenSideAutoQ2 extends LinearOpMode {
//    private Setup1 setup;
//    private Bot1 bot;
//    private ElapsedTime timer;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        timer = new ElapsedTime();
//        setup = new Setup1(hardwareMap, telemetry, true, this, Setup1.OpModeType.AUTO, Setup1.Team.Q3);
//
//        bot = new Bot1(Setup1.mechStates, Setup1.sensorStates);
//        bot.init();
//
//        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d())
//                .forward(3)
//                .strafeRight(40)
//                .build();
//
//
//        waitForStart();
//        timer.reset();
//
//        if (isStopRequested()) return;
//
//        drive.followTrajectorySequence(trajectory);
//
//        while (!isStopRequested() && opModeIsActive()) ;
//    }
//}
