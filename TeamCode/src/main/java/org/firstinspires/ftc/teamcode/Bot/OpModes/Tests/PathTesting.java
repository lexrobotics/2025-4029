package org.firstinspires.ftc.teamcode.Bot.OpModes.Tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Bot.Bot;
import org.firstinspires.ftc.teamcode.Bot.Sensors.IMUStatic;
import org.firstinspires.ftc.teamcode.Bot.Setup;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

public class PathTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Setup setup = new Setup(hardwareMap, telemetry, true, this, Setup.OpModeType.TELEOP, Setup.Team.Q1);
        Bot bot = new Bot(Setup.mechStates, Setup.sensorStates);
        IMUStatic imu = new IMUStatic();
        Follower follower = new Follower(hardwareMap);

        boolean isBlue = false;
        boolean isLeft = false;
        TrajectorySequence[] traj = new TrajectorySequence[3];
        while(opModeInInit()) {
            if (gamepad1.right_bumper) {
                isBlue = false;

            } else if (gamepad1.left_bumper) {
                isBlue = true;
            }
            if(gamepad1.dpad_left){
                isLeft = true;
            } else if(gamepad1.dpad_right){
                isLeft = false;
            }
        }


        waitForStart();
        if(isBlue){
            if(isLeft){
                bot.initDrivetrain(new Pose2d(12,60,Math.toRadians(90)));
                traj[0] = bot.drivetrain.buildTrajectorySequence(new Pose2d(bot.drivetrain.currentPos.getX(), bot.drivetrain.currentPos.getY(), bot.drivetrain.currentPos.getHeading()))
                        .splineToConstantHeading(new Vector2d(12,36), Math.toRadians(90))
                        .build();
                traj[1] = bot.drivetrain.buildTrajectorySequence(new Pose2d(bot.drivetrain.currentPos.getX(), bot.drivetrain.currentPos.getY(), bot.drivetrain.currentPos.getHeading()))
                        .splineToLinearHeading(new Pose2d(48,44,Math.toRadians(270)), Math.toRadians(270))
                        .build();
                traj[2] = bot.drivetrain.buildTrajectorySequence(new Pose2d(bot.drivetrain.currentPos.getX(), bot.drivetrain.currentPos.getY(), bot.drivetrain.currentPos.getHeading()))
                        .lineToLinearHeading(new Pose2d(60,60,Math.toRadians(225)))
                        .build();
            } else {
                bot.initDrivetrain(new Pose2d(-12,60,Math.toRadians(90)));
                traj[0] = bot.drivetrain.buildTrajectorySequence(new Pose2d(bot.drivetrain.currentPos.getX(), bot.drivetrain.currentPos.getY(), bot.drivetrain.currentPos.getHeading()))
                        .splineToConstantHeading(new Vector2d(-20,36), Math.toRadians(90))
                        .build();
                traj[1] = bot.drivetrain.buildTrajectorySequence(new Pose2d(bot.drivetrain.currentPos.getX(), bot.drivetrain.currentPos.getY(), bot.drivetrain.currentPos.getHeading()))
                        .splineToLinearHeading(new Pose2d(48,44,Math.toRadians(270)), Math.toRadians(270))
                        .build();
                traj[2] = bot.drivetrain.buildTrajectorySequence(new Pose2d(bot.drivetrain.currentPos.getX(), bot.drivetrain.currentPos.getY(), bot.drivetrain.currentPos.getHeading()))
                        .lineToLinearHeading(new Pose2d(-60,60,Math.toRadians(270)))
                        .build();
            }
        } else {
            if(isLeft){
                bot.initDrivetrain(new Pose2d(-12,-60,Math.toRadians(270)));
                traj[0] = bot.drivetrain.buildTrajectorySequence(new Pose2d(bot.drivetrain.currentPos.getX(), bot.drivetrain.currentPos.getY(), bot.drivetrain.currentPos.getHeading()))
                        .splineToConstantHeading(new Vector2d(-12,-36), Math.toRadians(270))
                        .build();
                traj[1] = bot.drivetrain.buildTrajectorySequence(new Pose2d(bot.drivetrain.currentPos.getX(), bot.drivetrain.currentPos.getY(), bot.drivetrain.currentPos.getHeading()))
                        .splineToLinearHeading(new Pose2d(-48,-44,Math.toRadians(90)), Math.toRadians(90))
                        .build();
                traj[2] = bot.drivetrain.buildTrajectorySequence(new Pose2d(bot.drivetrain.currentPos.getX(), bot.drivetrain.currentPos.getY(), bot.drivetrain.currentPos.getHeading()))
                        .lineToLinearHeading(new Pose2d(-60,-60,Math.toRadians(45)))
                        .build();
            } else {
                bot.initDrivetrain(new Pose2d(12,-60,Math.toRadians(270)));
                traj[0] = bot.drivetrain.buildTrajectorySequence(new Pose2d(bot.drivetrain.currentPos.getX(), bot.drivetrain.currentPos.getY(), bot.drivetrain.currentPos.getHeading()))
                        .splineToConstantHeading(new Vector2d(12,-36), Math.toRadians(270))
                        .build();
                traj[1] = bot.drivetrain.buildTrajectorySequence(new Pose2d(bot.drivetrain.currentPos.getX(), bot.drivetrain.currentPos.getY(), bot.drivetrain.currentPos.getHeading()))
                        .splineToLinearHeading(new Pose2d(48,-44,Math.toRadians(90)), Math.toRadians(90))
                        .build();
                traj[2] = bot.drivetrain.buildTrajectorySequence(new Pose2d(bot.drivetrain.currentPos.getX(), bot.drivetrain.currentPos.getY(), bot.drivetrain.currentPos.getHeading()))
                        .lineToLinearHeading(new Pose2d(60,-60,Math.toRadians(90)))
                        .build();
            }
        }
//        while(opModeIsActive()){
            for(TrajectorySequence t : traj){
                bot.drivetrain.setTrajectorySequence(t);
                while(bot.drivetrain.isBusy()){
                    bot.drivetrain.update();
                }
                wait(5);
            }
        }
//    }
}
