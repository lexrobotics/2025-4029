package org.firstinspires.ftc.teamcode.Bot.Drivetrain;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

public class RoadRunnerPaths {

    TrajectorySequence scoreSpec, intakeSample, toHumanPlayer, cycleSpec, toBucket, park;

    private double BOT_FRONT_TO_BACK_LENGTH = 16.5;
    private double BOT_LEFT_TO_RIGHT_LENGTH = 15.5;

    private TrajectorySequence[] redRightTrajectories = new TrajectorySequence[4];
    private TrajectorySequence[] blueLeftTrajectories = new TrajectorySequence[4];
    private TrajectorySequence[] redLeftTrajectories = new TrajectorySequence[4];
    private TrajectorySequence[] blueRightTrajectories = new TrajectorySequence[4];
    private TrajectorySequence[] rightParkTrajectories = new TrajectorySequence[2];
    private TrajectorySequence[] leftParkTrajectories = new TrajectorySequence[2];



    public TrajectorySequence buildRedRightTrajectory(Drivetrain drive, int trajectoryIndex){
        if(trajectoryIndex == 0){
            redRightTrajectories[0] = drive.buildTrajectorySequence(drive.currentPos)
                    .back(24)
                    .build();
            redRightTrajectories[1] = drive.buildTrajectorySequence(drive.currentPos)
                    .lineToLinearHeading(new Pose2d(6, -(24 + BOT_FRONT_TO_BACK_LENGTH + 3), Math.toRadians(270)))
                    .lineToLinearHeading(new Pose2d(33, -(24 + BOT_FRONT_TO_BACK_LENGTH + 3), Math.toRadians(270)))
                    .lineToLinearHeading(new Pose2d(33, -(12), Math.toRadians(270)))
                    .lineToLinearHeading(new Pose2d(45, -(12), Math.toRadians(270)))
                    .build();
            redRightTrajectories[2] = drive.buildTrajectorySequence(drive.currentPos)
                    .lineToLinearHeading(new Pose2d(45, -(72-12), Math.toRadians(270)))
                    .build();
            redRightTrajectories[3] = drive.buildTrajectorySequence(drive.currentPos)
                    .lineToLinearHeading(new Pose2d(30, 12, Math.toRadians(270)))
                    .build();
        }
        return redRightTrajectories[trajectoryIndex];
    }
    public TrajectorySequence buildBlueRightTrajectory(Drivetrain drive, int trajectoryIndex){
        blueRightTrajectories[0] = drive.buildTrajectorySequence(drive.currentPos)
                .back(24)
                .build();
        blueRightTrajectories[1] = drive.buildTrajectorySequence(drive.currentPos)
                .lineToLinearHeading(new Pose2d(-6, (24+15), Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-33, (24+15), Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-33, (12), Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-45, (12), Math.toRadians(90)))
                .build();
        blueRightTrajectories[2] = drive.buildTrajectorySequence(drive.currentPos)
                .lineToLinearHeading(new Pose2d(-45, (72-12), Math.toRadians(90)))
                .build();
        blueRightTrajectories[3] = drive.buildTrajectorySequence(drive.currentPos)
                .lineToLinearHeading(new Pose2d(-30, 12, Math.toRadians(90)))
                .build();
        return blueRightTrajectories[trajectoryIndex];
    }
    public void buildRedLeftTrajectory(){

    }
    public void buildBlueLeftTrajectory(){

    }

    public TrajectorySequence buildRightParkTrajectory(Drivetrain drive, int trajectoryIndex){
        if(trajectoryIndex == 0){
            rightParkTrajectories[0] = drive.buildTrajectorySequence(drive.currentPos)
                    .lineToLinearHeading(new Pose2d())
                    .build();
        }

        return rightParkTrajectories[trajectoryIndex];
    }
}
