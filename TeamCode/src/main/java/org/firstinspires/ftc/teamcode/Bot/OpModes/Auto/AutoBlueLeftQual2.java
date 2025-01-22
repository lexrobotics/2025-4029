package org.firstinspires.ftc.teamcode.Bot.OpModes.Auto;

import android.provider.SyncStateContract;
import android.view.animation.PathInterpolator;

import org.firstinspires.ftc.teamcode.PedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.PedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.PedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.PedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.PedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.PedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.PedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.PedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.PedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.Bot.StartPositions;
import org.firstinspires.ftc.teamcode.PedroPathing.tuning.FollowerConstants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous (name = "Auto Blue Left", group = "Qual 2")
public class AutoBlueLeftQual2 extends OpMode {

    private Follower follower;
    private Timer opmodeTimer, pathTimer;

    private int pathState;

    private final Pose score = new Pose(16, 128, Math.toRadians(135));

    private final Pose spikeOne = new Pose(35,120, Math.toRadians(0));

    private final Pose spikeTwo = new Pose(35, 132, Math.toRadians(0));

    private final Pose spikeThree = new Pose(44, 132, Math.toRadians(90));

    private final Pose spikeThreeControl = new Pose(46.092, 115.817, Math.toRadians(90));

    private final Pose parkPose = new Pose(60,90,Math.toRadians(270));

    private final Pose startPose = StartPositions.blueLeftDrivePos;

    private Path park;
    private PathChain scorePreload, getSample1, scoreSample1, getSample2, scoreSample2, getSample3, scoreSample3;


    public void buildPaths(){

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(score)))
                .setLinearHeadingInterpolation(startPose.getHeading(), score.getHeading())
                .build();

        getSample1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score), new Point(spikeOne)))
                .setLinearHeadingInterpolation(score.getHeading(), spikeOne.getHeading())
                .build();

        scoreSample1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(spikeOne), new Point(score)))
                .setLinearHeadingInterpolation(spikeOne.getHeading(), score.getHeading())
                .build();

        getSample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(score), new Point(spikeTwo)))
                .setLinearHeadingInterpolation(score.getHeading(), spikeTwo.getHeading())
                .build();

        scoreSample2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(spikeTwo), new Point(score)))
                .setLinearHeadingInterpolation(spikeTwo.getHeading(), score.getHeading())
                .build();

        getSample3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(score), new Point(spikeThreeControl), new Point(spikeThree)))
                .setLinearHeadingInterpolation(score.getHeading(), spikeThree.getHeading())
                .build();

        scoreSample3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(spikeThree), new Point(score)))
                .setLinearHeadingInterpolation(spikeThree.getHeading(), score.getHeading())
                .build();

        park = new Path(new BezierLine(new Point(score), new Point(parkPose)));
        park.setLinearHeadingInterpolation(score.getHeading(), parkPose.getHeading());
    }

    public void autonomousPathUpdate(){
        switch (pathState){
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()){
                    // score mechanism
                    follower.followPath(getSample1, true);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()){
                    // intake mechanism
                    follower.followPath(scoreSample1, true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()){
                    // score mechanism
                    follower.followPath(getSample2, true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    // intake mechanism
                    follower.followPath(scoreSample2, true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()){
                    // score mechanism
                    follower.followPath(getSample3, true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()){
                    // intake mechanism
                    follower.followPath(scoreSample3, true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()){
                    // score mechanism
                    follower.followPath(park);
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init(){
        opmodeTimer = new Timer();
        pathTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        //Constants.setConstants(FollowerConstants.class, LConstants.class);
        buildPaths();
    }

    @Override
    public void init_loop() {}

    @Override
    public void loop() {

        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("x:", follower.getPose().getX());
        telemetry.addData("y:", follower.getPose().getY());
        telemetry.addData("Heading:", follower.getPose().getHeading());
        telemetry.update();
    }
}
