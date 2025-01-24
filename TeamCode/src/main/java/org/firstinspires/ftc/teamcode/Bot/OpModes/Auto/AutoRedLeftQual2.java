package org.firstinspires.ftc.teamcode.Bot.OpModes.Auto;

import android.provider.SyncStateContract;
import android.view.animation.PathInterpolator;

import org.firstinspires.ftc.teamcode.Bot.Bot;
import org.firstinspires.ftc.teamcode.Bot.States.ActionSequences;
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
public class AutoRedLeftQual2 extends OpMode{
    private Follower follower;
    private Timer opmodeTimer, pathTimer;
    private Bot bot;
    private ActionSequences AS;

    private int pathState;

    private final Pose score = new Pose(128, 16, Math.toRadians(315));

    private final Pose spikeOne = new Pose(109,24, Math.toRadians(180));

    private final Pose spikeTwo = new Pose(109, 13, Math.toRadians(180));

    private final Pose spikeThree = new Pose(109, 12, Math.toRadians(270));

    private final Pose spikeThreeControl = new Pose(93.211, 26.569, Math.toRadians(90));

    private final Pose parkPose = new Pose(84,40,Math.toRadians(90));

    private final Pose startPose = StartPositions.redLeftDrivePos;

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
                    AS.sampleScoring(2);
                    AS.rest();
                    follower.followPath(getSample1, true);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()){
                    AS.intake();
                    AS.rest();
                    follower.followPath(scoreSample1, true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()){
                    AS.sampleScoring(2);
                    AS.rest();
                    follower.followPath(getSample2, true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    AS.intake();
                    AS.rest();
                    follower.followPath(scoreSample2, true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()){
                    AS.sampleScoring(2);
                    AS.rest();
                    follower.followPath(getSample3, true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()){
                    AS.intake();
                    AS.rest();
                    follower.followPath(scoreSample3, true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()){
                    AS.sampleScoring(2);
                    AS.rest();
                    follower.followPath(park);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()){
                    AS.hang(0);
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
        AS = new ActionSequences(bot);
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
