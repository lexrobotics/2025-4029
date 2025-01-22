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

@Autonomous(name = "Auto Red Right", group = "Qual 2")
public class AutoRedRightQual2 extends OpMode{

    private Follower follower;
    private Timer opmodeTimer, pathTimer;

    private int pathState;

    private final Pose startPose = StartPositions.redRightDrivePos;
    private final Pose startPoseControl = new Pose(116.991, 108.037, Math.toRadians(0));

    private final Pose scorePose = new Pose(108,80, Math.toRadians(180));
    private final Pose scorePoseControl = new Pose(133.871, 80, Math.toRadians(0));

    private final Pose pushPose1 = new Pose(84, 120, Math.toRadians(0));
    private final Pose pushPose1Control1 = new Pose(128.147, 103.633, Math.toRadians(0));
    private final Pose pushPose1Control2 = new Pose(121.248, 92.771, Math.toRadians(0));
    private final Pose pushPose1Control3 = new Pose(68.11, 119.633, Math.toRadians(0));

    private final Pose obsPose1 = new Pose(132,120, Math.toRadians(0));

    private final Pose pushPose2 = new Pose(84,130, Math.toRadians(0));
    private final Pose pushPose2Control1 = new Pose(100.55, 114.055, Math.toRadians(0));
    private final Pose pushPose2Control2 = new Pose(55.927, 129.908, Math.toRadians(0));

    private final Pose obsPose2 = new Pose(132, 130, Math.toRadians(0));

    private final Pose parkPose = new Pose(84,100, Math.toRadians(270));
    private final Pose parkPoseControl1 = new Pose(111.266, 127.56, Math.toRadians(0));
    private final Pose parkPoseControl2 = new Pose(84.257, 118.899, Math.toRadians(0));

    private Path park;
    private PathChain preloadScore, push1, obs1, push2, obs2, toStart, score, pickup;

    public void buildPaths() {

        preloadScore = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startPose), new Point(scorePoseControl), new Point(scorePose)))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();

        push1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(pushPose1Control1), new Point(pushPose1Control2), new Point(pushPose1Control3),new Point(pushPose1)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pushPose1.getHeading())
                .build();

        obs1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushPose1), new Point(obsPose1)))
                .setConstantHeadingInterpolation(obsPose1.getHeading())
                .build();

        push2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(obsPose1), new Point(pushPose2Control1), new Point(pushPose2Control2), new Point(pushPose2)))
                .setConstantHeadingInterpolation(pushPose2.getHeading())
                .build();

        obs2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pushPose2), new Point(obsPose2)))
                .setConstantHeadingInterpolation(obsPose2.getHeading())
                .build();

        toStart = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(obsPose2), new Point(startPoseControl), new Point(startPose)))
                .setConstantHeadingInterpolation(obsPose2.getHeading())
                .build();

        score = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startPose), new Point(scorePoseControl), new Point(scorePose)))
                .setLinearHeadingInterpolation(obsPose2.getHeading(), scorePose.getHeading())
                .build();

        pickup = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(scorePoseControl), new Point(startPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), obsPose2.getHeading())
                .build();

        park = new Path(new BezierCurve(new Point(scorePose), new Point(parkPoseControl1), new Point(parkPoseControl2), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                follower.followPath(preloadScore, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    // score mechanism
                    follower.followPath(push1, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(obs1, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(push2, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(obs2, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()){
                    follower.followPath(toStart, true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()){
                    //specimen pickup
                    follower.followPath(score, true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()){
                    //score
                    follower.followPath(pickup, true);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()){
                    //specimen pickup
                    follower.followPath(score, true);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()){
                    // score
                    follower.followPath(park);
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy()){
                    // ascent
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
