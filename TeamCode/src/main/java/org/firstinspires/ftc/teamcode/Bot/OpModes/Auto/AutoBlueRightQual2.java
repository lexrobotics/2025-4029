//package org.firstinspires.ftc.teamcode.Bot.OpModes.Auto;
//
//import org.firstinspires.ftc.teamcode.Bot.Bot;
//import org.firstinspires.ftc.teamcode.Bot.States.ActionSequences;
//import org.firstinspires.ftc.teamcode.PedroPathing.constants.FConstants;
//import org.firstinspires.ftc.teamcode.PedroPathing.follower.Follower;
//import org.firstinspires.ftc.teamcode.PedroPathing.constants.LConstants;
//import org.firstinspires.ftc.teamcode.PedroPathing.localization.Pose;
//import org.firstinspires.ftc.teamcode.PedroPathing.pathGeneration.BezierCurve;
//import org.firstinspires.ftc.teamcode.PedroPathing.pathGeneration.BezierLine;
//import org.firstinspires.ftc.teamcode.PedroPathing.pathGeneration.Path;
//import org.firstinspires.ftc.teamcode.PedroPathing.pathGeneration.PathChain;
//import org.firstinspires.ftc.teamcode.PedroPathing.pathGeneration.Point;
//import org.firstinspires.ftc.teamcode.PedroPathing.util.Timer;
//import org.firstinspires.ftc.teamcode.Bot.StartPositions;
//
//import com.pedropathing.util.Constants;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//@Autonomous(name = "Auto Blue Right", group = "Qual 2")
//public class AutoBlueRightQual2 extends OpMode{
//
//    private Follower follower;
//    private Timer opmodeTimer, pathTimer;
//    private Bot bot;
//    private ActionSequences AS;
//
//    private int pathState;
//
//    private final Pose startPose = StartPositions.blueRightDrivePos;
//
//    private final Pose scorePose = new Pose(35,64, Math.toRadians(0));
//    private final Pose scorePoseControl = new Pose(17.615, 64, Math.toRadians(0));
//
//    private final Pose pushPose1 = new Pose(56, 24, Math.toRadians(180));
//    private final Pose pushPose1Control1 = new Pose(13.211, 39.046, Math.toRadians(0));
//    private final Pose pushPose1Control2 = new Pose(93.945, 21.578, Math.toRadians(0));
//
//    private final Pose obsPose1 = new Pose(12,24, Math.toRadians(180));
//
//    private final Pose pushPose2 = new Pose(56,15, Math.toRadians(180));
//    private final Pose pushPose2Control1 = new Pose(52.110, 29.211, Math.toRadians(0));
//    private final Pose pushPose2Control2 = new Pose(71.046, 21.431, Math.toRadians(0));
//
//    private final Pose obsPose2 = new Pose(12, 15, Math.toRadians(180));
//
//    private final Pose parkPose = new Pose(60,46, Math.toRadians(90));
//    private final Pose parkPoseControl1 = new Pose(29.651, 25.541, Math.toRadians(0));
//    private final Pose parkPoseControl2 = new Pose(53.578, 22.606, Math.toRadians(0));
//
//    private Path park;
//    private PathChain preloadScore, push1, obs1, push2, obs2, toPickup, score, pickup;
//
//    public void buildPaths() {
//
//        preloadScore = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(startPose), new Point(scorePoseControl), new Point(scorePose)))
//                .setConstantHeadingInterpolation(scorePose.getHeading())
//                .build();
//
//        push1 = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(scorePose), new Point(pushPose1Control1), new Point(pushPose1Control2), new Point(pushPose1)))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pushPose1.getHeading())
//                .build();
//
//        obs1 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(pushPose1), new Point(obsPose1)))
//                .setConstantHeadingInterpolation(obsPose1.getHeading())
//                .build();
//
//        push2 = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(obsPose1), new Point(pushPose2Control1), new Point(pushPose2Control2), new Point(pushPose2)))
//                .setConstantHeadingInterpolation(pushPose2.getHeading())
//                .build();
//
//        obs2 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(pushPose2), new Point(obsPose2)))
//                .setConstantHeadingInterpolation(obsPose2.getHeading())
//                .build();
//
//        toPickup = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(obsPose2), new Point(startPose)))
//                .setConstantHeadingInterpolation(obsPose2.getHeading())
//                .build();
//
//        score = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(startPose), new Point(scorePoseControl), new Point(scorePose)))
//                .setLinearHeadingInterpolation(obsPose2.getHeading(), scorePose.getHeading())
//                .build();
//
//        pickup = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(scorePose), new Point(scorePoseControl), new Point(startPose)))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), obsPose2.getHeading())
//                .build();
//
//        park = new Path(new BezierCurve(new Point(scorePose), new Point(parkPoseControl1), new Point(parkPoseControl2), new Point(parkPose)));
//        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
//    }
//
//    public void autonomousPathUpdate() {
//        switch(pathState) {
//            case 0:
//                follower.followPath(preloadScore, true);
//                setPathState(1);
//                break;
//            case 1:
//                if (!follower.isBusy()) {
//                    AS.specimenScorePrep(2);
//                    AS.rest();
//                    follower.followPath(push1, true);
//                    setPathState(2);
//                }
//                break;
//            case 2:
//                if (!follower.isBusy()) {
//                    follower.followPath(obs1, true);
//                    setPathState(3);
//                }
//                break;
//            case 3:
//                if (!follower.isBusy()) {
//                    follower.followPath(push2, true);
//                    setPathState(4);
//                }
//                break;
//            case 4:
//                if (!follower.isBusy()) {
//                    follower.followPath(obs2, true);
//                    setPathState(5);
//                }
//                break;
//            case 5:
//                if (!follower.isBusy()){
//                    follower.followPath(toPickup, true);
//                    setPathState(6);
//                }
//                break;
//            case 6:
//                if(!follower.isBusy()){
//                   AS.intakePrep();
//                   AS.rest();
//                   follower.followPath(score, true);
//                   setPathState(7);
//                }
//                break;
//            case 7:
//                if(!follower.isBusy()){
//                    AS.specimenScorePrep(2);
//                    AS.rest();
//                    follower.followPath(pickup, true);
//                    setPathState(8);
//                }
//                break;
//            case 8:
//                if(!follower.isBusy()){
//                    AS.intakePrep();
//                    AS.rest();
//                    follower.followPath(score, true);
//                    setPathState(9);
//                }
//                break;
//            case 9:
//                if(!follower.isBusy()){
//                    AS.specimenScorePrep(2);
//                    AS.rest();
//                    follower.followPath(park);
//                    setPathState(10);
//                }
//                break;
//            case 10:
//                if(!follower.isBusy()){
//                    AS.hang(0);
//                    setPathState(-1);
//                }
//                break;
//        }
//    }
//
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//    }
//
//    @Override
//    public void init(){
//        opmodeTimer = new Timer();
//        pathTimer = new Timer();
//        opmodeTimer.resetTimer();
//
//        follower = new Follower(hardwareMap);
//        follower.setStartingPose(startPose);
//        AS = new ActionSequences(bot);
//        Constants.setConstants(FConstants.class, LConstants.class);
//        buildPaths();
//    }
//
//    @Override
//    public void init_loop() {}
//
//    @Override
//    public void loop() {
//
//        follower.update();
//        autonomousPathUpdate();
//
//        telemetry.addData("Path State", pathState);
//        telemetry.addData("x:", follower.getPose().getX());
//        telemetry.addData("y:", follower.getPose().getY());
//        telemetry.addData("Heading:", follower.getPose().getHeading());
//        telemetry.update();
//    }
//}
