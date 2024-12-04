package org.firstinspires.ftc.teamcode.Bot.Drivetrain;

import static org.firstinspires.ftc.teamcode.Bot.Setup.telemetry;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.getAccelerationConstraint;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.getVelocityConstraint;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.Bot.Sensors.IMUStatic;
import org.firstinspires.ftc.teamcode.Bot.Setup;
//import org.firstinspires.ftc.teamcode.PedroPathing.follower.Follower;
//import org.firstinspires.ftc.teamcode.PedroPathing.localization.Pose;
//import org.firstinspires.ftc.teamcode.PedroPathing.localization.ThreeWheelLocalizer;
//import org.firstinspires.ftc.teamcode.PedroPathing.pathGeneration.BezierCurve;
//import org.firstinspires.ftc.teamcode.PedroPathing.pathGeneration.MathFunctions;

import java.util.ArrayList;
import java.util.List;

//import org.firstinspires.ftc.teamcode.PedroPathing.pathGeneration.Path;
//import org.firstinspires.ftc.teamcode.PedroPathing.pathGeneration.PathChain;
//import org.firstinspires.ftc.teamcode.PedroPathing.pathGeneration.Point;
//import org.firstinspires.ftc.teamcode.PedroPathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

public class Drivetrain {

    public enum DriveState{
        TRAJECTORY_RR,
        POWER_RR,
        POWER,
        POWER_APRIL_TAG
    }
    protected DriveState state = DriveState.POWER;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(DriveConstants.MAX_ACCEL);

    protected TrajectorySequence trajectory;

    protected SampleMecanumDrive drive;

    public Pose2d currentPos;
//    public Vector targetDriveVector;
//    public Vector targetHeadingVector;
    public double[] teleOpTargets;
//    protected Follower follower;
    private IMUStatic imu;
    protected double imuOffset;


    public Drivetrain(){
        teleOpTargets = new double[3];
    }

    public void init(Pose2d startPose) {
        currentPos = startPose;
        drive = new SampleMecanumDrive(Setup.hardwareMap);
        drive.setPoseEstimate(startPose);
//        follower = new Follower(Setup.hardwareMap, false);
//        follower.setStartingPose(new Pose(startPose.getX(), startPose.getY(), startPose.getHeading()));
//        targetDriveVector = new Vector();
//        targetHeadingVector = new Vector();
        teleOpTargets = new double[3];
        imu = new IMUStatic();
//        telemetry.addLine("Follower: " + follower.driveError);
    }

    public void update(boolean usePeP){
        if(usePeP){
//            follower.setMovementVectors(follower.getCentripetalForceCorrection(), targetHeadingVector, targetDriveVector);
//            follower.update();
        }else{
            double x = teleOpTargets[0];
            double y = teleOpTargets[1];
            double spin = teleOpTargets[2];
////            follower.setMotorPowers(Range.clip(y + x, -1, 1) + spin,
//                    Range.clip(y - x, -1, 1) + spin,
//                    Range.clip(y + x, -1, 1) - spin,
//                    Range.clip(y - x, -1, 1) - spin);
        }
    }
    public void update() {
////        currentPos = drive.getPoseEstimate();
//        if (state == DriveState.TRAJECTORY_RR) {
//            drive.update();
//        } else if (state == DriveState.POWER_RR) {
//            Vector2d input = new Vector2d(
//                    teleOpTargets[1],
//                    -teleOpTargets[0]
//            ).rotated(-currentPos.getHeading());
//            drive.setWeightedDrivePower(
//                    new Pose2d(
//                            input.getX(),
//                            input.getY(),
//                            -teleOpTargets[2]
//                    )
//            );
//        } else if (state == DriveState.POWER) { // spin may be (+) instead of (-)
            double x = teleOpTargets[0];
            double y = teleOpTargets[1];
            double spin = teleOpTargets[2];
            drive.setMotorPowers(Range.clip(y + x, -1, 1) + spin,
                    Range.clip(y - x, -1, 1) + spin,
                    Range.clip(y + x, -1, 1) - spin,
                    Range.clip(y - x, -1, 1) - spin);
//        }else if (state == DriveState.POWER_APRIL_TAG) { // spin may be (-) instead of (+)
//            double x = teleOpTargets[0];
//            double y = teleOpTargets[1];
//            double yaw = teleOpTargets[2];
//
//            double leftFrontPower    =  x -y -yaw;
//            double rightFrontPower   =  x +y +yaw;
//            double leftBackPower     =  x +y -yaw;
//            double rightBackPower    =  x -y +yaw;
//
//            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
//            max = Math.max(max, Math.abs(leftBackPower));
//            max = Math.max(max, Math.abs(rightBackPower));
//
//            if (max > 1.0) {
//                leftFrontPower /= max;
//                rightFrontPower /= max;
//                leftBackPower /= max;
//                rightBackPower /= max;
//            }
//
//            drive.setMotorPowers( leftFrontPower,
//                    leftBackPower,
//                    rightBackPower,
//                    rightFrontPower);
//        }
    }

    public void telemetry(){
        telemetry.addData("Drivetrain currentPos", currentPos);
    }

//    public void setTargetVectors(double x, double y, double turn){
//        double theta = (imu.getYaw(AngleUnit.RADIANS));
//        x = MathFunctions.clamp(x,0,1);
//        y = MathFunctions.clamp(y,0,1);
//        double[] coordinates = CartesianToPolar(x,y);
//        coordinates[1] += theta;
//        targetDriveVector.setMagnitude(coordinates[0]);
//        targetDriveVector.setTheta(coordinates[1]);
////        targetDriveVector.setOrthogonalComponents(-y, -x);
////        targetDriveVector.setMagnitude(MathFunctions.clamp(targetDriveVector.getMagnitude(), 0, 1));
////        targetDriveVector.rotateVector(follower.getPose().getHeading());
//        targetHeadingVector.setComponents(turn, follower.getPose().getHeading());
//    }
    public void setTeleOpTargets(double x, double y, double theta){
        this.teleOpTargets = new double[]{x, y, theta};
    }


    public double[] CartesianToPolar(double x, double y) {
        return new double[]{Math.sqrt(x * x + y * y), Math.atan2(y, x)};
    }
//    private double[] PolarToCartesian(double r, double theta){
//        return new double[]{};
//    }


    public double getHeadingIMU() {return drive.getRawExternalHeading();}
    public void resetIMU(){imu.resetYaw();}

//    public PathChain BLInitToScoreClip(){
//        return new PathChain(new Path(new BezierCurve(new Point(13.6,83.5,0), new Point(13.6,96.3,1), new Point(62.4,123.6,2), new Point(62.4,100.2,3))));
//    }

    public TrajectorySequenceBuilder buildTrajectorySequence(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                DriveConstants.MAX_ANG_VEL,DriveConstants. MAX_ANG_ACCEL
        );
    }
    public boolean isBusy() {
        if (state == DriveState.TRAJECTORY_RR) {
            return drive.isBusy();
        } else {
            return false;
        }
    }
    public void setTrajectorySequence(TrajectorySequence traj){
        if(traj != null) {
            state = DriveState.TRAJECTORY_RR;
            trajectory = traj;
        }
    };

}
