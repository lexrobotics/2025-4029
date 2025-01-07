package org.firstinspires.ftc.teamcode.Bot.Drivetrain;

import static org.firstinspires.ftc.teamcode.Bot.Setup.telemetry;
import static org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive.getAccelerationConstraint;
import static org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive.getVelocityConstraint;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Bot.Setup;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequenceBuilder;

public class Drivetrain {

    public enum DriveState{
        TRAJECTORY_RR,
        POWER,
    }
    protected DriveState state = DriveState.POWER;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(DriveConstants.MAX_ACCEL);

    protected TrajectorySequence trajectory;

    protected SampleMecanumDrive drive;

    public Pose2d currentPos;
    public double[] powerTargets;
//    private IMUStatic imu;
    protected double imuOffset;


    public Drivetrain(){
        powerTargets = new double[3];
    }

    public void init(Pose2d startPose) {
        currentPos = startPose;
//        imu = new IMUStatic();
        drive = new SampleMecanumDrive(Setup.hardwareMap);
        drive.setPoseEstimate(startPose);

    }

    public void update() {
        currentPos = drive.getPoseEstimate();
        if (state == DriveState.TRAJECTORY_RR) {
            drive.update();
        } else if (state == DriveState.POWER) { // spin may be (+) instead of (-)
            double x = powerTargets[0];
            double y = powerTargets[1];
            double spin = powerTargets[2];
            drive.setMotorPowers(Range.clip(y + x, -1, 1) + spin,
                    Range.clip(y - x, -1, 1) + spin,
                    Range.clip(y + x, -1, 1) - spin,
                    Range.clip(y - x, -1, 1) - spin);
        }
    }

    public void telemetry(){
        telemetry.addData("Drivetrain currentPos", currentPos);
        telemetry.addData("Drivetrain state", state);
    }

    public void setTeleOpTargets(double x, double y, double theta){
        state = DriveState.POWER;
        this.powerTargets = new double[]{x, y, theta};
    }


    public double[] CartesianToPolar(double x, double y) {
        return new double[]{Math.sqrt(x * x + y * y), Math.atan2(y, x)};
    }
//    private double[] PolarToCartesian(double r, double theta){
//        return new double[]{};
//    }


//    public double getHeadingIMU() {return imu.getYaw(AngleUnit.RADIANS);}
//    public void resetIMU(){imu.resetYaw();}

    public double getHeadingIMU() {
        return drive.getRawExternalHeading();
    }
    public void resetIMU(){
        drive.resetIMU();
    }


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

    public TrajectorySequence backward(double amount){
        return drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .back(24)
                .build();
    }

    public TrajectorySequence leftward(double amount){
        return drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .strafeLeft(24)
                .build();
    }

}
