package org.firstinspires.ftc.teamcode.Bot.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Bot.Bot;
import org.firstinspires.ftc.teamcode.Bot.Drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.Bot.Sensors.IMUStatic;
import org.firstinspires.ftc.teamcode.Bot.Setup;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

@Autonomous
public class AutoTesting extends LinearOpMode {
    Drivetrain drivetrain;
    Setup setup;
    Bot bot;
    IMUStatic imu;

    @Override
    public void runOpMode(){
        setup = new Setup(hardwareMap, telemetry, true, this, Setup.OpModeType.TELEOP, Setup.Team.Q1);
        bot = new Bot(Setup.mechStates, Setup.sensorStates);
        imu = new IMUStatic();
        Follower follower = new Follower(hardwareMap);
        waitForStart();

//        follower.followPath(drivetrain.BLInitToScoreClip());
        while(opModeIsActive()) {
            if(gamepad1.right_bumper) follower.update();
        }
    }
}
