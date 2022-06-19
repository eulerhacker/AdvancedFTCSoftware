package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.Trajectory;

import javax.sql.PooledConnection;

@TeleOp (group="test")
@Config
public class FollowerPIDTuner extends LinearOpMode {

    public static double power = 0.5;
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();

        while(opModeIsActive()) {
//            drive.followTrajectory(this, new Trajectory(new Pose2d(0, 0, 0, 0.1), true)
//                    .addLine(new Pose2d(0,0,Math.toRadians(90),0,0,0.1)).end());
//            long start = System.currentTimeMillis();
//            while (System.currentTimeMillis() - start < 1000){
//                drive.update();
//            }
//            drive.followTrajectory(this, new Trajectory(new Pose2d(0, 0, 0, 0.1), true)
//                    .addLine(new Pose2d(0,0,Math.toRadians(-90),0,0,0.1)).end());
//            start = System.currentTimeMillis();
//            while (System.currentTimeMillis() - start < 1000){
//                drive.update();
//            }

            drive.update();
            drive.followTrajectory(this,
                    new Trajectory(new Pose2d(24,-24, Math.toRadians(0), power), false)
                            .addLine(new Pose2d(24, 24, Math.toRadians(0), Math.toRadians(0), 15, power))
                            .addLine(new Pose2d(-24, 24, Math.toRadians(0), Math.toRadians(0), 15, power))
                            .addLine(new Pose2d(-24, -24, Math.toRadians(0), Math.toRadians(0), 15, power))
            );
        }
    }
}