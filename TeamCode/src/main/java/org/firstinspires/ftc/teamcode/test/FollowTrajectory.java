package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.Trajectory;

@TeleOp(group="test")
public class FollowTrajectory extends LinearOpMode {
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();
        drive.followTrajectory(this,
                new Trajectory(new Pose2d(0,0,0,0.3), true)
                        .addLine(new Pose2d(30, -30, Math.toRadians(45), Math.toRadians(0), 20, 0.5))
                        .addLine(new Pose2d(30, 30, Math.toRadians(0), Math.toRadians(0), 20, 0.5))
                        .addLine(new Pose2d(-30, 30, Math.toRadians(0), Math.toRadians(0), 20, 0.5))
                        .addLine(new Pose2d(-30, -30, Math.toRadians(0), Math.toRadians(0), 20, 0.5))
                        .addLine(new Pose2d(0, 0, Math.toRadians(0), Math.toRadians(0), 20, 0.5))
                        .end()
        );
    }
}
