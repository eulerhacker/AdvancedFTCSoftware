package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.Trajectory;

import javax.sql.PooledConnection;

@TeleOp (group="test")
public class HeadingPIDTuner extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();

        while(opModeIsActive()) {
            drive.followTrajectory(this, new Trajectory(new Pose2d(0, 0, 0, 0.75), true)
                    .addLine(new Pose2d(0,0,Math.toRadians(90),0,0,0.75));
        }
    }
}
