package org.firstinspires.ftc.teamcode.test;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Localizer;

@TeleOp (group="test")
public class LocalizationTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Log.e("*******************","james");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();
        for(int i=0; i<4; i++) {
            drive.motors.get(i).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drive.motors.get(i).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            drive.motors.get(i).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        double [] motorPowers = new double[4];

        while(opModeIsActive()) {
            drive.update();
            double forward = -0.4 * gamepad1.left_stick_y;
            double left = 0.6 * gamepad1.left_stick_x;
            double turn = 0.35 * gamepad1.right_stick_x;

            motorPowers[0] = forward + left + turn;
            motorPowers[1] = forward - left + turn;
            motorPowers[2] = forward + left - turn;
            motorPowers[3] = forward - left - turn;

            drive.setMotorPowers(motorPowers);

            telemetry.addData("X Position", drive.localizer.x);
            telemetry.addData("Y Position", drive.localizer.y);
            telemetry.addData("Heading", drive.localizer.heading);
            telemetry.update();
        }
    }
}
