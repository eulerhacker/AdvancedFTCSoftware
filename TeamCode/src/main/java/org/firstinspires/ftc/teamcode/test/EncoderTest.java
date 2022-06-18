package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.SampleMecanumDrive;

@TeleOp(group="test")
public class EncoderTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
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

            telemetry.addData("Motor: leftFront", drive.motors.get(0).getCurrentPosition());
            telemetry.addData("Motor: leftBack", drive.motors.get(1).getCurrentPosition());
            telemetry.addData("Motor: rightBack", drive.motors.get(2).getCurrentPosition());
            telemetry.addData("Motor: rightFront", drive.motors.get(3).getCurrentPosition());
            telemetry.update();
        }


    }
}
