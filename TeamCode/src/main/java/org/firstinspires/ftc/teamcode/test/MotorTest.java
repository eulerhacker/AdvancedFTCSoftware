package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SampleMecanumDrive;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.Arrays;

@TeleOp(group="test")
public class MotorTest extends LinearOpMode {
    ExpansionHubMotor leftFront, leftBack, rightFront, rightBack;
    ExpansionHubMotor intake, slides, slides2, turret;
    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();
        double [] motorPower = new double[drive.motors.size()];

        int i = 0;
        boolean lastA = false;
        while (opModeIsActive()){
            boolean a = gamepad1.a;
            motorPower[i] = -gamepad1.right_stick_y;
            drive.motors.get(i).setPower(motorPower[i]);
            switch(i){
                case(0):
                    telemetry.addData("Motor: leftFront", i);
                    telemetry.addData("MotorPower", motorPower[i]);
                    break;
                case(1):
                    telemetry.addData("Motor: leftBack", i);
                    telemetry.addData("MotorPower", motorPower[i]);
                    break;
                case(2):
                    telemetry.addData("Motor: rightBack", i);
                    telemetry.addData("MotorPower", motorPower[i]);
                    break;
                case(3):
                    telemetry.addData("Motor: rightFront", i);
                    telemetry.addData("MotorPower", motorPower[i]);
                    break;
                case(4):
                    telemetry.addData("Motor: intake", i);
                    telemetry.addData("MotorPower", motorPower[i]);
                    break;
                case(5):
                    telemetry.addData("Motor: turret", i);
                    telemetry.addData("MotorPower", motorPower[i]);
                    break;
                case(6):
                    telemetry.addData("Motor: slides", i);
                    telemetry.addData("MotorPower", motorPower[i]);
                    break;
                case(7):
                    telemetry.addData("Motor: slides2", i);
                    telemetry.addData("MotorPower", motorPower[i]);
                    break;
            }
            telemetry.update();

            if (a && !lastA) {
                i = (i+1) % 8;
            }
            lastA = a;
        }
    }
}
