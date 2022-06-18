package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.Button;

@TeleOp(group="test")
public class ServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();
        double [] servoPositions = new double[drive.servos.size()];

        int i = 0;

        Button y = new Button();
        Button a = new Button();

        Button leftBumper = new Button();
        Button rightBumper = new Button();

        while (opModeIsActive()){
            if(leftBumper.isClicked(gamepad1.left_bumper)) {
                i = (i+1) % (drive.servos.size());
            }
            if(rightBumper.isClicked(gamepad1.right_bumper)) {
                i = (i+drive.servos.size()-1) % (drive.servos.size());
            }

            if(y.isClicked(gamepad1.y)) {
                servoPositions[i] = Math.max(0., Math.min(servoPositions[i] + 0.01, 1.));
                drive.servos.get(i).setPosition(servoPositions[i]);
            }
            if(a.isClicked(gamepad1.a)) {
                servoPositions[i] = Math.max(0., Math.min(servoPositions[i] - 0.01, 1.));
                drive.servos.get(i).setPosition(servoPositions[i]);
            }

            if(gamepad1.x) {
                servoPositions[i] = Math.max(0., Math.min(servoPositions[i] + 0.1, 1.));
                drive.servos.get(i).setPosition(servoPositions[i]);
            }

            if(gamepad1.b) {
                servoPositions[i] = Math.max(0., Math.min(servoPositions[i] - 0.1, 1.));
                drive.servos.get(i).setPosition(servoPositions[i]);
            }

            switch(i){
                case(0):
                    telemetry.addData("Servo: rightIntake", i);
                    telemetry.addData("Servo Position", servoPositions[i]);
                    break;
                case(1):
                    telemetry.addData("Servo: leftIntake", i);
                    telemetry.addData("Servo Position", servoPositions[i]);
                    break;
                case(2):
                    telemetry.addData("Servo: deposit", i);
                    telemetry.addData("Servo Position", servoPositions[i]);
                    break;
                case(3):
                    telemetry.addData("Servo: odoLift", i);
                    telemetry.addData("Servo Position", servoPositions[i]);
                    break;
                case(4):
                    telemetry.addData("Servo: v4bar", i);
                    telemetry.addData("Servo Position", servoPositions[i]);
                    break;
                case(5):
                    telemetry.addData("Servo: rightCapstone", i);
                    telemetry.addData("Servo Position", servoPositions[i]);
                    break;
                case(6):
                    telemetry.addData("Servo: leftCapstone", i);
                    telemetry.addData("Servo Position", servoPositions[i]);
                    break;
                case(7):
                    telemetry.addData("Servo: duckSpinSpin", i);
                    telemetry.addData("Servo Position", servoPositions[i]);
                    break;
                case(8):
                    telemetry.addData("Servo: rightOdo", i);
                    telemetry.addData("Servo Position", servoPositions[i]);
                    break;
                case(9):
                    telemetry.addData("Servo: leftOdo", i);
                    telemetry.addData("Servo Position", servoPositions[i]);
                    break;
            }
            telemetry.update();
        }
    }
}
