package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SampleMecanumDrive;

@TeleOp(group="test")
public class SensorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Sensor: rightIntake", drive.rightIntake.getVoltage());
            telemetry.addData("Sensor: leftIntake", drive.leftIntake.getVoltage());
            telemetry.addData("Sensor: depositSensor", drive.depositSensor.getVoltage());
            telemetry.addData("Sensor: distLeft", drive.distLeft.getVoltage());
            telemetry.addData("Sensor: distRight", drive.distRight.getVoltage());
            telemetry.addData("Sensor: magLeft", drive.magLeft.getVoltage());
            telemetry.addData("Sensor: magRight", drive.magRight.getVoltage());
            telemetry.addData("Sensor: flex", drive.flex.getVoltage());
            telemetry.update();
        }
    }
}
