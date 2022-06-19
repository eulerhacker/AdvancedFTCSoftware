package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.utils.Localizer;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.Trajectory;
import org.firstinspires.ftc.teamcode.utils.UpdatePriority;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class SampleMecanumDrive {
    FtcDashboard dashboard;

    RevBulkData bulkData;
    ExpansionHubEx expansionHub1, expansionHub2;
    public ExpansionHubMotor leftFront, leftBack, rightFront, rightBack;
    public ExpansionHubMotor intake, slides, slides2, turret;
    public CRServo duckSpin, duckSpin2;

    public List<ExpansionHubMotor> motors;
    public List<Servo> servos;
    public AnalogInput leftIntake, rightIntake, depositSensor, distLeft, distRight, magLeft, magRight, flex;

    public VoltageSensor batteryInfo;
    public BNO055IMU imu;
    public Localizer localizer = new Localizer();

    public int[] encoders = new int[3];
    public int loops = 0;
    public long start;

    public ArrayList<UpdatePriority> motorPriorities = new ArrayList<>();

    public boolean updateHub2 = false;
    public Pose2d targetPoint = new Pose2d(0,0,0);
    public double headingError = 0;

    public SampleMecanumDrive(HardwareMap hardwareMap) {
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25); // 25ms

        initMotors(hardwareMap);
        initSensors(hardwareMap);
        initServos(hardwareMap);

        localizer.imu = imu;
    }

    void initMotors(HardwareMap hardwareMap) {
        expansionHub1 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");
        leftFront = (ExpansionHubMotor) hardwareMap.dcMotor.get("lf");
        leftBack = (ExpansionHubMotor) hardwareMap.dcMotor.get("lr");
        rightFront = (ExpansionHubMotor) hardwareMap.dcMotor.get("rf");
        rightBack = (ExpansionHubMotor) hardwareMap.dcMotor.get("rr");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);;

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        expansionHub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        intake = (ExpansionHubMotor) hardwareMap.dcMotor.get("intake");
        slides = (ExpansionHubMotor) hardwareMap.dcMotor.get("slides");
        slides2 = (ExpansionHubMotor) hardwareMap.dcMotor.get("slides2");
        turret = (ExpansionHubMotor) hardwareMap.dcMotor.get("turret");

        slides.setDirection(DcMotorSimple.Direction.REVERSE);
        slides2.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);


        motors = Arrays.asList(leftFront, leftBack, rightBack, rightFront, intake, turret, slides, slides2);

        for(int i=0; i<4; i++) {
            motorPriorities.add(new UpdatePriority(3, 5));
            // basePriority: how constantly changing    priorityScale: weighs how much it changes larger means larger changes
        }
        motorPriorities.add(new UpdatePriority(1, 2)); // intake
        motorPriorities.add(new UpdatePriority(1, 3)); // turret
        motorPriorities.add(new UpdatePriority(2, 6)); // slides
    }

    void setDriveMode(DcMotor.RunMode runMode) {
        leftFront.setMode(runMode);
        leftBack.setMode(runMode);
        rightFront.setMode(runMode);
        rightBack.setMode(runMode);
    }

    void initServos(HardwareMap hardwareMap) {
        servos = new ArrayList<>();
        servos.add(hardwareMap.servo.get("rightIntake"));
        servos.add(hardwareMap.servo.get("leftIntake"));
        servos.add(hardwareMap.servo.get("deposit"));
        servos.add(hardwareMap.servo.get("odoLift"));
        servos.add(hardwareMap.servo.get("v4bar"));
        servos.add(hardwareMap.servo.get("rightCapstone"));
        servos.add(hardwareMap.servo.get("leftCapstone"));
        servos.add(hardwareMap.servo.get("duckSpinSpin"));
        servos.add(hardwareMap.servo.get("rightOdo"));
        servos.add(hardwareMap.servo.get("leftOdo"));

        duckSpin = hardwareMap.crservo.get("duckSpin");
        duckSpin2 = hardwareMap.crservo.get("duckSpin2");
    }

    void initSensors(HardwareMap hardwareMap) {
        rightIntake = hardwareMap.analogInput.get("rightIntake");
        leftIntake = hardwareMap.analogInput.get("leftIntake");
        depositSensor = hardwareMap.analogInput.get("depositSensor");
        distLeft = hardwareMap.analogInput.get("distLeft");
        distRight = hardwareMap.analogInput.get("distRight");
        magLeft = hardwareMap.analogInput.get("magLeft");
        magRight = hardwareMap.analogInput.get("magRight");
        flex = hardwareMap.analogInput.get("flex");

        batteryInfo = hardwareMap.voltageSensor.iterator().next();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        for(LynxModule lynxModule: hardwareMap.getAll(LynxModule.class)) {
            lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO); // expansion hub chip
        }
    }

    public void setMotorPowers(double[] powers) {
        motorPriorities.get(0).setTargetPower(powers[0]); //lf
        motorPriorities.get(1).setTargetPower(powers[1]); //lb
        motorPriorities.get(2).setTargetPower(powers[2]); //rb
        motorPriorities.get(3).setTargetPower(powers[3]); //rf
    }

    public double loopTime = 0;
    public void update() {
        long loopStart = System.nanoTime();
        if(loops==0) start=System.currentTimeMillis();

        loops++;

        getEncoders();
        updateHub2();

        loopTime = (System.nanoTime() - loopStart) / (double) 1e9;
        double targetLoopLength = 0.01;
        double bestMotorUpdate = 1;

        int numMotorsUpdated = 0;
        while((bestMotorUpdate > 0) && (loopTime <= targetLoopLength)) {
            int bestIndex = 0;
            bestMotorUpdate = motorPriorities.get(0).getPriority();
            for(int i=1; i<motorPriorities.size(); i++) {
                if(motorPriorities.get(i).getPriority() > bestMotorUpdate) {
                    bestIndex = i;
                    bestMotorUpdate = motorPriorities.get(i).getPriority();
                }
            }

            if(bestMotorUpdate != 0){
                numMotorsUpdated++;
                motors.get(bestIndex).setPower(motorPriorities.get(bestIndex).power);
                if(bestIndex == motorPriorities.size() - 1) {
                    numMotorsUpdated++;
                    slides2.setPower(motorPriorities.get(bestIndex).power);
                }
                motorPriorities.get(bestIndex).update();
            }
            loopTime = (System.nanoTime() - loopStart) / (double) 1e9;
        }

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("Loops", loops);
        packet.put("Loop speed", (double) ((System.currentTimeMillis() - start)) / loops);
        packet.put("Person:", "James");
        packet.put("Heading Error", Math.toDegrees(headingError));
        packet.put("Forward V Error", errorForwardVelocity);
        packet.put("Left V Error", errorLeftVelocity);

        Canvas fieldOverlay = packet.fieldOverlay();
        fieldOverlay.strokeCircle(localizer.x, localizer.y, 9);
        fieldOverlay.strokeLine(localizer.x, localizer.y, localizer.x + 11*Math.cos(localizer.heading), localizer.y + 11*Math.sin(localizer.heading));
        fieldOverlay.strokeCircle(localizer.leftSensor.x, localizer.leftSensor.y, 2);
        fieldOverlay.strokeCircle(localizer.rightSensor.x, localizer.rightSensor.y, 2);
        fieldOverlay.setStroke("#ff0000");
        fieldOverlay.strokeCircle(targetPoint.x, targetPoint.y, 4);

        dashboard.sendTelemetryPacket(packet);

        updateHub2 = false;
    }



    double currentIntakeSpeed;
    int rightIntakeVal, leftIntakeVal;
    int depositVal;
    int flexSensorVal;
    int magValLeft, magValRight;
    double currentSlideLength, currentSlideSpeed, currentTurretAngle, distValLeft, distValRight;
    double lastDisValRight, lastDisValLeft;

    public void updateHub2() {
        if(!updateHub2) {
            updateHub2 = true;
            bulkData = expansionHub2.getBulkInputData();
            if(bulkData != null) {
                try {
                    currentSlideLength = bulkData.getMotorCurrentPosition(slides2) / 25.1372713591; // 25.137 = m gear ratio * circumference / ticksPerRev of motor
                    currentSlideSpeed = bulkData.getMotorVelocity(slides2) / 25.1372713591;
                    currentTurretAngle = bulkData.getMotorCurrentPosition(turret) / 578.3213; // ticks to radians

                    distValLeft = bulkData.getAnalogInputValue(distLeft) / 3.2; // volts to inches
                    distValRight = bulkData.getAnalogInputValue(distRight) / 3.2;

                    magValRight = bulkData.getAnalogInputValue(magRight);
                    magValLeft = bulkData.getAnalogInputValue(magLeft);

                    if(lastDisValLeft != lastDisValRight || distValLeft != distValRight) {
                        localizer.distUpdate(distValRight, distValLeft);
                    }
                    lastDisValLeft = distValLeft;
                    lastDisValRight = distValRight;
                }
                catch(Exception e) {
                    e.printStackTrace();
                    Log.e("***** Exception ******", e.getClass().getName());
                }
            }
        }
    }

    public void getEncoders() {
        bulkData = expansionHub1.getBulkInputData();
        if(bulkData != null) {
            try {
                encoders[0] = bulkData.getMotorCurrentPosition(rightFront);
                encoders[1] = bulkData.getMotorCurrentPosition(leftFront);
                encoders[2] = bulkData.getMotorCurrentPosition(rightBack);

                flexSensorVal = bulkData.getAnalogInputValue(flex);
                localizer.updateFlex(flexSensorVal);

                localizer.updateEncoders(encoders);
                localizer.update();
            }
            catch(Exception e) {
                e.printStackTrace();
                Log.e("***** Exception ******", e.getClass().getName());
            }
        }
    }

    public void driveToPoint(LinearOpMode opMode, Pose2d targetPoint) {
        this.targetPoint = targetPoint;
        double error = Math.sqrt(Math.pow(localizer.x - targetPoint.x, 2) + Math.pow(localizer.y - targetPoint.y, 2));

        while(opMode.opModeIsActive() && error > 3) {
            error = Math.sqrt(Math.pow(localizer.x - targetPoint.x, 2) + Math.pow(localizer.y - targetPoint.y, 2));
            update();
            double targetAngle = Math.atan2(targetPoint.y - localizer.y, targetPoint.x - localizer.x);
            double headingError = targetAngle - localizer.heading;

            while(headingError > Math.PI) { // restrict to 180 -180
                headingError -= 2 * Math.PI;
            }
            while(headingError < -Math.PI) {
                headingError += 2 * Math.PI;
            }

            double relErrorX = Math.cos(headingError) * error;
            double relErrorY = Math.sin(headingError) * error;

            double turn = Math.toDegrees(headingError) * 0.3 / 15; // 15deg / s
            double forward = 0;
            double left = 0;

            if(error != 0){
                forward = (relErrorX / error) * 0.5 / (1.0-Math.abs(turn));
                left = (relErrorY / error) * 0.5 / (1.0-Math.abs(turn));
            }

            double [] mp = new double[4];
            mp[0] = forward - left - turn;
            mp[1] = forward + left - turn;
            mp[2] = forward - left + turn;
            mp[3] = forward + left + turn;

            setMotorPowers(mp);
        }
    }

    public static double headingP=3;
    public static double headingI=0.03;
    public static double headingD=0.05;
    public double headingIntegral=0;
    public double lastHeading=0;
    public double lastTargetHeading = 0;

    public static double forwardP=0.03;
    public static double forwardI=0.004;
    public static double forwardD=0.005;
    public double forwardIntegral=0;
    public double lastForward=0;
    public double lastTargetForwardVelocity = 0;

    public static double leftP=0.04;
    public static double leftI=0.004;
    public static double leftD=0.04;
    public double leftIntegral=0;
    public double lastLeft=0;
    public double lastTargetLeftVelocity = 0;


    public double errorForwardVelocity;
    public double errorLeftVelocity;
    public void followTrajectory(LinearOpMode opMode, Trajectory trajectory) {
        Pose2d targetPoint;

        while(opMode.opModeIsActive() && trajectory.points.size() != 0) {
            update();

            targetPoint = trajectory.points.get(0);
            this.targetPoint = targetPoint;

            double error = Math.sqrt(Math.pow(localizer.x - targetPoint.x, 2) + Math.pow(localizer.y - targetPoint.y, 2));
            double lastError = Math.sqrt(Math.pow(localizer.x - trajectory.points.get(trajectory.points.size()-1).x, 2) + Math.pow(localizer.y - trajectory.points.get(trajectory.points.size()-1).y, 2));

            double targetAngle = Math.atan2(targetPoint.y - localizer.y, targetPoint.x - localizer.x);
            headingError = targetAngle - localizer.heading;

            double relErrorX = Math.cos(headingError) * error;
            double relErrorY = Math.sin(headingError) * error;

            if(lastError < 8 && trajectory.points.size() < 100) {
                targetAngle = trajectory.points.get(trajectory.points.size()-1).heading;
            }
            double hDiff = targetAngle - lastTargetHeading;
            while(Math.abs(hDiff) > Math.PI) {
                hDiff -= Math.signum(hDiff) * 2 * Math.PI;
            }
            double expectedAngle = Math.signum(hDiff) * Math.toRadians(90) * loopTime + lastTargetHeading;
            lastTargetHeading = expectedAngle;
            if(Math.abs(targetAngle - expectedAngle) < Math.toRadians(5)) {
                expectedAngle = targetAngle;
            }
            headingError = expectedAngle - localizer.heading;

            while(headingError > Math.PI) { // restrict to 180 -180
                headingError -= 2 * Math.PI;
            }
            while(headingError < -Math.PI) {
                headingError += 2 * Math.PI;
            }

            headingIntegral += headingError * loopTime;
            double dHeadingError = (headingError - lastHeading) / loopTime;
            lastHeading = headingError;


            double turn = headingError * headingP + headingIntegral * headingI + dHeadingError * headingD;

//            double turn = Math.min(Math.abs(headingError * 4.47 * targetPoint.speed), 0.6 * targetPoint.speed) * Math.signum(headingError); // Math.min(Math.abs(headingError * sensitivity), maxTurn)

            double targetForwardVelocity = 0;
            double targetLeftVelocity = 0;

            if(error != 0) {
                double expectedTargetForwardVelocity = (relErrorX / error) * targetPoint.speed * (1.0 - Math.abs(turn)); // 0.5 max power forward accounting for turn
                targetForwardVelocity = Math.signum(expectedTargetForwardVelocity - lastTargetForwardVelocity) * 20 * loopTime + lastTargetForwardVelocity;
                if(Math.abs(targetForwardVelocity) - expectedTargetForwardVelocity < 2) {
                    targetForwardVelocity = expectedTargetForwardVelocity;
                }
                lastTargetForwardVelocity = targetForwardVelocity;

                double expectedTargetLeftVelocity = (relErrorY / error) * targetPoint.speed * (1.0 - Math.abs(turn));
                targetLeftVelocity = Math.signum(expectedTargetLeftVelocity - lastTargetLeftVelocity) * 10 * loopTime + lastTargetLeftVelocity;
                if(Math.abs(targetLeftVelocity) - expectedTargetLeftVelocity < 2) {
                    targetLeftVelocity = expectedTargetLeftVelocity;
                }
                lastTargetLeftVelocity = targetLeftVelocity;
            }


            targetForwardVelocity = targetForwardVelocity * 54 * 0.9; // motor power to in/s
            targetLeftVelocity = targetLeftVelocity * 40 * 0.9;

            errorForwardVelocity =  targetForwardVelocity - localizer.relCurrentVel.x;
            forwardIntegral += errorForwardVelocity * loopTime;
            double dForwardError = (errorForwardVelocity - lastForward) / loopTime;
            lastForward = errorForwardVelocity;

            double errorLeftVelocity = targetLeftVelocity - localizer.relCurrentVel.y;
            leftIntegral += errorLeftVelocity * loopTime;
            double dLeftError = (errorLeftVelocity - lastLeft) / loopTime;
            lastLeft = errorLeftVelocity;

            double forward = errorForwardVelocity * forwardP + forwardIntegral * forwardI + dForwardError * forwardD;
            double left = errorLeftVelocity * leftP + leftIntegral * leftI + dLeftError * leftD;

            double [] mp = new double[4];
            mp[0] = forward - left - turn;
            mp[1] = forward + left - turn;
            mp[2] = forward - left + turn;
            mp[3] = forward + left + turn;

            setMotorPowers(mp);
            trajectory.update(localizer.currentPos, localizer.relCurrentVel);
        }
        setMotorPowers(new double[]{0,0,0,0});
    }
}
