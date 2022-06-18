package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class Localizer {
    public Encoder[] encoders;
    public long lastTime = System.nanoTime();
    public double x = 0, y = 0, heading = 0, startingHeading = 0;

    public Pose2d currentPos = new Pose2d(0,0,0);
    public Pose2d currentVel = new Pose2d(0,0,0);
    public Pose2d relCurrentVel = new Pose2d(0,0,0);
    public Pose2d currentPowerVector = new Pose2d(0,0,0);

    public ArrayList<Pose2d> posHistory = new ArrayList<>();
    public ArrayList<Pose2d> relativeHistory = new ArrayList<>();
    public ArrayList<Double> loopTimes = new ArrayList<>();

    public Pose2d lastPos;
    public double lastIMUHeading;
    public int counter = 0;

    public BNO055IMU imu;

    public Pose2d leftSensor = new Pose2d(0,0,0);
    public Pose2d rightSensor = new Pose2d(0,0,0);

    public Localizer() {
        encoders = new Encoder[3];
        encoders[0] = new Encoder(new Pose2d(0,-4.087365470633), 1); // right
        encoders[1] = new Encoder(new Pose2d(0,5.2195710226290455), -1); // left
        encoders[2] = new Encoder(new Pose2d(2.1001917100567495,0), -1); // strafe
    }

    public void update() {
        long currentTime = System.nanoTime();
        double loopTime = (currentTime - lastTime) / (1e9);
        loopTimes.add(0, loopTime);
        lastTime = currentTime;

        double deltaRight = encoders[0].getDelta();
        double deltaLeft = encoders[1].getDelta();
        double deltaBack = encoders[2].getDelta();

        double relDeltaX = ((deltaRight * encoders[1].y) - (deltaLeft * encoders[0].y)) / (encoders[1].y - encoders[0].y);
        double deltaHeading = (deltaRight - deltaLeft) / (encoders[1].y - encoders[0].y);
        double relDeltaY = deltaBack - encoders[2].x * deltaHeading;

        heading += deltaHeading;
        relativeHistory.add(0, new Pose2d(relDeltaX, relDeltaY, deltaHeading));
        if(deltaHeading != 0) {
            double r1 = relDeltaX / deltaHeading;
            double r2 = relDeltaY / deltaHeading;
            relDeltaX = Math.sin(deltaHeading) * r1 + (1.0-Math.cos(deltaHeading)) * r2;
            relDeltaY = Math.sin(deltaHeading) * r2 + (1.0-Math.cos(deltaHeading)) * r1;
        }
        double lastHeading = heading - deltaHeading;
        x += relDeltaX*Math.cos(lastHeading) - relDeltaY*Math.sin(lastHeading);
        y += relDeltaY*Math.cos(lastHeading) + relDeltaX*Math.sin(lastHeading);

        currentPos = new Pose2d(x, y, heading);
        posHistory.add(0, currentPos);
        updateVelocity();

        if(counter==0 && ((Math.abs(relCurrentVel.getY()) > 6) || (Math.abs(relCurrentVel.getY()) / Math.max(0.0001, Math.abs(relCurrentVel.getX()))) > 1)){
            counter = 10;
            lastPos = new Pose2d(x, y, heading);
            lastIMUHeading = imu.getAngularOrientation().firstAngle;
        }

        if(counter > 0) {
            counter--;
            if(counter==0) {
                double headingError = ((heading - imu.getAngularOrientation().firstAngle) - (lastPos.heading - lastIMUHeading)) * 0.9;
                heading -= headingError;

                double deltaX = x - lastPos.x;
                double deltaY = y - lastPos.y;

                x = lastPos.x + Math.cos(-headingError) * deltaX - Math.sin(-headingError) * deltaY;
                y = lastPos.y + Math.cos(-headingError) * deltaY + Math.sin(-headingError) * deltaX;
            }
        }
    }

    public void updateVelocity() {
        double targetVelTimeEstimate = 0.2;
        double actualVelTime = 0;
        double relDeltaXTotal = 0;
        double relDeltaYTotal = 0;
        double totalTime = 0;
        int lastIndex = 0;

        for(int i=0; i<loopTimes.size(); i++) {
            totalTime += loopTimes.get(i);
            if(totalTime <= targetVelTimeEstimate) {
                actualVelTime += loopTimes.get(i);
                relDeltaXTotal += relativeHistory.get(i).getX();
                relDeltaYTotal += relativeHistory.get(i).getY();
                lastIndex = i;
            }
        }

        currentVel = new Pose2d((posHistory.get(0).getX() - posHistory.get(lastIndex).getX())/actualVelTime,
                (posHistory.get(0).getY() - posHistory.get(lastIndex).getY())/actualVelTime,
                (posHistory.get(0).getHeading() - posHistory.get(lastIndex).getHeading())/actualVelTime
        );

        relCurrentVel = new Pose2d(relDeltaXTotal / actualVelTime,
                relDeltaYTotal / actualVelTime,
                (posHistory.get(0).getHeading() - posHistory.get(lastIndex).getHeading())/actualVelTime
        );

        while(lastIndex+1 < loopTimes.size()) {
            loopTimes.remove(loopTimes.size()-1);
            posHistory.remove(posHistory.size()-1);
            relativeHistory.remove(relativeHistory.size()-1);
        }
    }

    public void updateEncoders(int[] arr) {
        for(int i=0; i<arr.length; i++) {
            encoders[i].update(arr[i]);
        }
    }

    public void distUpdate(double rightDistance, double leftDistance) {
        double rightSensorX = x + Math.cos(heading) * 8 - Math.sin(heading) * (-6); // translate to sensor pos;
        double rightSensorY = y + Math.cos(heading) * (-6) + Math.sin(heading) * 8;
        double leftSensorX = x + Math.cos(heading) * 8 - Math.sin(heading) * 6; // translate to sensor pos;
        double leftSensorY = y + Math.cos(heading) * 6 + Math.sin(heading) * 8;

        double xErrorLeft = 0;
        double yErrorLeft = 0;

        if(((Math.abs(leftSensorY) < 64) && (Math.abs(leftSensorX) < 64)) && leftDistance > 24) { // make distance correct
            leftSensorX += Math.cos(heading) * leftDistance;
            leftSensorY += Math.sin(heading) * leftDistance;
            if((Math.abs(Math.abs(leftSensorX)-72) < 3) ^ (Math.abs(Math.abs(leftSensorY) - 72) < 3)) { // prevent pointing to corner 72 = wall to center
                if(Math.abs(Math.abs(leftSensorX) - 72) < 3) {
                    xErrorLeft = 72 * Math.signum(leftSensorX) - leftSensorX; // true center is 72 measured center is leftSensorX
                }
                else {
                    yErrorLeft = 72 * Math.signum(leftSensorY) - leftSensorY;
                }
            }
        }
        else {
            leftSensorX += Math.cos(heading) * leftDistance;
            leftSensorY += Math.sin(heading) * leftDistance;
        }

        double xErrorRight = 0;
        double yErrorRight = 0;

        if(((Math.abs(rightSensorY) < 64) && (Math.abs(rightSensorX) < 64)) && rightDistance > 24) { // make distance correct
            rightSensorX += Math.cos(heading) * rightDistance;
            rightSensorY += Math.sin(heading) * rightDistance;
            if((Math.abs(Math.abs(rightSensorX) - 72) < 3) ^ (Math.abs(Math.abs(rightSensorY) - 72) < 3)) { // prevent pointing to corner 72 = wall to center
                if(Math.abs(Math.abs(rightSensorX) - 72) < 3) {
                    xErrorRight = 72 * Math.signum(rightSensorX) - rightSensorX; // true center is 72 measured center is leftSensorX
                }
                else {
                    yErrorRight = 72 * Math.signum(rightSensorY) - rightSensorY;
                }
            }
        }
        else {
            rightSensorX += Math.cos(heading) * rightDistance;
            rightSensorY += Math.sin(heading) * rightDistance;
        }

        x += xErrorLeft * 0.005 * Math.pow(2, -(leftDistance-32)/18) + xErrorRight * 0.005 * Math.pow(2, -(rightDistance-32)/18);
        y += yErrorLeft * 0.005 * Math.pow(2, -(leftDistance-32)/18) + yErrorRight * 0.005 * Math.pow(2, -(rightDistance-32)/18);

        leftSensor = new Pose2d(leftSensorX, leftSensorY);
        rightSensor = new Pose2d(rightSensorX, rightSensorY);
    }

    public void updateFlex(double valFlex) {
        int valPressed = 333;

        if(valFlex < valPressed) {
            double sensorX = x + Math.cos(heading) * 5 - Math.sin(heading) * (-6.75 * Math.signum(y)); // translate to sensor pos;
            double sensorY = y + Math.cos(heading) * (-6.75 * Math.signum(y)) + Math.sin(heading) * 5; // signum(y) to account for which wall

            if(Math.abs(sensorX) >= 68) { // 4 inches of error sticking to wall
                x += (72 * Math.signum(sensorX) - sensorX) * 0.5; //0.5 gain
            }
            if(Math.abs(sensorY) >= 68) { // 4 inches of error sticking to wall
                y += (72 * Math.signum(sensorY) - sensorY) * 0.5; //0.5 gain
            }
        }
    }
}
