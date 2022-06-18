package org.firstinspires.ftc.teamcode.utils;

public class Encoder {
    public double ticksToInches;
    public int lastVal, currentVal;
    public double scaleFactor;
    public double x, y;

    public Encoder(Pose2d point, double scaleFactor) {
        double ticksPerRotation = 8192;
        double wheelRadius = 0.6889764;
        ticksToInches = wheelRadius * Math.PI * 2 / ticksPerRotation;

        x = point.getX();
        y = point.getY();
        currentVal = 0;
        lastVal = currentVal;
        this.scaleFactor = scaleFactor;
    }

    public void update(int currentPos) {
        lastVal = currentVal;
        currentVal = currentPos;
    }

    public double getDelta() {
        return (double) (currentVal - lastVal) * ticksToInches * scaleFactor;
    }

    public double getCurrentDistance() {
        return (double) currentVal * ticksToInches * scaleFactor;
    }
}


