package org.firstinspires.ftc.teamcode.utils;

public class Pose2d {
    public double x, y;
    public double heading, headingOffset;
    public double radius;
    public double speed;

    public Pose2d(double x, double y, double heading, double headingOffset, double radius, double speed) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.headingOffset = headingOffset;
        this.radius = radius;
        this.speed = speed;
    }

    public Pose2d(double x, double y) {
        this(x, y, 0, 0, 0, 0);
    }

    public Pose2d(double x, double y, double heading) {
        this(x, y, heading, 0, 0, 0);
    }

    public Pose2d(double x, double y, double heading, double speed) {
        this(x, y, heading, 0, 0, speed);
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }
}
