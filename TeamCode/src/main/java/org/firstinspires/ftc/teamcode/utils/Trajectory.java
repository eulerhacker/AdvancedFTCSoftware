package org.firstinspires.ftc.teamcode.utils;

import java.util.ArrayList;

public class Trajectory {
    public ArrayList<Pose2d> points;
    public boolean slowDown;

    public Trajectory(Pose2d a, boolean slowDown) {
        points = new ArrayList<>();
        points.add(a);
        this.slowDown = slowDown;
    }
    // pure pursuit gives us changes in x y but this gives speeds to go at
    public Trajectory addLine(Pose2d end) {
        points.set(points.size()-1,  // make sure connection smooth
                new Pose2d(
                        points.get(points.size()-1).x,
                        points.get(points.size()-1).y,
                        points.get(points.size()-1).heading,
                        end.headingOffset,
                        end.radius,
                        points.get(points.size() - 1).speed
                )
        );

        Pose2d start = points.get(points.size()-1);

        double i = 0;

        ArrayList<Pose2d> newPoints = new ArrayList<>();
        double d = Math.sqrt(Math.pow(start.x - end.x, 2) + Math.pow(start.y - end.y, 2));

        while(i <= 1) {
            i += 0.01; // 100 loops
            newPoints.add(
                    new Pose2d(
                            start.x + (end.x - start.x) * i,
                            start.y + (end.y - start.y) * i,
                            start.heading + (end.heading - start.heading) * i,
                            end.headingOffset,
                            end.radius,
                            Math.min(Math.max(((d-end.radius-4)/16) * i * (end.speed-start.speed), Math.min(start.speed, end.speed)), Math.max(start.speed, end.speed)) // accelerate then maintain
                    )
            );
        }
        points.addAll(newPoints);
        Trajectory a = new Trajectory(new Pose2d(0,0), slowDown);
        a.points = points;
        return a;
    }

    public Trajectory end() {
        if(slowDown) {
            points.get(points.size() - 1).radius = 4;
            points.get(points.size() - 1).speed = 0.2;
            for(int i=2; i<points.size(); i++) {
                double d = Math.sqrt(Math.pow(points.get(points.size() - i).x - points.get(points.size()-1).x, 2) + Math.pow(points.get(points.size() - i).y - points.get(points.size()-1).y, 2));
                double speed = 0.2 + Math.max((d-8)/15, 0);
                double radius = 4 + Math.max((d-5) * 0.6666, 0);
                if(speed >= 1) {
                    Trajectory trajectory = new Trajectory(new Pose2d(0,0), slowDown);
                    trajectory.points = points;
                    return trajectory;
                }
                points.get(points.size()-i).speed = Math.min(points.get(points.size() - i).speed, speed);
                points.get(points.size()-i).radius = Math.min(points.get(points.size() - i).radius, radius);
            }
        }
        Trajectory trajectory = new Trajectory(new Pose2d(0,0), slowDown);
        trajectory.points = points;
        return trajectory;
    }

    public void update(Pose2d currentPos, Pose2d currentRelVelocity) {
        double vel = Math.sqrt(Math.pow(currentRelVelocity.x,2) + Math.pow(currentRelVelocity.y, 2));

        while((points.size() > 1) && (Math.sqrt(Math.pow(points.get(0).x-currentPos.x, 2) + Math.pow(points.get(0).y-currentPos.y,2)) <= points.get(0).radius)) {
            points.remove(0);
        }
        if(points.size()==1) {
            if(slowDown) {
                double headingError = currentPos.heading - points.get(0).heading;
                while(Math.abs(headingError) > Math.PI) {
                    headingError -= 2 * Math.PI * Math.signum(headingError);
                }
                if((vel <= 4) && (Math.sqrt(Math.pow(points.get(0).x-currentPos.x, 2) + Math.pow(points.get(0).y-currentPos.y,2)) <= 2) && Math.abs(headingError) < Math.toRadians(5)) {
                    points.remove(0);
                }
            }
            else {
                if(Math.sqrt(Math.pow(points.get(0).x-currentPos.x, 2) + Math.pow(points.get(0).y-currentPos.y,2)) <= points.get(0).radius) {
                    points.remove(0);
                }
            }
        }
    }
}
