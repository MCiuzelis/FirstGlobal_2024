package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.geometry.Vector2d;

public class Pose {
    private Vector2d vector;
    private double turn;

    public Pose(Vector2d vector, double turn){
        this.vector = vector;
        this.turn = turn;
    }

    public Pose(double x, double y, double turn){
        this(new Vector2d(x, y), turn);
    }

    public double getX(){
        return vector.getX();
    }

    public double getY(){
        return vector.getY();
    }

    public double getTurn(){
        return turn;
    }

    public Vector2d getVector(){
        return vector;
    }

    public void rotateVector(double degrees){
        vector.rotateBy(degrees);
    }
}
