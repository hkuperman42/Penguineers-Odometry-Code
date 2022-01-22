package org.firstinspires.ftc.teamcode;

/* 
 * Location objects represent the robot's position (x-coord, y-coord, and heading) on the field
 */
public class Location {
    
    //Initialize variables
    public double xPosition;
    public double yPosition;
    public double heading;
    
    //Constructor so you don't have to manually input everything later 
    public Location(double x, double y, double heading) {
        this.xPosition = x;
        this.yPosition = y;
        this.heading = heading;
    }
    
    //Getter methods (to make the code readable and fancy)
    public double getX() {
        return xPosition;
    }
    
    public double getY() {
        return yPosition;
    }
    
    public double getHeading() {
        return heading;
    }
    
    
    //Setter methods (to make the code readable and fancy)
    public void setX(double x) {
        this.xPosition = x;
    }
    
    public void setY(double y) {
        this.yPosition = y;
    }
    
    public void setHeading(double heading) {
        this.heading = heading;
    }

}
