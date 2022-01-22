package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/* 
 * This class uses three-dead-wheel odometry to determine the robot's position
 * on the field. Much of the math is adapted from this paper on using arcs and odometry 
 * for localization: https://drive.google.com/file/d/11c3Z9EkDj2_GuOQSFzpVzba_HPKqD6ej/view
 */
public class PositionManager {
    
    //Create constants 
    public double ticksToInches = 0;
    public double strafeCorrection = 0;
    public double offsetRL = 1;
    public double offsetStrafe = 1;
    
    //Create objects
    Location robotLocation = new Location(0.0, 0.0, 0.0);
    DcMotor right;
    DcMotor left;
    DcMotor strafe;
    
    //Create variables to hold encoder values
    public int currentRightEncoder;
    public int currentLeftEncoder;
    public int currentStrafeEncoder;
    public int previousRightEncoder;
    public int previousLeftEncoder;
    public int previousStrafeEncoder;

    //Create other necessary variables
    public double deltaRightDistance;
    public double deltaLeftDistance;
    public double deltaStrafeDistance;
    public double deltaAngle;
    public double deltaX;
    public double deltaY;
    public double forwardBackwardRadius;
    public double strafeRadius;
    public double startHeading;
    public double lastHeading;

    //This method should be run during the init phase of an opmode
    public void setup() {
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafe.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    
    
    //This method should be ran immediatly before we begin tracking the robot's position
    public void initLocalization() {
        previousRightEncoder = right.getCurrentPosition();
        previousLeftEncoder = left.getCurrentPosition();
        previousStrafeEncoder = strafe.getCurrentPosition();
        
        startHeading = previousRightEncoder - previousLeftEncoder;
    }
    
    
    //Updates the location of the robot and returns it
    public Location updateLocation() {
    
        //Update all of the encoder values
        currentRightEncoder = right.getCurrentPosition();
        currentLeftEncoder = left.getCurrentPosition();
        currentStrafeEncoder = strafe.getCurrentPosition();
        
        //Calculate the distanced traveled by each wheel
        deltaRightDistance = (currentRightEncoder - previousRightEncoder) * ticksToInches;
        deltaLeftDistance = (currentLeftEncoder - previousLeftEncoder) * ticksToInches;
        deltaStrafeDistance = (currentStrafeEncoder - previousStrafeEncoder) * ticksToInches;
        
        //Update all of the previous encoder variables using the readings made at the beginning of the method
        previousRightEncoder = currentRightEncoder;
        previousLeftEncoder = currentLeftEncoder;
        previousStrafeEncoder = currentStrafeEncoder;
        
        //Calculate the angle that the robot traveled through during this frame
        deltaAngle = (deltaRightDistance - deltaLeftDistance) / (2 * offsetRL);
        
        //TODO:
        //CORRECT THE STRAFING VALUE BASED ON THE ANGLE
        //Swtch Radius and final values
        
        //If the robot did not travel through an angle (i.e. it moved straight), apply the appropriate math
        if (deltaAngle == 0) {
            deltaX = deltaStrafeDistance;
            deltaY = deltaRightDistance;
          
        //Otherwise, apply the (somewhat more complicated) math to calculate and combine the two arcs  
        } else {
            //Calculate the radii of the arcs
            //CHANGE - Absolute Value OR change the radius value...
            forwardBackwardRadius = offsetRL * (deltaLeftDistance + deltaRightDistance) / (deltaRightDistance - deltaLeftDistance);
            strafeRadius = deltaStrafeDistance / deltaAngle - offsetStrafe;
            
            //Calculate the change in the X and Y coordinates
            deltaX = forwardBackwardRadius * (Math.cos(deltaAngle) - 1) + strafeRadius * Math.sin(deltaAngle);
            deltaY = forwardBackwardRadius * Math.sin(deltaAngle) + strafeRadius * (1 - Math.cos(deltaAngle));
        }
        
        //Store the heading value from the previous frame
        lastHeading = robotLocation.getHeading();
        
        //Calculate the current heading of the robot
        robotLocation.setHeading(normalizeAngle(ticksToInches * (currentRightEncoder - currentLeftEncoder) / (2 * offsetRL) + startHeading));
        
        //Update the position of the robot (rotated from the robot's heading to the orientation of the field)
        robotLocation.setX(robotLocation.getX() + deltaX * Math.cos(lastHeading) - deltaY * Math.sin(lastHeading));
        robotLocation.setY(robotLocation.getY() + deltaY * Math.cos(lastHeading) + deltaX * Math.sin(lastHeading));
        
        //Return the robot's current location
        return robotLocation;
    }
    
    
    //Normalizes an angle in radians
    public double normalizeAngle(double angle) {
        //If the angle is greater than 2pi, find the next-lower coterminal angle 
        while (angle >= 2 * Math.PI) {
            angle -= 2 * Math.PI;
        }
        
        //If the angle is less than 2pi, find the next-greater coterminal angle 
        while (angle < 0) {
            angle += 2 * Math.PI;
        }
        
        //Return the new angle
        return angle;
    }

    
    //Returns the robot's current location
    public Location getCurrentLocation() {
        return robotLocation;
    }
}
