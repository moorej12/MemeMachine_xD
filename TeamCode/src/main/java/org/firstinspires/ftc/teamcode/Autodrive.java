package org.firstinspires.ftc.teamcode;

/**
 * Created by spaet on 12/20/2016.
 */

public class Autodrive {
    Hardware robot;
    Finder finder;
    public void init(Hardware hardware, Finder finder){
        robot = hardware;
        this.finder= finder;
    }

    public void driveForwards(double speed){
        robot.leftFrontMotor.setPower(speed);
        robot.rightFrontMotor.setPower(speed);
    }

    public void driveBackwards(double speed){
        robot.leftFrontMotor.setPower(-speed);
        robot.rightFrontMotor.setPower(-speed);
    }

    public void turnLeft(double speed){
        robot.leftFrontMotor.setPower(-speed);
        robot.rightFrontMotor.setPower(speed);
    }

    public void turnRight(double speed){
        robot.leftFrontMotor.setPower(speed);
        robot.rightFrontMotor.setPower(-speed);
    }

//    public boolean aimAtTarget(){
//        double targetAngle = finder.getAngleToTarget();
//        double positionAngle = robot.gyro.getHeading();
//        if (targetAngle > positionAngle) {
//            turnRight(0.3);
//            return false;
//        }
//        if (targetAngle <  positionAngle) {
//            turnLeft(0.3);
//            return false;
//        }
//        robot.leftFrontMotor.setPower(0);
//        robot.rightFrontMotor.setPower(0);
//        return true;
//    }

}