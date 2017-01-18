
/**
 * Created by James on 2016-10-18.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static android.R.attr.left;
import static android.R.attr.right;

@TeleOp(name = "MemeMachine_xD_BallLoader", group = "Indeterminate")

//***NOTES/CURRENT TO-DO:
//  Give actual values to:
//  MAX_BRUSH_SPEED / BRUSH_SPEED / LOAD_SERVO_POSITION / INITIAL_SERVO_POSITION / SERVO_TIME

public class BallLoader {

    Hardware robot;  //contains all the hardware and stuff
    Gamepad theGamepad;
    Boolean brushToggle;
    Telemetry info;
    ElapsedTime elapsedTime;


    private static final int MAX_BRUSH_SPEED = 1;
    private static final double BRUSH_SPEED = 1.0;
    private static final double LOAD_SERVO_POSITION = 50;
    private static final double INITIAL_SERVO_POSITION = 0.0;
    private static final double SERVO_TIME = 500;

    private enum ServoState{kRising, kFalling, kResting}
    private ServoState servoAction;


    public void init(Hardware passedRobot, Gamepad passedGamepad, Telemetry passedInfo) {

        robot = passedRobot;
        theGamepad = passedGamepad;
        info = passedInfo;
        elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        elapsedTime.reset();
        robot.brushMotor.setMaxSpeed(MAX_BRUSH_SPEED);
        brushToggle = false;
//        robot.loadServo.setPosition(0.0);
        servoAction = ServoState.kResting;

    }

    public void loop() {

        if(theGamepad.x) {
            //if the x button is pressed the roller will suck the ball in -

            robot.brushMotor.setPower(BRUSH_SPEED);
        }
        else {
            robot.brushMotor.setPower(0);
        }

        if(servoAction == ServoState.kResting) {
            if(theGamepad.b) {

//                robot.loadServo.setPosition(LOAD_SERVO_POSITION);
                servoAction = ServoState.kRising;
                elapsedTime.reset();
            }
        }
        if(servoAction == ServoState.kRising) {
            if(elapsedTime.time() >= SERVO_TIME) {
//                robot.loadServo.setPosition(INITIAL_SERVO_POSITION);
                servoAction = ServoState.kFalling;
                elapsedTime.reset();
            }
        }
        if(servoAction == ServoState.kFalling) {
            if(elapsedTime.time() >= SERVO_TIME + 200) {
                //NOT SURE IF THIS IS NECESSARY, BUT WANT TO ^^^^^^^^^^^^^ BE SURE THAT IT IS FOR SURE AT POSITION 0
                servoAction = ServoState.kResting;
            }
        }
        //testing stuff; may not be necessary ALL of the time, use it to see what values are needed
        info.addData("Time", "The Time is: " + elapsedTime.time());
//        info.addData("Servo Position", "The Servo Position is: " + robot.loadServo.getPosition());
    }
}
