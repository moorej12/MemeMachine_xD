
/**
 * Created by James on 2016-10-18.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 *
 */

@TeleOp(name = "MemeMachine_xD_Shooter", group = "Indeterminate")
public class Shooter {

    public static final double LAUNCH_SERVO_EXTENDED = 0.85;  //needs value
    public static final double LAUNCH_SERVO_RETRACTED = 0.45;  //needs value
    public static final double LEFT_LIFT_SERVO_EXTENDED = 0; //needs value
    public static final double LEFT_LIFT_SERVO_RETRACTED = 1.0; //needs value
    public static final double RIGHT_LIFT_SERVO_EXTENDED = 1.0; //needs value
    public static final double RIGHT_LIFT_SERVO_RETRACTED = 0; //needs value
    public static final int LIFT_MOTOR_UP = 0; //needs value
    public static final int LIFT_MOTOR_DOWN = 720; //needs value
    public static final double LIFT_MOTOR_SPEED = 0.5; //needs value
    public static final double LAUNCH_SPEED = 9001.0; //needs value

    Hardware robot = new Hardware();
    Gamepad gamepad = new Gamepad();
    Gamepad gamepad2 = new Gamepad();
    Telemetry telemetry;

    ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    enum ShootingState {idle, spooling, shooting, shot;};
    ShootingState state;

    public void init(Hardware ahwMap, Gamepad gamepad, Gamepad gamepad2, Telemetry telemetry) {
        robot = ahwMap;
        this.gamepad = gamepad;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;

        robot.tiltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.tiltMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.leftLiftServo.setPosition(LEFT_LIFT_SERVO_RETRACTED);
        robot.rightLiftServo.setPosition(RIGHT_LIFT_SERVO_RETRACTED);

        robot.pushServo.setPosition(LAUNCH_SERVO_RETRACTED);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        state = ShootingState.idle;
    }

    public void loop() {

        telemetry.addData("Right Trigger: ", gamepad.right_trigger+"");

        if(gamepad2.right_stick_y != 0) {
            robot.tiltMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.tiltMotor.setPower(gamepad2.right_stick_y);
        } else {
            robot.tiltMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if(gamepad.y) {
            robot.leftLiftServo.setPosition(LEFT_LIFT_SERVO_EXTENDED);
            robot.rightLiftServo.setPosition(RIGHT_LIFT_SERVO_EXTENDED);
            robot.tiltMotor.setTargetPosition(LIFT_MOTOR_DOWN);
            robot.tiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.tiltMotor.setPower(Math.abs(LIFT_MOTOR_SPEED));
        }

        if(gamepad.a) {
            robot.tiltMotor.setTargetPosition(LIFT_MOTOR_UP);
            robot.tiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.tiltMotor.setPower(Math.abs(LIFT_MOTOR_SPEED));
        }

        switch(state) {
            case idle:
                //when right trigger is pressed start launch motors
                if(gamepad.right_trigger != 0) {
                    telemetry.addData("Right Trigger: ", "Being pressed");
                    state = ShootingState.spooling;
                    elapsedTime.reset();
                }
                break;

            case spooling:
                /**
                 * if time is greater than one second then motor has
                 * spooled up and its time to shoot
                 */
                if(elapsedTime.time() < 1500 ) { //needs value
                    robot.leftSpinMotor.setPower(LAUNCH_SPEED);
                    robot.rightSpinMotor.setPower(LAUNCH_SPEED);
                } else if(elapsedTime.time() > 1000) { //needs value
                    state = ShootingState.shooting;
                }
                break;

            case shooting:
                    //extend servo to push ball into brush wheels
                    robot.pushServo.setPosition(LAUNCH_SERVO_EXTENDED);
                    //after 0.2 more seconds then ball has been shot
                    if(elapsedTime.time() > 2500) {
                        state = ShootingState.shot;
                    }
                break;

            case shot:
                    //stop shooting
                    robot.leftSpinMotor.setPower(0);
                    robot.rightSpinMotor.setPower(0);
                    robot.pushServo.setPosition(LAUNCH_SERVO_RETRACTED);
                    state = ShootingState.idle;
                break;
        }
    }
}