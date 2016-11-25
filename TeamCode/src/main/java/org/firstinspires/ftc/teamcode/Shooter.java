
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
import org.firstinspires.ftc.robotcore.internal.TelemetryImpl;

import static android.R.attr.left;
import static android.R.attr.right;

/*
 *
 */

@TeleOp(name = "MemeMachine_xD_Shooter", group = "Indeterminate")
public class Shooter {

    public static final double SERVO_EXTENDED = 1;  //needs value
    public static final double SERVO_RETRACTED = 0;  //needs value
    public static final double LAUNCH_SPEED = 1; //needs value

    Hardware robot = new Hardware();
    Gamepad gamepad = new Gamepad();
    Telemetry telemetry;
    ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    enum ShootingState {idle, spooling, shooting, shot};
    ShootingState state;

    public void init(Hardware ahwMap, Gamepad gamepad, Telemetry telemetry) {
        robot = ahwMap;
        this.gamepad = gamepad;
        this.telemetry = telemetry;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        state = ShootingState.idle;
    }

    public void loop() {

        telemetry.addData("Right Trigger: ", gamepad.right_trigger+"");

        //when right trigger is pressed start launch motors
        switch(state) {
            case idle:
                if(gamepad.right_trigger != 0) {
                    telemetry.addData("Right Trigger: ", "Being pressed");
                    state = ShootingState.spooling;
                    elapsedTime.reset();
                }
                break;

            case spooling:
                if(elapsedTime.time() < 1000 ) { //needs value
                    robot.leftSpinMotor.setPower(LAUNCH_SPEED);
                } else if(elapsedTime.time() > 1000) { //needs value
                    state = ShootingState.shooting;
                }
                break;

            case shooting:
                    robot.pushServo.setPosition(SERVO_EXTENDED);
                    if(elapsedTime.time() > 1200) {
                        state = ShootingState.shot;
                    }
                break;

            case shot:
                    robot.leftSpinMotor.setPower(0);
                    robot.pushServo.setPosition(SERVO_RETRACTED);
                    state = ShootingState.idle;
                break;
        }
    }
}