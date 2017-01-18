
/**
 * Created by James on 2016-10-18.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static android.R.attr.left;
import static android.R.attr.right;

@TeleOp(name = "MemeMachine_xD_Lift", group = "Indeterminate")
public class Lift {


//    Telemetry telemetry;
//    Hardware robot; // name of class name of variable = new name of class
//    Gamepad controller;
//
//    public void init(Hardware ahwMap, Gamepad xboxController, Telemetry telemetry) { // gamepad controller is my parameter
//        robot = ahwMap;
//        controller = xboxController;
//        telemetry.addData("Status", "Initialized");
//        this.telemetry=telemetry;
//    }
//
//    public void loop() {
//
//        double motorspeed = 0;
//
//
//        if (controller.dpad_up && !robot.topTouchButton.isPressed()){
//            motorspeed = 0.5;
//            telemetry.addData("move up","lift is moving up");
//        }
//        if (controller.dpad_down && !robot.bottomTouchButton.isPressed()){
//            motorspeed = -0.5;
//            telemetry.addData("move down","lift is moving down");
//        }
//        robot.liftMotor.setPower(motorspeed);
//    }
}