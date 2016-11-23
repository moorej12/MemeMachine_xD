package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class Hardware
{
    /* Public OpMode members. */
    public DcMotor leftRearMotor = null;
    public DcMotor rightRearMotor = null;
    public DcMotor rightFrontMotor = null;
    public DcMotor leftFrontMotor = null;
    public DcMotor brushMotor = null;
    public DcMotor liftMotor = null;
    public DcMotor leftSpinMotor  = null;
    public DcMotor rightSpinMotor  = null;
    public Servo pushServo = null;
    public TouchSensor bottomTouchButton  = null;
    public TouchSensor topTouchButton  = null;
    public RangeSensor rangeSensor = null;

    public static final double MID_SERVO       =  0.5 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Hardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
//        leftRearMotor = hwMap.dcMotor.get("left_rear_drive");
//        rightRearMotor = hwMap.dcMotor.get("right_rear_drive");
//        leftFrontMotor = hwMap.dcMotor.get("left_front_drive");
//        rightFrontMotor = hwMap.dcMotor.get("right_front_drive");
//        brushMotor = hwMap.dcMotor.get("brush_motor");
//        liftMotor = hwMap.dcMotor.get("lift_motor");
        leftSpinMotor = hwMap.dcMotor.get("left_spin_motor");
//        rightSpinMotor = hwMap.dcMotor.get("right_spin_motor");
//        rangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "range sensor");

        //define and initialize servos
        pushServo = hwMap.servo.get("push_servo");
        pushServo.setPosition(MID_SERVO);

        //define and initialize buttons
//        bottomTouchButton = hwMap.touchSensor.get("bottom_touch_button");
//        topTouchButton = hwMap.touchSensor.get("top_touch_button");

//        leftRearMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
//        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
//        leftRearMotor.setPower(0);
//        rightRearMotor.setPower(0);
//        leftFrontMotor.setPower(0);
//        rightFrontMotor.setPower(0);
//        brushMotor.setPower(0);
//        liftMotor.setPower(0);
        leftSpinMotor.setPower(0);
//        rightSpinMotor .setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
//        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        brushMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSpinMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightSpinMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

