
package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Auto Facing Depot", group="5501")
@Disabled
public class Auto_Facing_Crater_XposTests extends LinearOpMode {


    //  HardwareMainBot robot = new HardwareMainBot();
    HardwareMap hwMap = null;

    char detv;

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor back_left;
    private DcMotor back_right;

    int zAccumulated;
    ////////////
    //Sensors//
    ///////////

    //CAMERA DETECTOR
    private GoldAlignDetector detector;

    //  ColorSensor colorsensor;

    //  colorsensor = hardwareMap.get(ColorSensor.class, "colorsensor");

    ModernRoboticsI2cRangeSensor frontrange;
    ModernRoboticsI2cRangeSensor backrange;

    //IDK Yet But It Is Helpful
    Orientation angles;
    Acceleration gravity;


    @Override
    public void runOpMode() {
        //robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        HardwareMainBot robot = new HardwareMainBot();

        frontrange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "frontrange");
        backrange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "backrange");

        front_left = hardwareMap.get(DcMotor.class, "fl");
        back_left = hardwareMap.get(DcMotor.class, "bl");
        front_right = hardwareMap.get(DcMotor.class, "fr");
        back_right = hardwareMap.get(DcMotor.class, "br");

        front_left.setDirection(DcMotor.Direction.FORWARD);
        front_right.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.FORWARD);
        back_right.setDirection(DcMotor.Direction.REVERSE);

        //Reset Encoders
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        //Set the Run Mode For The Motors
        robot.back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  //Controls the speed of the motors to be consistent even at different battery levels
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.update();

        // Set up detector
        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        detector.enable(); // Start the detector!



        while (opModeIsActive()) {
            detect();

            //Knock block off

            if (detv == 'm');{
                back_right.setPower(1);          // Only for tesing, not for final program
                //driveStraight(1500,.5);
                //drivebackwards(1500,.5);
                //TurnLeft(700,.5);
                //TurnRight(1200,.5);
                //drivebackwards(2500,.5);
                //driveStraight(3500,.5);

            }
            if (detv == 'l');{
                //TurnLeft(250,.5);
                //driveStraight(1700,.5);
                back_right.setPower(0.25);   //  Only for testing, not for final program
                //drivebackwards(1700,.5);
                //TurnRight(1250,5);
                //TurnLeft(700,.5);
                //TurnRight(1200,.5);
                //drivebackwards(2500,.5);
                //driveStraight(3500,.5);

            }
            if (detv =='r');{
                //TurnRight(250,.5);
                //driveStraight(1700,.5);
                back_right.setPower(-1);    //        Only for testing, not for final program
                //drivebackwards(1700,.5);
                //TurnLeft(250,.5);
                //TurnLeft(700,.5);
                //TurnRight(1200,.5);
                //drivebackwards(2500,.5);
                //driveStraight(3500,.5);

            }
        }
    }

    public void detect() {
        telemetry.addData("IsAligned", detector.getAligned()); // Is the bot aligned with the gold mineral?
        telemetry.addData("X Pos", detector.getXPosition()); // Gold X position.

        detector.enable();
        if (detector.getXPosition()  > 150 && detector.getXPosition() < 450){
            detv = 'm';

        }

        else if (detector.getXPosition() > 450)
        {
            detv = 'l';
        }
        else if (detector.getXPosition() < 150) {

            detv = 'r';

        }



        detector.disable();
    }



    //DRIVE STRAIGHT USING ENCODERS
    public void driveStraight(double duration, double power) {
        double leftSpeed; //Power to feed the motors
        double rightSpeed;
        double currentdirection;

        double target = angles.firstAngle;  //Starting direction
        double startPosition = back_right.getCurrentPosition();//Starting position


        while (back_right.getCurrentPosition() < duration + startPosition && opModeIsActive()) {  //While we have not passed out intended distance
            currentdirection = angles.firstAngle;  //Current direction

            leftSpeed = power + (angles.firstAngle - target) / 100;  //Calculate speed for each side
            rightSpeed = power - (angles.firstAngle - target) / 100;  //See Gyro Straight video for detailed explanation

            leftSpeed = Range.clip(leftSpeed, .1, 1);
            rightSpeed = Range.clip(rightSpeed, -1, 1);

            back_left.setPower(leftSpeed);
            front_left.setPower(leftSpeed);
            back_right.setPower(rightSpeed);
            front_right.setPower(rightSpeed);

            telemetry.addData("1. Back Left",back_left.getCurrentPosition());
            telemetry.addData("2. Front Left",front_left.getCurrentPosition());
            telemetry.addData("3. Back Right",back_right.getCurrentPosition());
            telemetry.addData("4. Front Right",front_right.getCurrentPosition());
            telemetry.addData("5. Distance to go", duration + startPosition -back_right.getCurrentPosition());
            telemetry.update();
        }

        back_left.setPower(0);
        front_left.setPower(0);
        back_right.setPower(0);
        front_right.setPower(0);
    }

    //Turn Left Using Encoders

    public void TurnLeft(double duration, double power) {
        double leftSpeed; //Power to feed the motors
        double rightSpeed;
        double currentdirection;

        double target = angles.firstAngle;  //Starting direction
        double startPosition = back_right.getCurrentPosition();//Starting position


        while (back_right.getCurrentPosition() < duration + startPosition && opModeIsActive()) {  //While we have not passed out intended distance
            currentdirection = angles.firstAngle;  //Current direction

            //leftSpeed = power + (angles.firstAngle - target) / 100;  //Calculate speed for each side
            //rightSpeed = power - (angles.firstAngle - target) / 100;  //See Gyro Straight video for detailed explanation
            leftSpeed=0.15;
            rightSpeed=0.15;

            leftSpeed = Range.clip(leftSpeed, .1, 1);
            rightSpeed = Range.clip(rightSpeed, -1, 1);

            back_left.setPower(-leftSpeed);
            front_left.setPower(-leftSpeed);
            back_right.setPower(rightSpeed);
            front_right.setPower(rightSpeed);

            telemetry.addData("1. Back Left", back_left.getCurrentPosition());
            telemetry.addData("2. Front Left", front_left.getCurrentPosition());
            telemetry.addData("3. Back Right", back_right.getCurrentPosition());
            telemetry.addData("4. Front Right", front_right.getCurrentPosition());
            telemetry.addData("5. Distance to go", duration + startPosition -back_right.getCurrentPosition());
            telemetry.update();
        }

        back_left.setPower(0);
        front_left.setPower(0);
        back_right.setPower(0);
        front_right.setPower(0);
    }
    //Turn Right Using Encoders

    public void TurnRight(int duration, double power) {
        double leftSpeed; //Power to feed the motors
        double rightSpeed;
        double currentdirection;

        double target = angles.firstAngle;  //Starting direction
        double startPosition = back_right.getCurrentPosition();  //Starting position

        while (back_right.getCurrentPosition() > duration + startPosition && opModeIsActive()) {  //While we have not passed out intended distance
            currentdirection = angles.firstAngle;  //Current direction

            //leftSpeed = power + (zAccumulated - target) / 100;  //Calculate speed for each side
            //rightSpeed = power - (zAccumulated - target) / 100;  //See Gyro Straight video for detailed explanation
            leftSpeed= 0.15;
            rightSpeed= 0.15;

            leftSpeed = Range.clip(leftSpeed, -1, 1);
            rightSpeed = Range.clip(rightSpeed, -1, 1);

            back_left.setPower(leftSpeed);
            front_left.setPower(leftSpeed);
            back_right.setPower(-rightSpeed);
            front_right.setPower(-rightSpeed);

            telemetry.addData("1. Back Left", back_left.getCurrentPosition());
            telemetry.addData("2. Front Left", front_left.getCurrentPosition());
            telemetry.addData("3. Back Right", back_right.getCurrentPosition());
            telemetry.addData("4. Front Right", front_right.getCurrentPosition());
            telemetry.addData("5. Distance to go", duration + startPosition - back_right.getCurrentPosition());
            telemetry.update();
        }

        back_left.setPower(0);
        front_left.setPower(0);
        back_right.setPower(0);
        front_right.setPower(0);
    }
    //TURN ABSOLUTE
    public void turnAbsolute(int target) {
        double direction = angles.firstAngle;  //Set variables to gyro readings
        double minspeed = .1;
        double maxspeed = .15;
        double errorDegs = Math.abs(direction-target);
        double turnSpeed = maxspeed * (errorDegs/180) + minspeed;

        while (errorDegs > 3 && opModeIsActive()) {  //Continue while the robot direction is further than three degrees from the target
            if (direction > target) {  //if gyro is positive, we will turn right
                back_left.setPower(-turnSpeed);
                front_left.setPower(-turnSpeed);
                back_right.setPower(turnSpeed);
                front_right.setPower(turnSpeed);
            }

            if (direction < target) {  //if gyro is positive, we will turn left
                back_left.setPower(turnSpeed);
                front_left.setPower(turnSpeed);
                back_right.setPower(-turnSpeed);
                front_right.setPower(-turnSpeed);
            }

            direction = angles.firstAngle;  //Set variables to gyro readings
            errorDegs = Math.abs(direction-target);
            turnSpeed = maxspeed * (errorDegs/180) + minspeed;
            telemetry.addData("accu", String.format("%03d", zAccumulated));
            telemetry.update();
        }


        back_left.setPower(0);
        front_left.setPower(0);
        back_right.setPower(0);
        front_right.setPower(0);
    }

    //DRIVE BACKWARDS USING ENCODERS
    public void drivebackwards(int duration, double power) {
        double leftSpeed; //Power to feed the motors
        double rightSpeed;
        double currentdirection;

        double target = angles.firstAngle;  //Starting direction
        double startPosition = back_right.getCurrentPosition();  //Starting position

        while (back_right.getCurrentPosition() > duration + startPosition && opModeIsActive()) {  //While we have not passed out intended distance
            currentdirection = angles.firstAngle;  //Current direction

            //leftSpeed = power + (zAccumulated - target) / 100;  //Calculate speed for each side
            // rightSpeed = power - (zAccumulated - target) / 100;  //See Gyro Straight video for detailed explanation
            leftSpeed = 0.2;
            rightSpeed = 0.2;

            leftSpeed = Range.clip(leftSpeed, -1, 1);
            rightSpeed = Range.clip(rightSpeed, -1, 1);

            back_left.setPower(-leftSpeed);
            front_left.setPower(-leftSpeed);
            back_right.setPower(-rightSpeed);
            front_right.setPower(-rightSpeed);

            telemetry.addData("1. Back Left", back_left.getCurrentPosition());
            telemetry.addData("2. Front Left", front_left.getCurrentPosition());
            telemetry.addData("3. Back Right", back_right.getCurrentPosition());
            telemetry.addData("4. Front Right", front_right.getCurrentPosition());
            telemetry.addData("5. Distance to go", duration + startPosition - back_right.getCurrentPosition());
            telemetry.update();
        }

        back_left.setPower(0);
        front_left.setPower(0);
        back_right.setPower(0);
        front_right.setPower(0);
    }
}
