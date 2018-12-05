package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.util.Locale;

import static org.firstinspires.ftc.teamcode.Rover_Hardware.CLOSEGATE;
import static org.firstinspires.ftc.teamcode.Rover_Hardware.DEPOSITDOWN;
import static org.firstinspires.ftc.teamcode.Rover_Hardware.DEPOSITUP;
import static org.firstinspires.ftc.teamcode.Rover_Hardware.MARKERUP;
import static org.firstinspires.ftc.teamcode.Rover_Hardware.WRISTDOWN;
import static org.firstinspires.ftc.teamcode.Rover_Hardware.WRISTUP;

@Autonomous(name="Rover_autoDepot3", group="5501")
//@Disabled
public class Rover_autoDepot extends LinearOpMode {

    Rover_Hardware robot = new Rover_Hardware();

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // Detector object
    private GoldAlignDetector detector;

    //Declare Sensors
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    //ModernRoboticsI2cRangeSensor frontrange;
    //ModernRoboticsI2cRangeSensor backrange;

    //USER GENERATED VALUES//
    int zAccumulated;  //Total rotation left/right
    double headingResetValue;
    int detv;
    int heading;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //CODE FOR INITIALIZING AND SETTING UP DOGECV
        telemetry.addData("Status", "DogeCV 2018.0 - Gold Align Example");

        // Set up detector
        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 1, false); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 2000; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        detector.enable(); // Start the detector!


        //CODE FOR SETTING UP AND INITIALIZING IMU
        BNO055IMU.Parameters parameters2 = new BNO055IMU.Parameters();
        parameters2.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters2.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters2.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters2.loggingEnabled = true;
        parameters2.loggingTag = "IMU";
        parameters2.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        robot.front_left.setDirection(DcMotor.Direction.REVERSE);
        robot.back_left.setDirection(DcMotor.Direction.REVERSE);
        robot.front_right.setDirection(DcMotor.Direction.FORWARD);
        robot.back_right.setDirection(DcMotor.Direction.FORWARD);

        //Reset Encoders
        robot.front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        //Set the Run Mode For The Motors
        robot.back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  //Controls the speed of the motors to be consistent even at different battery levels
        robot.front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //DEFINE SENSORS
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters2);

        //frontrange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "frontrange");
        //backrange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "backrange");

        //Setup The Telemetry Dashboard
        composeTelemetry();

        //Initilization
        int opState = 0;
        int mineralState = 0;

        //Dc Motors


        robot.lift.setPower(0);
        robot.front_left.setPower(0);
        robot.front_right.setPower(0);
        robot.back_left.setPower(0);
        robot.back_right.setPower(0);


        robot.depositlift.setPower(0);
        robot.collectarm.setPower(0);

        //Servos

        //robot.marker_deposit.setPosition(MARKERUP);
        //robot.deposit.setPosition(DEPOSITDOWN);
        //robot.collectwrist.setPosition(WRISTDOWN);
        robot.collection.setPower(0);


        // Wait for the game to start (driver presses PLAY)
        this.headingResetValue = this.getAbsoluteHeading();
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            telemetry.addData("IsAligned", detector.getAligned()); // Is the bot aligned with the gold mineral?
            telemetry.addData("X Pos", detector.getXPosition()); // Gold X position.

            telemetry.update();
            final double heading = this.getRelativeHeading();

            if (opState == 0 && opModeIsActive()) {



                /*
                sleep(200);
                robot.lift.setPower(.4);
                sleep(1500);
                robot.lift.setPower(0);
                sleep(500);
                driveStraight(200,.2);
                sleep(500);
                robot.lift.setPower(-.4);
                sleep(1500);
                robot.lift.setPower(0);
*/

                sleep(500);

                //LOCATE THE GOLD MINERAL AND STORE THAT VALUE
                if (detector.getXPosition() > 250) {
                    detv = 'c';
                    telemetry.addData("Gold Block:", "Center");
                    telemetry.update();
                } else if (detector.getXPosition() < 250) {
                    detv = 'l';
                    telemetry.addData("Gold Block:", "left");
                    telemetry.update();
                } else if (detector.getAligned() == false) {
                    detv = 'r';
                    telemetry.addData("Gold Block:", "Right");
                    telemetry.update();
                }

                driveStraight(80, .2);


                sleep(500);
                detector.disable();
                opState++;

            } else if (opState == 1 && mineralState < 3 && opModeIsActive()) {
                if (detv == 'c') {
                    if (mineralState == 0) {
                        boolean DoneTurn = this.gyroCorrect(90, 1, heading, .8, .1) > 10;
                        if (DoneTurn) mineralState++;
                        mineralState++;
                    }

                    if (mineralState == 1){
                        sleep(300);
                        //sleep(5000);
                        //stop();
                        //boolean DoneTurn = this.gyroCorrect(90, 2, heading, .05, .1) > 10;
                        //if (DoneTurn)mineralState++;
                        mineralState++;
                        }

                        if(mineralState == 2){

                        driveStraight(2600, .7);
                        sleep(300);
                        drivebackwards(300, .5);
                        sleep(300);
                        mineralState++;
                    }
                    if (mineralState == 4
                            ) {
                        boolean DoneTurn = this.gyroCorrect(133.0, 1., heading, 0.1, 0.2) > 10;
                        if (DoneTurn) mineralState++;
                    }
                    if (mineralState == 5) {
                        sleep(1000);
                        driveStraight(3900, .7);
                        stop();
                    }
                }
                if (detv == 'l') {
                    if (mineralState == 0) {
                        boolean DoneTurn = this.gyroCorrect(25.0, 2., heading, 0.1, 0.2) > 10;
                        if (DoneTurn) mineralState++;
                    }
                    if (mineralState == 1) {
                        sleep(500);
                        driveStraight(2250, .5);
                        sleep(500);
                        mineralState++;
                    }
                    if (mineralState == 2) {
                        boolean DoneTurn1 = this.gyroCorrect(135.0, 2., heading, 0.1, 0.2) > 10;
                        if (DoneTurn1) mineralState++;
                    }
                    if (mineralState == 3) {
                        sleep(500);
                        drivebackwards(-1500, .5);
                        sleep(500);
                        mineralState++;
                    }

                }
                if (mineralState == 4) {
                    sleep(500);
                    driveStraight(4500, .5);
                    sleep(500);
                    stop();
                }
            }

            if (detv == 'r') {
                if (mineralState == 0) {
                    boolean DoneTurn = this.gyroCorrect(340.0, 1., heading, 0.1, 0.2) > 10;
                    if (DoneTurn) mineralState++;
                }
                if (mineralState == 1) {
                    sleep(500);
                    driveStraight(2000, .5);
                    sleep(500);
                    mineralState++;
                }
                if (mineralState == 2) {
                    boolean DoneTurn = this.gyroCorrect(43.0, 1., heading, 0.1, 0.2) > 10;
                    if (DoneTurn) mineralState++;
                }
                if (mineralState == 3) {
                    sleep(500);
                    driveStraight(1200, .3);
                    sleep(500);
                    mineralState++;
                }
                if (mineralState == 4) {
                    boolean DoneTurn = this.gyroCorrect(90.0, 1., heading, 0.1, 0.2) > 10;
                    if (DoneTurn) mineralState++;
                }
                if (mineralState == 5) {
                    driveStraight(1000, .5);
                    sleep(500);
                    //robot.marker_deposit.setPosition(MARKERDOWN);
                    sleep(500);
                    driveStraight(3000, .5);
                    sleep(500);
                    mineralState++;
                }
                if (mineralState == 6) {
                    robot.deposit.setPosition(DEPOSITUP);
                    robot.collectwrist.setPosition(WRISTUP);
                    robot.gate.setPosition(CLOSEGATE);
                    stop();
                }
            }
        }
    }

    //FUNCTIONS

    //DRIVE STRAIGHT USING ENCODERS
    public void driveStraight(double duration, double power) {
        double leftSpeed; //Power to feed the motors
        double rightSpeed;
        double currentdirection;

        double target = angles.firstAngle;  //Starting direction
        double startPosition = robot.back_right.getCurrentPosition();//Starting position


        while (robot.back_right.getCurrentPosition() < duration + startPosition && opModeIsActive()) {  //While we have not passed out intended distance
            currentdirection = angles.firstAngle;  //Current direction

            leftSpeed = power + (angles.firstAngle - target) / 100;  //Calculate speed for each side
            rightSpeed = power - (angles.firstAngle - target) / 100;  //See Gyro Straight video for detailed explanation

            leftSpeed = Range.clip(leftSpeed, .1, 1);
            rightSpeed = Range.clip(rightSpeed, -1, 1);

            robot.back_left.setPower(leftSpeed);
            robot.front_left.setPower(leftSpeed);
            robot.back_right.setPower(rightSpeed);
            robot.front_right.setPower(rightSpeed);

            telemetry.addData("1. Back Left", robot.back_left.getCurrentPosition());
            telemetry.addData("2. Front Left", robot.front_left.getCurrentPosition());
            telemetry.addData("3. Back Right", robot.back_right.getCurrentPosition());
            telemetry.addData("4. Front Right", robot.front_right.getCurrentPosition());
            telemetry.addData("5. Distance to go", duration + startPosition - robot.back_right.getCurrentPosition());
            telemetry.update();
        }

        robot.back_left.setPower(0);
        robot.front_left.setPower(0);
        robot.back_right.setPower(0);
        robot.front_right.setPower(0);
    }

    //DRIVE BACKWARDS USING ENCODERS
    public void drivebackwards(int duration, double power) {
        double leftSpeed; //Power to feed the motors
        double rightSpeed;
        double currentdirection;

        double target = angles.firstAngle;  //Starting direction
        double startPosition = robot.back_right.getCurrentPosition();  //Starting position

        while (robot.back_right.getCurrentPosition() > duration + startPosition && opModeIsActive()) {  //While we have not passed out intended distance
            currentdirection = angles.firstAngle;  //Current direction

            //leftSpeed = power + (zAccumulated - target) / 100;  //Calculate speed for each side
            // rightSpeed = power - (zAccumulated - target) / 100;  //See Gyro Straight video for detailed explanation
            leftSpeed = 0.2;
            rightSpeed = 0.2;

            leftSpeed = Range.clip(leftSpeed, -1, 1);
            rightSpeed = Range.clip(rightSpeed, -1, 1);

            robot.back_left.setPower(-leftSpeed);
            robot.front_left.setPower(-leftSpeed);
            robot.back_right.setPower(-rightSpeed);
            robot.front_right.setPower(-rightSpeed);

            telemetry.addData("1. Back Left", robot.back_left.getCurrentPosition());
            telemetry.addData("2. Front Left", robot.front_left.getCurrentPosition());
            telemetry.addData("3. Back Right", robot.back_right.getCurrentPosition());
            telemetry.addData("4. Front Right", robot.front_right.getCurrentPosition());
            telemetry.addData("5. Distance to go", duration + startPosition - robot.back_right.getCurrentPosition());
            telemetry.update();
        }

        robot.back_left.setPower(0);
        robot.front_left.setPower(0);
        robot.back_right.setPower(0);
        robot.front_right.setPower(0);
    }

    //TURN ABSOLUTE USING GYRO
    public void turnAbsolute(int target) {
        double direction = angles.firstAngle;  //Set variables to gyro readings
        double minspeed = .1;
        double maxspeed = .15;
        double errorDegs = Math.abs(direction - target);
        double turnSpeed = maxspeed * (errorDegs / 180) + minspeed;

        while (errorDegs > 3 && opModeIsActive()) {  //Continue while the robot direction is further than three degrees from the target
            if (direction > target) {  //if gyro is positive, we will turn right
                robot.back_left.setPower(-turnSpeed);
                robot.front_left.setPower(-turnSpeed);
                robot.back_right.setPower(turnSpeed);
                robot.front_right.setPower(turnSpeed);
            }

            if (direction < target) {  //if gyro is positive, we will turn left
                robot.back_left.setPower(turnSpeed);
                robot.front_left.setPower(turnSpeed);
                robot.back_right.setPower(-turnSpeed);
                robot.front_right.setPower(-turnSpeed);
            }

            direction = angles.firstAngle;  //Set variables to gyro readings
            errorDegs = Math.abs(direction - target);
            turnSpeed = maxspeed * (errorDegs / 180) + minspeed;
            telemetry.addData("accu", String.format("%03d", zAccumulated));
            telemetry.update();
        }


        robot.back_left.setPower(0);
        robot.front_left.setPower(0);
        robot.back_right.setPower(0);
        robot.front_right.setPower(0);
    }

    //FUNCTIONS NEEDED BY THE GYRO
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    private double getAbsoluteHeading() {
        return this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    private double getRelativeHeading() {
        return this.getAbsoluteHeading() - this.headingResetValue;
    }

    private int correctCount = 0;

    /*
     @param gyroTarget The target heading in degrees, between 0 and 360
     @param gyroRange The acceptable range off target in degrees, usually 1 or 2
     @param gyroActual The current heading in degrees, between 0 and 360
     @param minSpeed The minimum power to apply in order to turn (e.g. 0.05 when moving or 0.15 when stopped)
     @param addSpeed The maximum additional speed to apply in order to turn (proportional component), e.g. 0.3
     @return The number of times in a row the heading has been in the range
     */
    public int gyroCorrect(double gyroTarget, double gyroRange, double gyroActual, double minSpeed, double addSpeed) {
        double delta = (gyroTarget - gyroActual + 360.0) % 360.0; //the difference between target and actual mod 360
        if (delta > 180.0) delta -= 360.0; //makes delta between -180 and 180
        if (Math.abs(delta) > gyroRange) { //checks if delta is out of range
            this.correctCount = 0;
            double gyroMod = delta / 45.0; //scale from -1 to 1 if delta is less than 45 degrees
            if (Math.abs(gyroMod) > 1.0)
                gyroMod = Math.signum(gyroMod); //set gyromod to 1 or -1 if the error is more than 45 degrees
            this.turn(minSpeed * Math.signum(gyroMod) + addSpeed * gyroMod);
        } else {
            this.correctCount++;
            this.turn(0.0);
        }
        return this.correctCount;
    }

    //BASIC TURN
    public void turn(double sPower) {
        robot.front_left.setPower(-sPower);
        robot.back_left.setPower(-sPower);
        robot.front_right.setPower(+sPower);
        robot.back_right.setPower(+sPower);
    }

    //COMPOSE TELEMETRY
    public void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the n    ecessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override
                    public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                            @Override
                            public String value() {
                                return String.format(Locale.getDefault(), "%.3f",
                                        Math.sqrt(gravity.xAccel * gravity.xAccel
                                                + gravity.yAccel * gravity.yAccel
                                                + gravity.zAccel * gravity.zAccel));
                            }
                        }
                );


    }

    //Turn left using encoders
    public void TurnLeft(double duration, double power) {
        double leftSpeed; //Power to feed the motors
        double rightSpeed;
        double currentdirection;

        double target = angles.firstAngle;  //Starting direction
        double startPosition = robot.back_right.getCurrentPosition();//Starting position


        while (robot.back_right.getCurrentPosition() < duration + startPosition && opModeIsActive()) {  //While we have not passed out intended distance
            currentdirection = angles.firstAngle;  //Current direction

            //leftSpeed = power + (angles.firstAngle - target) / 100;  //Calculate speed for each side
            //rightSpeed = power - (angles.firstAngle - target) / 100;  //See Gyro Straight video for detailed explanation
            leftSpeed=0.15;
            rightSpeed=0.15;

            leftSpeed = Range.clip(leftSpeed, .1, 1);
            rightSpeed = Range.clip(rightSpeed, -1, 1);

            robot.back_left.setPower(-leftSpeed);
            robot.front_left.setPower(-leftSpeed);
            robot.back_right.setPower(rightSpeed);
            robot.front_right.setPower(rightSpeed);

            telemetry.addData("1. Back Left", robot.back_left.getCurrentPosition());
            telemetry.addData("2. Front Left", robot.front_left.getCurrentPosition());
            telemetry.addData("3. Back Right", robot.back_right.getCurrentPosition());
            telemetry.addData("4. Front Right", robot.front_right.getCurrentPosition());
            telemetry.addData("5. Distance to go", duration + startPosition - robot.back_right.getCurrentPosition());
            telemetry.update();
        }

        robot.back_left.setPower(0);
        robot.front_left.setPower(0);
        robot.back_right.setPower(0);
        robot.front_right.setPower(0);
    }

    public void turnright(double duration,double power)
    {
        //robot.front_left.setPower(1,power);
    }

    public void TurnRight(int duration, double power) {
        double leftSpeed; //Power to feed the motors
        double rightSpeed;
        double currentdirection;

        double target = angles.firstAngle;  //Starting direction
        double startPosition = robot.back_right.getCurrentPosition();  //Starting position

        while (robot.back_right.getCurrentPosition() > duration + startPosition && opModeIsActive()) {  //While we have not passed out intended distance
            currentdirection = angles.firstAngle;  //Current direction

            //leftSpeed = power + (zAccumulated - target) / 100;  //Calculate speed for each side
            //rightSpeed = power - (zAccumulated - target) / 100;  //See Gyro Straight video for detailed explanation
            leftSpeed= 0.15;
            rightSpeed= 0.15;

            leftSpeed = Range.clip(leftSpeed, -1, 1);
            rightSpeed = Range.clip(rightSpeed, -1, 1);

            robot.back_left.setPower(leftSpeed);
            robot.front_left.setPower(leftSpeed);
            robot.back_right.setPower(-rightSpeed);
            robot.front_right.setPower(-rightSpeed);

            telemetry.addData("1. Back Left", robot.back_left.getCurrentPosition());
            telemetry.addData("2. Front Left", robot.front_left.getCurrentPosition());
            telemetry.addData("3. Back Right", robot.back_right.getCurrentPosition());
            telemetry.addData("4. Front Right", robot.front_right.getCurrentPosition());
            telemetry.addData("5. Distance to go", duration + startPosition - robot.back_right.getCurrentPosition());
            telemetry.update();
        }

        robot.back_left.setPower(0);
        robot.front_left.setPower(0);
        robot.back_right.setPower(0);
        robot.front_right.setPower(0);
    }
}
    /*
    // Front Range
    public void frontRange(double distance,double power) {
        double errorDist = Math.abs(backrange.rawUltrasonic()-distance);
        double leftSpeed; //Power to feed the motors
        double rightSpeed;

        leftSpeed = power ;
        rightSpeed = power ;
        leftSpeed = Range.clip(leftSpeed, .1, 1);
        rightSpeed = Range.clip(rightSpeed, -1, 1);

        while (errorDist > 1 && opModeIsActive()) {  //Continue while the robot direction is further than three degrees from the target
            if (backrange.rawUltrasonic() > distance) {  //if gyro is positive, we will turn right
                robot.back_left.setPower(leftSpeed);
                robot.front_left.setPower(leftSpeed);
                robot.back_right.setPower(rightSpeed);
                robot.front_right.setPower(rightSpeed);
            }

            if (backrange.rawUltrasonic() < distance) {  //if gyro is positive, we will turn left
                robot.back_left.setPower(-leftSpeed);
                robot.front_left.setPower(-leftSpeed);
                robot.back_right.setPower(-rightSpeed);
                robot.front_right.setPower(-rightSpeed);
            }

            errorDist = Math.abs(backrange.rawUltrasonic()-distance);
            telemetry.addData("accu", String.format("%03d", zAccumulated));
            telemetry.update();
        }


        robot.back_left.setPower(0);
        robot.front_left.setPower(0);
        robot.back_right.setPower(0);
        robot.front_right.setPower(0);
    }
}
