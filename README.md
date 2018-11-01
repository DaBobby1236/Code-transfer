
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="State Red 1", group="5501")
@Disabled
public class Auto_Facing_Depot extends LinearOpMode {


    //  HardwareMainBot robot = new HardwareMainBot();
    HardwareMap hwMap = null;

    int detv;

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor back_left;
    private DcMotor back_right;

    //CAMERA DETECTOR
    private GoldAlignDetector detector;

    @Override
    public void runOpMode() {
        //robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        HardwareMainBot robot = new HardwareMainBot();

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


        //  ColorSensor colorsensor;

        //  colorsensor = hardwareMap.get(ColorSensor.class, "colorsensor");

        while (opModeIsActive()){
        detect();

                            }
                        }

                        public void detect ()
                        {
                            telemetry.addData("IsAligned" , detector.getAligned()); // Is the bot aligned with the gold mineral?
                            telemetry.addData("X Pos" , detector.getXPosition()); // Gold X position.

                            detector.enable();
                            if(detector.isFound() == true){
                                detv = 1;
                            }


                            detector.disable();
                            }
                            public void middle(){}
                            public void left (){}
                            public void right () {}

                    }


