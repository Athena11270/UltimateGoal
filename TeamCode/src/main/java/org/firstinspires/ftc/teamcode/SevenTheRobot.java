/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// test

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
//vision import
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import java.util.List;
//dont mind me just importing some imu stuff
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.nio.file.DirectoryIteratorException;

// This is not an OpMode.  It is a class that holds all the boring stuff

public class SevenTheRobot {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //Vuforia key
    private static final String VUFORIA_KEY =
            "AZ3ryeD/////AAABmeY7zPkTgkgivhkyR/Wv11tYFIVOPjl+cbNTS5QTjJcDTzTxj2tsXDiSAYKEk0bwRs5LARKJohUORPn9Jg5ti0zDqAugPqNQaHDd24+J0E85fxlpZRPw75z+XNOy+x5s6zMR5hwHKKEwQ+/CxT8vppDZ4UOi1pns8UGn2OptQbZc42qEiwcaImzVmnC1OG+XLniEgCyrIFrCXEA727Wa+cKiaTAHIjQughcOmM7E9+gg8obCjU0iDt91ibtNCSB/PRzyXmxK2GRcwTl7DRHiu7DhaC5JqTWj4Yx4nMbkU0YF5tskagd47y0xyl1uzxi/eqJW5NxmhZjUravn9K5yl+w4NwWou5j5lnf/X/Mddn5N";
    //Tensorflow reference file
    private static final String TensorFlowInferenceModel = "UltimateGoal.tflite";
    //more vuforia :)
    private VuforiaLocalizer vuforia;

    // create arrays for motors
    public DcMotor[] LeftMotors = new DcMotor[2];
    public DcMotor[] RightMotors = new DcMotor[2];
    public DcMotor[] AllMotors = new DcMotor[4];


    BNO055IMU imu;

    // Motors
    public DcMotor FL = null;
    public DcMotor FR = null;
    public DcMotor BL = null;
    public DcMotor BR = null;

    // arm and stuff
    public DcMotor Arm = null;
    public Servo Claw = null;

    // Outtake/Launchers
    /*private DcMotor LR = null;
    private DcMotor LL = null;*/
    //in case of 1 motor for launcher
    private DcMotor OT = null;

    // Intake
    private DcMotor IT = null;

    // servo for reservoir
    public Servo BP = null;

    //Vision Variables tm
    public static final String RINGS_NONE = "ZERO_RINGS";
    public static final String RINGS_ONE = "ONE_RING";
    public static final String RINGS_FOUR = "FOUR_RINGS";

    //Tensorflow object detector
    private TFObjectDetector objectDetector;

    // just gonna define some variables for encoders real quick dont mind me
    static final double mmPerInch               = 25.4f;    // this is jus math tho
    static final double countsPerRevolution     = 383.6f;   // Gobilda Yellowjacket 435
    static final double wheelDiameterMM         = 100;      // For figuring circumference
    static final double WheelDiameterIn         = wheelDiameterMM / mmPerInch;
    static final double wheelCircumferenceIn    = WheelDiameterIn * Math.PI;
    static final double countsPerInch         = (countsPerRevolution / wheelCircumferenceIn);

    // you will need a reference to your OpMode
    private LinearOpMode OpModeReference;
    public SevenTheRobot(LinearOpMode opMode) {
        OpModeReference = opMode;
    }

    public void initialize() {

        // imu stuff from last year cause i dont want to rebuild this from the ground up
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode

        imu = OpModeReference.hardwareMap.get(BNO055IMU.class, "IMU");

        // wheels
        FL = OpModeReference.hardwareMap.get(DcMotor.class, "FL");
        FR = OpModeReference.hardwareMap.get(DcMotor.class, "FR");
        BL = OpModeReference.hardwareMap.get(DcMotor.class, "BL");
        BR = OpModeReference.hardwareMap.get(DcMotor.class, "BR");

        BP = OpModeReference.hardwareMap.get(Servo.class, "BP");

        // arm and stuff electric boogaloo

        Arm = OpModeReference.hardwareMap.get(DcMotor.class, "Arm");
        Claw = OpModeReference.hardwareMap.get(Servo.class, "Claw");

        // outtake
        /*LR = OpModeReference.hardwareMap.get(DcMotor.class, "LR");
        LL = OpModeReference.hardwareMap.get(DcMotor.class, "LL");*/
        OT = OpModeReference.hardwareMap.get(DcMotor.class, "OT");
        IT = OpModeReference.hardwareMap.get(DcMotor.class, "IT");
        // motor arrays
        // left
        LeftMotors[0] = FL;
        LeftMotors[1] = BL;
        // right
        RightMotors[0] = FR;
        RightMotors[1] = BR;
        // all
        AllMotors[0] = FL;
        AllMotors[1] = FR;
        AllMotors[2] = BL;
        AllMotors[3] = BR;

        /*LR.setDirection(DcMotor.Direction.FORWARD);
        LL.setDirection(DcMotor.Direction.REVERSE);*/
        OT.setDirection(DcMotor.Direction.FORWARD);
        IT.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotor l : LeftMotors)
            l.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotor r : RightMotors)
            r.setDirection(DcMotorSimple.Direction.FORWARD);
        for (DcMotor m : AllMotors){
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm.setDirection(DcMotorSimple.Direction.FORWARD);
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // initialize the IMU
        imu.initialize(parameters);
    }

    // This method makes the robot turn.
    // DO NOT try to turn more than 180 degrees in either direction
    // targetAngleDifference is the number of degrees you want to turn
    // should be positive if turning left, negative if turning right
// This method makes the robot turn.
    // DO NOT try to turn more than 180 degrees in either direction
    // targetAngleDifference is the number of degrees you want to turn
    // should be positive if turning left, negative if turning right
    public void turn(double targetAngleDifference, double power) {

        // before starting the turn, take note of current angle as startAngle
        double startAngle = GetCurrentZAngle();

        // just some boolean variables to tell if we've stepped motor power down
        // might actually want more than two steps
        boolean firstStepDownComplete = false;
        boolean secondStepDownComplete = false;

        // if target angle is Negative, we're turning RIGHT
        if (targetAngleDifference < 0) {
            // turning right, so we want all right motors going backwards
            for (DcMotor m : RightMotors)
                m.setPower(-power/2);
            for (DcMotor m : LeftMotors)
                m.setPower(power/2);
            // sleep a tenth of a second
            // WARNING - not sure why this is needed - but sometimes right turns didn't work without
            OpModeReference.sleep(100);

            // we're turning right, so our target angle difference will be negative (ex: -90)
            // so GetAngleDifference will go from 0 to -90
            // keep turning while difference is greater than target
            while (OpModeReference.opModeIsActive() && GetAngleDifference(startAngle) > targetAngleDifference) {

                // THIS CODE IS FOR STEPPING DOWN MOTOR POWER
                if (!secondStepDownComplete && GetAngleDifference(startAngle) / targetAngleDifference > 0.75) {
                    for (DcMotor m : RightMotors)
                        m.setPower(-power/4);
                    for (DcMotor m : LeftMotors)
                        m.setPower(power/4);
                    secondStepDownComplete = true;
                } else if (!firstStepDownComplete && GetAngleDifference(startAngle) / targetAngleDifference > 0.50) {
                    for (DcMotor m : RightMotors)
                        m.setPower(-power/2);
                    for (DcMotor m : LeftMotors)
                        m.setPower(power/2);
                    firstStepDownComplete = true;
                }

                OpModeReference.telemetry.addData("target", targetAngleDifference);
                OpModeReference.telemetry.addData("current", GetAngleDifference(startAngle));
//                OpModeReference.telemetry.addData("LeftMotorPower", ML.getPower());
//                OpModeReference.telemetry.addData("RightMotorPower", MR.getPower());
                OpModeReference.telemetry.update();
            }
            // if targetAngleDifference is Positive, we're turning LEFT
        } else if (targetAngleDifference > 0) {
            // turning left so want all left motors going backwards
            for (DcMotor m : RightMotors)
                m.setPower(power);
            for (DcMotor m : LeftMotors)
                m.setPower(-power);

            // WARNING not sure if this sleep is needed - seemed necessary for right turns
            OpModeReference.sleep (100);

            // we're turning right, so our target angle difference will be positive (ex: 90)
            // so GetAngleDifference will go from 0 to 90
            // keep turning while difference is less than target
            while (OpModeReference.opModeIsActive() && GetAngleDifference(startAngle) < targetAngleDifference) {

                // THIS CODE IS FOR STEPPING DOWN MOTOR POWER
                if (!secondStepDownComplete && GetAngleDifference(startAngle) / targetAngleDifference > 0.75) {
                    for (DcMotor m : RightMotors)
                        m.setPower(power/4);
                    for (DcMotor m : LeftMotors)
                        m.setPower(-power/4);
                    secondStepDownComplete = true;
                } else if (!firstStepDownComplete && GetAngleDifference(startAngle) / targetAngleDifference > 0.50) {
                    for (DcMotor m : RightMotors)
                        m.setPower(power/2);
                    for (DcMotor m : LeftMotors)
                        m.setPower(-power/2);
                    firstStepDownComplete = true;
                }
                OpModeReference.telemetry.addData("target", targetAngleDifference);
                OpModeReference.telemetry.addData("current", GetAngleDifference(startAngle));
                OpModeReference.telemetry.addData("LeftMotorPower", FL.getPower());
                OpModeReference.telemetry.addData("RightMotorPower", FR.getPower());
                OpModeReference.telemetry.update();
            }
        } else {
            // is zero - not turning - just return
            return;
        }

        // turn all motors off
        stopDriving();
    }

    // this is a method to get the current heading/z angle from the IMU
    // WE WANT THE Z ANGLE :)
    // AxesOrder.XYZ means we want thirdAngle
    // AxesOrder.ZYX would mean we want firstAngle
    public double GetCurrentZAngle() {
        Orientation currentAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        return currentAngles.thirdAngle;
    }

    // This method calculates the difference of the current angle from the start angle
    // If you're left of your original angle, the value will be POSITIVE
    // If you're right of your original angle, the value will be NEGATIVE
    public double GetAngleDifference(double startAngle) {
        double angleDifference = GetCurrentZAngle() - startAngle;

        // handle going past the 0 or 180 barriers
        // where we switch from positive to negative or vice versa
        if (angleDifference < -180)
            angleDifference += 360;
        else if (angleDifference > 180)
            angleDifference -=360;

        return angleDifference;
    }

    //Autonomous Methods:

    public void stopDriving (){
        for (DcMotor m : AllMotors)
            m.setPower(0);
    }

    public void openClaw () {
        Claw.setPosition(1);
    }

    public void closeClaw () {
        Claw.setPosition(0);
    }

    public void setArm (int pos) {
        Arm.setTargetPosition(pos);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (pos > Arm.getCurrentPosition()) {
            Arm.setPower(-0.5);
        }
        else {
            Arm.setPower(0.5);
        }
    }

    public void armUp () {
        setArm(620);
    }

    public void armDown () {
        setArm(1155);
    }


    public void drive(double inches, double speed) {

        // Ensure that the opmode is still active
        if (OpModeReference.opModeIsActive()) {

            // calculate the number of ticks you want to travel (cast to integer)
            int targetTicks = (int) (2 * inches * countsPerInch);

            // reset ticks to 0 on all motors
            for (DcMotor m : AllMotors)
                m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // set target position on all motors
            // mode must be changed to RUN_TO_POSITION

            for(DcMotor m : AllMotors) {
                m.setTargetPosition(targetTicks);
                m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            // turn all motors on!
            for (DcMotor m : AllMotors)
                m.setPower(speed/2);

            // just keep looping while both motors are busy
            // stop if driver station stop button pushed
            while (OpModeReference.opModeIsActive() && ((FL.isBusy() && FR.isBusy()) && (BL.isBusy() && BR.isBusy()))) {
                OpModeReference.telemetry.addData("target ticks", targetTicks);
                OpModeReference.telemetry.addData("right current", FR.getCurrentPosition());
                OpModeReference.telemetry.addData("left current", FL.getCurrentPosition());
                OpModeReference.telemetry.update();
            }

            // once all motors get to where they need to be, turn them off
            stopDriving();

            // set motors back to RUN_USING_ENCODERS
            for (DcMotor m : AllMotors)
                m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void strafe (double inches, double speed){
        if (OpModeReference.opModeIsActive()) {
            int targetTicks = (int) (countsPerInch * inches * 2);
            for (DcMotor m : AllMotors) {
                m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            FL.setTargetPosition(targetTicks);
            FR.setTargetPosition(-targetTicks);
            BL.setTargetPosition(-targetTicks);
            BR.setTargetPosition(targetTicks);

            for (DcMotor m : AllMotors)
                m.setPower(speed / Math.sqrt(2));

            while (OpModeReference.opModeIsActive() && (FL.isBusy() && FR.isBusy() && BL.isBusy()
                    && BR.isBusy())) {
                OpModeReference.telemetry.addData("target ticks", targetTicks);
                OpModeReference.telemetry.update();
            }
            stopDriving();
            for (DcMotor m : AllMotors)
                m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

//  im not sure why multiplying it by 1.05 works, but it seems to fix it
    public void strafeL (double inches, double speed) {
        strafe(-inches * 1.05, speed);
    }
    public void strafeR (double inches, double speed) {
        strafe(inches * 1.05, speed);
    }


    //Tele-op Methods:

    public void launcherMono (double outtakePower) {
        OT.setPower(-outtakePower);
    }

    public void intakeMono  (double intakePower) {
        IT.setPower(intakePower);
    }

    public void bumper (boolean trig) {

        if (trig)
            BP.setPosition(0);
        else
            BP.setPosition(0.55);
    }

    public void mecanum () {

        double speed = -OpModeReference.gamepad1.left_stick_y / Math.sqrt(2);
        double strafe = -OpModeReference.gamepad1.left_stick_x;
        double rotate = -OpModeReference.gamepad1.right_stick_x;
        double movingSpeed;

        if (OpModeReference.gamepad1.left_bumper) {
            movingSpeed = 0.5;
        }
        else {
            movingSpeed = 1;
        }

        double leftFrontDir = Range.clip((speed - strafe - rotate), -1, 1) * movingSpeed;
        double rightFrontDir = Range.clip((speed + strafe + rotate), -1, 1) * movingSpeed;
        double leftBackDir = Range.clip((speed + strafe - rotate), -1, 1) * movingSpeed;
        double rightBackDir = Range.clip((speed - strafe + rotate), -1, 1) * movingSpeed;

        FL.setPower(leftFrontDir);
        FR.setPower(rightFrontDir);
        BL.setPower(leftBackDir);
        BR.setPower(rightBackDir);

//        OpModeReference.telemetry.addData("Central Velocity", speed*movingSpeed);
//        OpModeReference.telemetry.addData("Lateral Velocity", strafe*movingSpeed);
//        OpModeReference.telemetry.addData("Rotation", rotate*movingSpeed);
    }

    public void mecanumInv () {

        double speed = OpModeReference.gamepad1.left_stick_y / Math.sqrt(2);
        double strafe = OpModeReference.gamepad1.left_stick_x;
        double rotate = -OpModeReference.gamepad1.right_stick_x;
        double movingSpeed;

        if (OpModeReference.gamepad1.left_bumper) {
            movingSpeed = 0.5;
        }
        else {
            movingSpeed = 1;
        }

        double leftFrontDir = Range.clip((speed - strafe - rotate), -1, 1) * movingSpeed;
        double rightFrontDir = Range.clip((speed + strafe + rotate), -1, 1) * movingSpeed;
        double leftBackDir = Range.clip((speed + strafe - rotate), -1, 1) * movingSpeed;
        double rightBackDir = Range.clip((speed - strafe + rotate), -1, 1) * movingSpeed;

        FL.setPower(leftFrontDir);
        FR.setPower(rightFrontDir);
        BL.setPower(leftBackDir);
        BR.setPower(rightBackDir);

//        OpModeReference.telemetry.addData("Central Velocity", speed*movingSpeed);
//        OpModeReference.telemetry.addData("Lateral Velocity", strafe*movingSpeed);
//        OpModeReference.telemetry.addData("Rotation", rotate*movingSpeed);
    }

    // this is a method to initialize the camera, vuforia, and tensor flow
    // it needs to be called before WaitForStart()
    // keep separate from other initialization code - only use in autonomous where want to detect rings
    public void InitCamera() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        // make sure webcam name in hardware config matches deviceName here
        parameters.cameraName = OpModeReference.hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // I don't think this part can change
        int tfodMonitorViewId = OpModeReference.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", OpModeReference.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);

        // I think this one is possibly editable - currently set to 80% minimum confidence
        tfodParameters.minResultConfidence = 0.8f;
        objectDetector = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // if objectDetector is null - this will throw error, and spoil rest of autonomous
        if (objectDetector != null) {
            // first label will be for stack of 4, second for stack of 1
            // so we pass our predefined strings from above
            objectDetector.loadModelFromAsset(TensorFlowInferenceModel, RINGS_FOUR, RINGS_ONE);
            objectDetector.activate();

            // this part can be changed - check documentation
            // it adjusts digital zoom if having detection issues
            //objectDetector.setZoom(2.5, 1.78);
        }
    }

    // make sure to call this method when done detecting rings
    public void CloseCamera() {
        if (objectDetector != null)
            objectDetector.shutdown();
    }

    // the result of this method will be one of our predefined strings from above
    // in your opmode, you can check for RINGS_NONE, RINGS_ONE, or RINGS_FOUR
    // and then decide what to do based on that
    public String DetectStackSize() {
        // if we can't detect any - we'll return none
        // also, if there were other failures - things can't be initialized, etc.
        // we'll still return none, and the rest of autonomous can proceed with 1:3 chance of getting it right
        String stackSize = RINGS_NONE;

        // if we didn't detect rings at least this many times, probably just false positives?
        int necessaryRecognitions = 5;

        // variables to hold recognition counts
        int oneCount = 0;
        int fourCount = 0;
        int zeroCount = 0; // for illustration purposes

        // if objectDetector couldn't be initialized - the rest will just throw errors
        if (objectDetector != null) {

            // it takes a little time to detect - so need to try for a bit
            long currentTime = System.currentTimeMillis();
            long endTime = currentTime + 2000; // 2000ms = 2 seconds
            while (System.currentTimeMillis() < endTime) {
                List<Recognition> updatedRecognitions = objectDetector.getUpdatedRecognitions();

                if (updatedRecognitions != null) {
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(RINGS_FOUR))
                            fourCount++;
                        else if (recognition.getLabel().equals(RINGS_ONE))
                            oneCount++;
                    }
                } else {
                    // if updatedRecognitions is null - means it didn't detect anything
                    // which doesn't necessarily mean there aren't any there
                    // (MOST OF THE TIME THIS WILL BE THE CASE)
                    zeroCount++;
                }
                OpModeReference.telemetry.addData("One : ", oneCount);
                OpModeReference.telemetry.addData("Four: ", fourCount);
                OpModeReference.telemetry.addData("Zero: ", zeroCount);
                OpModeReference.telemetry.update();
            }

            // if we detected one or four rings enough times - we'll go with that
            if (oneCount >= necessaryRecognitions || fourCount >= necessaryRecognitions) {
                if (fourCount > oneCount)
                    stackSize = RINGS_FOUR;
                else
                    stackSize = RINGS_ONE;
            }
        }

        return stackSize;
    }

}
