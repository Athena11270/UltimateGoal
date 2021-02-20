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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

// imports needed for vision stuff
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import java.util.List;

// This is not an OpMode.  It is a class that holds all the boring stuff

// ************************************************************************************************
// ** This is an example based on ConceptTensorFlowObjectDetectionWebcam.java, which was         **
// ** provided with the FTC SDK sample code.  Do not use this.  This was easier for me to        **
// ** follow - so hopefully will help provide some clarity.  Enjoy  --Mike                       **
// ************************************************************************************************

@Disabled
public class MKTestHardware {

    // this is the vuforia key I got from https://developer.vuforia.com/license-manager
    private static final String VUFORIA_KEY =
            "AZ3ryeD/////AAABmeY7zPkTgkgivhkyR/Wv11tYFIVOPjl+cbNTS5QTjJcDTzTxj2tsXDiSAYKEk0bwRs5LARKJohUORPn9Jg5ti0zDqAugPqNQaHDd24+J0E85fxlpZRPw75z+XNOy+x5s6zMR5hwHKKEwQ+/CxT8vppDZ4UOi1pns8UGn2OptQbZc42qEiwcaImzVmnC1OG+XLniEgCyrIFrCXEA727Wa+cKiaTAHIjQughcOmM7E9+gg8obCjU0iDt91ibtNCSB/PRzyXmxK2GRcwTl7DRHiu7DhaC5JqTWj4Yx4nMbkU0YF5tskagd47y0xyl1uzxi/eqJW5NxmhZjUravn9K5yl+w4NwWou5j5lnf/X/Mddn5N";

    // this is the tensor flow reference model file included with the FTC SDK
    // we could make our own if we wanted
    private static final String TensorFlowInferenceModel = "UltimateGoal.tflite";

    // vuforia is only used by object detector for accessing camera
    private VuforiaLocalizer vuforia;

    // public variables so we can safely compare strings (they're only defined in one place - here)
    public static final String RINGS_NONE = "ZERO_RINGS";
    public static final String RINGS_ONE = "ONE_RING";
    public static final String RINGS_FOUR = "FOUR_RINGS";

    // tensor flow object detector
    private TFObjectDetector objectDetector;

    // you will need a reference to your OpMode
    private LinearOpMode OpModeReference;
    public MKTestHardware(LinearOpMode opMode) {
        OpModeReference = opMode;
    }

    public void initialize() {
        // this is regular robot hardware initialization code - we don't want to mess with this
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
                        if (recognition.getLabel() == RINGS_FOUR)
                            fourCount++;
                        else if (recognition.getLabel() == RINGS_ONE)
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
