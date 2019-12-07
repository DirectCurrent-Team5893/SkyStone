/* Copyright (c) 2019 FIRST. All rights reserved.
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

package org.firstinspires.ftc.robotcontroller.internal.FTC_5893_2019;

import com.qualcomm.robotcore.hardware.HardwareMap;



import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

/**
 * This 2019-2020 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Skystone game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
public class CustomTenserFlow5893  {
    public CustomTenserFlow5893(HardwareMap map){
        this.hardwareMap = map;
    }

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private HardwareMap hardwareMap;
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AUZDG8f/////AAABmaazVMUrgE3DsW62VUJjlMc1wra8JWNvNazE8edoKBKAtbY04213dls4wAW5jKyxCVXhGR0uR2AD90/bEtm+e7U5z63qpgQDnlDtEplsZwLZsNCsjguBCCGZuAcjvnbpfLuBQDVPJ9v0IepRczqFVg2LsMaZgjIJhYwrJOAS0xrNgDXy571FjcP9JTTsnofDkjL3vyi1tJgBWsIfCKNpkJBeMjtrM1GenDtHzwgEULtIv3XkRb0rIu1Xh/OF4N37wWOyEIm1NaT0hDJq5mHBWj/uxDnXIthdO7zxgLymdRxoWsHQg7IBfeWzp3apJnZog3OIVh7RSbn9X8b+zCgKdPGUli6K/NjBHCKIY4j/gnFm ";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    public enum SkyStonePosition
    {
        LEFT,MIDDLE,RIGHT,UNKNOWN
    }
    public void init()
    {
        initVuforia();
        if(ClassFactory.getInstance().canCreateTFObjectDetector())
        {
            initTfod();
        }
        /**
     * Activate TensorFlow Object Detection before we wait for the start command.
     * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
     **/
        if (tfod != null) {
        tfod.activate();
    }
    }



        public List<Recognition> getObjects(){
        if(tfod != null)
        {
            /*
            getUpdatedRecognitions() will return null if no new information is available
            the last time that call was made.
            */
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if(updatedRecognitions != null)
        {
            return updatedRecognitions;
        }
        }
        return null;
        }
        public static SkyStonePosition getPosition(List<Recognition> stones)
        {
            SkyStonePosition position =SkyStonePosition.UNKNOWN;
            ArrayList<Recognition> skystones = new ArrayList<>();
            Iterator iterator = stones.iterator();
            while (iterator.hasNext())
            {
                Recognition stone = (Recognition)iterator.next();
                if(stone.getLabel().equals(("Skystone")))
                {
                    skystones.add(stone);
                    skystones.remove(stone);
                }
            }
        if (skystones.size() ==1 && stones.size() ==2)
        {
            if(skystones.get(0).getRight()>stones.get(0).getRight() && skystones.get(0).getRight() > stones.get(1).getRight())
            {
                position = SkyStonePosition.RIGHT;
            }
            else if (skystones.get(0).getRight() < stones.get(0).getRight() && skystones.get(0).getRight() < stones.get(1).getRight()){
                position = SkyStonePosition.LEFT;
            }
            else{
                position = SkyStonePosition.MIDDLE;
            }
        }
        else if (skystones.size() == 1){
            if (skystones.get(0).getLeft() > 293 || skystones.get(0).getRight() > 540){
                position = SkyStonePosition.RIGHT;
            }
            else if (skystones.get(0).getLeft() < 102 || skystones.get(0).getRight() <  344){
                position = SkyStonePosition.LEFT;
            }
            else{
                position = SkyStonePosition.MIDDLE;
            }
        }


            return position;
        }


    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}

