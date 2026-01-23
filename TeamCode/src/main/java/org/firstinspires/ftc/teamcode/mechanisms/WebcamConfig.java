package org.firstinspires.ftc.teamcode.mechanisms;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;


public class WebcamConfig {
    public static final int WIDTH = 1280;
    public static final int HEIGHT = 720;
    public static VisionPortal buildVisionPortal(
            HardwareMap hardwareMap,
            VisionProcessor processor
    ){
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(webcam);
        builder.setCameraResolution(new Size(WIDTH, HEIGHT));
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.enableLiveView(true);

        if(processor != null){
            builder.addProcessor(processor);
        }

        return builder.build();
    }
}
