package org.firstinspires.ftc.teamcode.util.misc;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

public class OpenCVHelper {

    HardwareMap hardwareMap;
    Telemetry telemetry;
    WebcamName webcam;
    OpenCvCamera camera;

    public OpenCVHelper(HardwareMap hW, Telemetry tm) {
        hardwareMap = hW;
        telemetry = tm;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = hardwareMap.get(WebcamName.class, "CAMERA_NAME");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcam); // cameraMonitorViewId
        telemetry.addData("Camera Status", "Initializing");
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                telemetry.addData("Camera Status", "Opened");
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Status", "Error");
                telemetry.addData("Camera Error Code", errorCode);
            }
        });
    }

    public void beginStreaming() {
        camera.openCameraDevice();
        camera.startStreaming(1920, 1080);

    }

}
