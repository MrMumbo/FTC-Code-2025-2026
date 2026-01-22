package org.firstinspires.ftc.teamcode.mechanisms;

import android.graphics.Canvas;

import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;

import java.io.File;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.atomic.AtomicBoolean;

public class FrameCaptureProcessor implements VisionProcessor {

    private static final String DIR = "/sdcard/FIRST/calib/";

    private final AtomicBoolean saveNext = new AtomicBoolean(false);

    // Save frames off the camera thread so VisionPortal doesn't freeze
    private final LinkedBlockingQueue<Mat> saveQueue = new LinkedBlockingQueue<>(2);
    private final Thread saverThread;

    private volatile int imageCount = 0;
    private volatile String lastSavedPath = "none";
    private volatile String lastError = "none";

    public FrameCaptureProcessor() {
        // Ensure directory exists
        File dir = new File(DIR);
        //noinspection ResultOfMethodCallIgnored
        dir.mkdirs();

        saverThread = new Thread(() -> {
            while (!Thread.currentThread().isInterrupted()) {
                Mat toSave = null;
                try {
                    toSave = saveQueue.take();

                    String path = DIR + "img_" + imageCount + ".png";
                    boolean ok = Imgcodecs.imwrite(path, toSave);

                    if (ok) {
                        lastSavedPath = path;
                        lastError = "none";
                        imageCount++;
                    } else {
                        lastError = "imwrite returned false";
                    }
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                } catch (Exception e) {
                    lastError = "save exception: " + e.getMessage();
                } finally {
                    if (toSave != null) {
                        toSave.release();
                    }
                }
            }
        });

        saverThread.setName("FrameSaver");
        saverThread.start();
    }

    /** Call this from your OpMode when you want to save the next frame. */
    public void requestSaveNextFrame() {
        saveNext.set(true);
    }

    public int getImageCount() {
        return imageCount;
    }

    public String getLastSavedPath() {
        return lastSavedPath;
    }

    public String getLastError() {
        return lastError;
    }

    /** Call this when your OpMode ends to stop the background thread cleanly. */
    public void shutdown() {
        saverThread.interrupt();
        Mat m;
        while ((m = saveQueue.poll()) != null) {
            m.release();
        }
    }

    @Override
    public void init(int width, int height,
                     org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration calibration) {
        // No-op
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        if (saveNext.compareAndSet(true, false)) {
            Mat copy = frame.clone(); // MUST clone; frame is reused by pipeline

            // If queue is full, drop (prevents memory buildup / stalls)
            boolean offered = saveQueue.offer(copy);
            if (!offered) {
                copy.release();
                lastError = "save queue full (try again)";
            }
        }
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // No overlay drawing
    }
}
