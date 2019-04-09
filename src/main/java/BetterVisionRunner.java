/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import org.opencv.core.Mat;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServerSharedStore;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionPipeline;

/**
 * Add your docs here.
 */
public class BetterVisionRunner<P extends VisionPipeline> {
    private final CvSink imgFeed = new CvSink("VisionRunner CvSink");
    private final P pipeline;
    private final Mat image = new Mat();
    private final Listener<? super P> listener;
    private volatile boolean enabled = true;
    private VideoSource frontCam, backCam;
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("RaspberryPi");
  
    /**
     * Listener interface for a callback that should run after a pipeline has processed its input.
     *
     * @param <P> the type of the pipeline this listener is for
     */
    @FunctionalInterface
    public interface Listener<P extends VisionPipeline> {
      /**
       * Called when the pipeline has run. This shouldn't take much time to run because it will delay
       * later calls to the pipeline's {@link VisionPipeline#process process} method. Copying the
       * outputs and code that uses the copies should be <i>synchronized</i> on the same mutex to
       * prevent multiple threads from reading and writing to the same memory at the same time.
       *
       * @param pipeline the vision pipeline that ran
       */
      void copyPipelineOutputs(P pipeline);
  
    }
  
    /**
     * Creates a new vision runner. It will take images from the {@code videoSource}, send them to
     * the {@code pipeline}, and call the {@code listener} when the pipeline has finished to alert
     * user code when it is safe to access the pipeline's outputs.
     *
     * @param videoSource the video source to use to supply images for the pipeline
     * @param pipeline    the vision pipeline to run
     * @param listener    a function to call after the pipeline has finished running
     */
    public BetterVisionRunner(VideoSource frontCam, VideoSource backCam, P pipeline, Listener<? super P> listener) {
      this.pipeline = pipeline;
      this.listener = listener;
      imgFeed.setSource(frontCam);
      this.frontCam = frontCam;
      this.backCam = backCam;
    }
  
    /**
     * Runs the pipeline one time, giving it the next image from the video source specified
     * in the constructor. This will block until the source either has an image or throws an error.
     * If the source successfully supplied a frame, the pipeline's image input will be set,
     * the pipeline will run, and the listener specified in the constructor will be called to notify
     * it that the pipeline ran.
     *
     * <p>This method is exposed to allow teams to add additional functionality or have their own
     * ways to run the pipeline. Most teams, however, should just use {@link #runForever} in its own
     * thread using a {@link VisionThread}.</p>
     */
    public void runOnce() {
      Long id = CameraServerSharedStore.getCameraServerShared().getRobotMainThreadId();
  
      if (id != null && Thread.currentThread().getId() == id.longValue()) {
        throw new IllegalStateException(
            "VisionRunner.runOnce() cannot be called from the main robot thread");
      }
      runOnceInternal();
    }
  
    private void runOnceInternal() {
        setImgFeed();
        long frameTime = imgFeed.grabFrame(image);
        if (frameTime == 0) {
            // There was an error, report it
            String error = imgFeed.getError();
            CameraServerSharedStore.getCameraServerShared().reportDriverStationError(error);
        } else {
            // No errors, process the image
            pipeline.process(image);
            listener.copyPipelineOutputs(pipeline);
        }
    }

    private void setImgFeed(){
        imgFeed.setSource(table.getEntry("Reverse Drive").getBoolean(false) ? backCam : frontCam);
    }
  
    /**
     * A convenience method that calls {@link #runOnce()} in an infinite loop. This must
     * be run in a dedicated thread, and cannot be used in the main robot thread because
     * it will freeze the robot program.
     *
     * <p><strong>Do not call this method directly from the main thread.</strong></p>
     *
     * @throws IllegalStateException if this is called from the main robot thread
     * @see VisionThread
     */
    public void runForever() {
      Long id = CameraServerSharedStore.getCameraServerShared().getRobotMainThreadId();
  
      if (id != null && Thread.currentThread().getId() == id.longValue()) {
        throw new IllegalStateException(
            "VisionRunner.runForever() cannot be called from the main robot thread");
      }
      while (enabled && !Thread.interrupted()) {
        runOnceInternal();
      }
    }
  
    /**
     * Stop a RunForever() loop.
     */
    public void stop() {
      enabled = false;
    }
}
