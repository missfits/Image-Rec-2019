import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

//import com.sun.org.apache.bcel.internal.generic.NameSignatureInstruction;

import java.util.HashMap;
import java.util.Collections;
import java.util.Comparator;

import org.opencv.core.*;
import org.opencv.core.Core.*;
import org.opencv.features2d.FeatureDetector;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.*;
import org.opencv.objdetect.*;
import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;

/**
* GripPipeline class.
*
* <p>An OpenCV pipeline generated by GRIP.
*
* @author GRIP
*/
public class GripPipeline implements VisionPipeline {

	//Outputs
	private Mat hslThresholdOutput = new Mat();
	private Mat blurOutput = new Mat();
	private ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();
	private ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<MatOfPoint>();
	public Mat outputImg = new Mat();
	private NetworkTable table = NetworkTableInstance.getDefault().getTable("RaspberryPi");
	static {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}

	/**
	 * This is the primary method that runs the entire pipeline and updates the outputs.
	 */
	public double midOffset, sideOffset;
	public void process(Mat source0) {
		//outputImg = source0;
		double startTime = System.currentTimeMillis();
		
		if(!table.getEntry("Reverse Drive").getBoolean(false)){
			Core.flip(source0,outputImg,-1);
		}else{
			outputImg = source0;
		}
		
		// Step HSL_Threshold0:
		Mat hslThresholdInput = outputImg;
		double[] hslThresholdHue = {64, 111};
		double[] hslThresholdSaturation = {191, 255};
		double[] hslThresholdLuminance = {78, 245};
		hslThreshold(hslThresholdInput, hslThresholdHue, hslThresholdSaturation, hslThresholdLuminance, hslThresholdOutput);

		double hslTime = System.currentTimeMillis();
		//System.out.println("hsl: " + (hslTime - startTime));

		// Step Blur0:
		Mat blurInput = hslThresholdOutput;
		BlurType blurType = BlurType.get("Box Blur");
		double blurRadius = 2;
		blur(blurInput, blurType, blurRadius, blurOutput);

		double blurTime = System.currentTimeMillis();
		//System.out.println("blur: " + (blurTime - hslTime));

		// Step Find_Contours0:
		Mat findContoursInput = blurOutput;
		boolean findContoursExternalOnly = false;
		findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);

		double findContoursTime = System.currentTimeMillis();
		//System.out.println("find contours: " + (findContoursTime - blurTime));

		// Step Filter_Contours0:
		ArrayList<MatOfPoint> filterContoursContours = findContoursOutput;
		double filterContoursMinArea = 60.0;
		double filterContoursMinPerimeter = 0;
		double filterContoursMinWidth = 0;
		double filterContoursMaxWidth = 1000;
		double filterContoursMinHeight = 5.0;
		double filterContoursMaxHeight = 1000;
		double[] filterContoursSolidity = {79, 100.0};
		double filterContoursMaxVertices = 1000000;
		double filterContoursMinVertices = 0;
		double filterContoursMinRatio = 0;
		double filterContoursMaxRatio = 1000;
		filterContours(filterContoursContours, filterContoursMinArea, filterContoursMinPerimeter, filterContoursMinWidth, filterContoursMaxWidth, filterContoursMinHeight, filterContoursMaxHeight, filterContoursSolidity, filterContoursMaxVertices, filterContoursMinVertices, filterContoursMinRatio, filterContoursMaxRatio, filterContoursOutput);
		
		double filterContoursTime = System.currentTimeMillis();
		//System.out.println("filter contours: " + (filterContoursTime - findContoursTime));

		if(table.getEntry("Vision Mode").getBoolean(false)){
			Imgproc.line(outputImg, new Point(outputImg.width()/2,0), new Point(outputImg.width()/2, outputImg.height()), new Scalar(255,0,0), 1);
			//Imgproc.circle(outputImg, new Point(outputImg.width()/2, outputImg.height()/2), 2, new Scalar(255,0,0), -1);
			//Imgproc.drawContours(outputImg, filterContoursOutput, 0, new Scalar(0,0,255));
			for (int i = 0; i <  filterContoursOutput.size(); i++){
				Imgproc.drawContours(outputImg, filterContoursOutput, i , new Scalar(isTiltedClockwise(filterContoursOutput.get(i))? 0 : 255,0,255));
			}
			table.getEntry("Contour Number").setNumber(filterContoursOutput.size());
			if(filterContoursOutput.size() >= 2){
				MatOfPoint[] targets = new MatOfPoint[2];
				if(filterContoursOutput.size() == 2){
					targets[0] = filterContoursOutput.get(0);
					targets[1] = filterContoursOutput.get(1);
				}else{
					filterContoursOutput.sort(Comparator.comparingDouble((c) -> getDistFromCenter(c)));
					//get middle 2 contours
					int center = centerIndex(filterContoursOutput);
					targets[0] = filterContoursOutput.get(center);
					int secondTargetIndex = center + (isTiltedClockwise(targets[0]) ? 1 : -1);
					targets[1] = filterContoursOutput.get(!(secondTargetIndex >= filterContoursOutput.size() || secondTargetIndex < 0) ? secondTargetIndex : center);
					//if chosen contours are \ /, shift left in the list of contours
					/*if(!(isTiltedClockwise(targets[0]) && !isTiltedClockwise(targets[1]))){
						targets[1] = targets[0];
						targets[0] = filterContoursOutput.get(filterContoursOutput.size()/2 - 1);
					}*/
				}
				
				ArrayList<Point> c1Points = new ArrayList<Point>();
				c1Points.addAll(targets[0].toList());
				ArrayList<Point> c2Points = new ArrayList<Point>();
				c2Points.addAll(targets[1].toList());

				ArrayList<Point> leftContour = c1Points.get(0).x < c2Points.get(0).x? c1Points : c2Points;
				ArrayList<Point> rightContour = c1Points.get(0).x < c2Points.get(0).x? c2Points: c1Points;

				double leftLargestX = leftContour.get(0).x;
				for(int a = 1; a < leftContour.size(); a ++){
					if(leftLargestX < leftContour.get(a).x){
						leftLargestX = leftContour.get(a).x;
					}
				}
				double rightSmallestX = rightContour.get(0).x;
				for(int a = 1; a < rightContour.size(); a ++){
					if(rightSmallestX > rightContour.get(a).x){
						rightSmallestX = rightContour.get(a).x;
					}
				}
				double midpoint = (rightSmallestX + leftLargestX)/2;
				Imgproc.line(outputImg, new Point(midpoint,0), new Point(midpoint, outputImg.height()),new Scalar (0, 0, 255), 1);
				//negative if center is to the left of midpoint
				midOffset = 0.5 - (midpoint/outputImg.width());
				//if offset is positive, want right contour
				//sideOffset = 0.5 - (findAverageX(midOffset > 0? rightContour : leftContour)/outputImg.width());
				
				double findMiddleTime = System.currentTimeMillis();
				//System.out.println("find center: " + (findMiddleTime- filterContoursTime));
			}else{
				midOffset = 0;
			}
		}
	}
	public int centerIndex(ArrayList<MatOfPoint> contours){
		int index = 0;
		for(int a = 0; a < contours.size(); a++){
			if(Math.abs(getDistFromCenter(contours.get(a))) < Math.abs(getDistFromCenter(contours.get(index)))){
				index = a;
			}
		}
		return index;
	}

	public double findAverageX(ArrayList<Point> points){
		double sum = 0;
		for(Point a : points){
			sum += a.x;
		}
		return sum/points.size();
	}
	
	public boolean isTiltedClockwise(MatOfPoint contour){
		//sorting points in contour by ascending y coordinate & storing them in pointsList
		ArrayList<Point> pointsList = new ArrayList<Point>(contour.toList());
		pointsList.sort(Comparator.comparingDouble((c) -> c.x));
		//putting top 25% of points in one ArrayList, bottom 25% in another
		/*ArrayList<Point> topPoints = new ArrayList<Point>();
		ArrayList<Point> bottomPoints = new ArrayList<Point>();
		for(int a = 0; a < pointsList.size()/4; a++){
			bottomPoints.add(pointsList.get(a));
			topPoints.add(pointsList.get(pointsList.size() - 1 - a));
		}*/
		//sorting top & bottom point lists by ascending x coordinate
		/*bottomPoints.sort(Comparator.comparingDouble((c) -> c.x));
		topPoints.sort(Comparator.comparingDouble((c) -> c.x));
		//deciding whether to check for leftmost or rightmost points in contour
		int pointIndex = getDistFromCenter(contour) <= 0 ? bottomPoints.size() - 1 : 0;
		//seeing if bottom points are to the left or right of top points (/ or \)
		return bottomPoints.get(pointIndex).x - topPoints.get(pointIndex).x >= 0;
		*/
		return pointsList.get(0).y > pointsList.get(pointsList.size() - 1).y;
	}
	public double getDistFromCenter(MatOfPoint contour){
		//negative if left of center, positive if right
		return contour.toList().get(0).x -outputImg.width()/2;
	}
	public int largestDistanceIndex(ArrayList<Point> points){
		double largestDist = 0;
		int index = 0;
		for(int a = 0; a < points.size() - 1; a ++){
			if(points.get(a + 1).x - points.get(a).x > largestDist){
				largestDist = points.get(a + 1).x - points.get(a).x;
				index = a;
			}
		}
		return index;
	}

	public boolean xTooClose(ArrayList<Point> points){
		for(int b = 0; b < points.size(); b ++){
			for(int c = b + 1; c < points.size(); c++){
				if(Math.abs(points.get(b).x - points.get(c).x) <= 1){
					return true;
				}
			}
		}
		return false;
	}
	/*public boolean inOrder(ArrayList<Point> points){
		return points.get(0).x < points.get(1).x && points.get(1).x < points.get(2).x && points.get(2).x < points.get(3).x;
	}*/
	public boolean inOrder(ArrayList<Point> points){
		for(int a = 0; a < points.size() - 1; a++){
			if(points.get(a).x >= points.get(a + 1).x){
				return false;
			}
		}
		return true;
	}

	/**
	 * This method is a generated getter for the output of a HSL_Threshold.
	 * @return Mat output from HSL_Threshold.
	 */
	public Mat hslThresholdOutput() {
		return hslThresholdOutput;
	}

	/**
	 * This method is a generated getter for the output of a Blur.
	 * @return Mat output from Blur.
	 */
	public Mat blurOutput() {
		return blurOutput;
	}

	/**
	 * This method is a generated getter for the output of a Find_Contours.
	 * @return ArrayList<MatOfPoint> output from Find_Contours.
	 */
	public ArrayList<MatOfPoint> findContoursOutput() {
		return findContoursOutput;
	}

	/**
	 * This method is a generated getter for the output of a Filter_Contours.
	 * @return ArrayList<MatOfPoint> output from Filter_Contours.
	 */
	public ArrayList<MatOfPoint> filterContoursOutput() {
		return filterContoursOutput;
	}


	/**
	 * Segment an image based on hue, saturation, and luminance ranges.
	 *
	 * @param input The image on which to perform the HSL threshold.
	 * @param hue The min and max hue
	 * @param sat The min and max saturation
	 * @param lum The min and max luminance
	 * @param output The image in which to store the output.
	 */
	private void hslThreshold(Mat input, double[] hue, double[] sat, double[] lum,
		Mat out) {
		Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HLS);
		Core.inRange(out, new Scalar(hue[0], lum[0], sat[0]),
			new Scalar(hue[1], lum[1], sat[1]), out);
	}

	/**
	 * An indication of which type of filter to use for a blur.
	 * Choices are BOX, GAUSSIAN, MEDIAN, and BILATERAL
	 */
	enum BlurType{
		BOX("Box Blur"), GAUSSIAN("Gaussian Blur"), MEDIAN("Median Filter"),
			BILATERAL("Bilateral Filter");

		private final String label;

		BlurType(String label) {
			this.label = label;
		}

		public static BlurType get(String type) {
			if (BILATERAL.label.equals(type)) {
				return BILATERAL;
			}
			else if (GAUSSIAN.label.equals(type)) {
			return GAUSSIAN;
			}
			else if (MEDIAN.label.equals(type)) {
				return MEDIAN;
			}
			else {
				return BOX;
			}
		}

		@Override
		public String toString() {
			return this.label;
		}
	}

	/**
	 * Softens an image using one of several filters.
	 * @param input The image on which to perform the blur.
	 * @param type The blurType to perform.
	 * @param doubleRadius The radius for the blur.
	 * @param output The image in which to store the output.
	 */
	private void blur(Mat input, BlurType type, double doubleRadius,
		Mat output) {
		int radius = (int)(doubleRadius + 0.5);
		int kernelSize;
		switch(type){
			case BOX:
				kernelSize = 2 * radius + 1;
				Imgproc.blur(input, output, new Size(kernelSize, kernelSize));
				break;
			case GAUSSIAN:
				kernelSize = 6 * radius + 1;
				Imgproc.GaussianBlur(input,output, new Size(kernelSize, kernelSize), radius);
				break;
			case MEDIAN:
				kernelSize = 2 * radius + 1;
				Imgproc.medianBlur(input, output, kernelSize);
				break;
			case BILATERAL:
				Imgproc.bilateralFilter(input, output, -1, radius, radius);
				break;
		}
	}

	/**
	 * Sets the values of pixels in a binary image to their distance to the nearest black pixel.
	 * @param input The image on which to perform the Distance Transform.
	 * @param type The Transform.
	 * @param maskSize the size of the mask.
	 * @param output The image in which to store the output.
	 */
	private void findContours(Mat input, boolean externalOnly,
		List<MatOfPoint> contours) {
		Mat hierarchy = new Mat();
		contours.clear();
		int mode;
		if (externalOnly) {
			mode = Imgproc.RETR_EXTERNAL;
		}
		else {
			mode = Imgproc.RETR_LIST;
		}
		int method = Imgproc.CHAIN_APPROX_SIMPLE;
		Imgproc.findContours(input, contours, hierarchy, mode, method);
	}


	/**
	 * Filters out contours that do not meet certain criteria.
	 * @param inputContours is the input list of contours
	 * @param output is the the output list of contours
	 * @param minArea is the minimum area of a contour that will be kept
	 * @param minPerimeter is the minimum perimeter of a contour that will be kept
	 * @param minWidth minimum width of a contour
	 * @param maxWidth maximum width
	 * @param minHeight minimum height
	 * @param maxHeight maximimum height
	 * @param Solidity the minimum and maximum solidity of a contour
	 * @param minVertexCount minimum vertex Count of the contours
	 * @param maxVertexCount maximum vertex Count
	 * @param minRatio minimum ratio of width to height
	 * @param maxRatio maximum ratio of width to height
	 */
	private void filterContours(List<MatOfPoint> inputContours, double minArea,
		double minPerimeter, double minWidth, double maxWidth, double minHeight, double
		maxHeight, double[] solidity, double maxVertexCount, double minVertexCount, double
		minRatio, double maxRatio, List<MatOfPoint> output) {
		final MatOfInt hull = new MatOfInt();
		output.clear();
		//operation
		for (int i = 0; i < inputContours.size(); i++) {
			final MatOfPoint contour = inputContours.get(i);
			final Rect bb = Imgproc.boundingRect(contour);
			if (bb.width < minWidth || bb.width > maxWidth) continue;
			if (bb.height < minHeight || bb.height > maxHeight) continue;
			final double area = Imgproc.contourArea(contour);
			if (area < minArea) continue;
			if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < minPerimeter) continue;
			Imgproc.convexHull(contour, hull);
			MatOfPoint mopHull = new MatOfPoint();
			mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
			for (int j = 0; j < hull.size().height; j++) {
				int index = (int)hull.get(j, 0)[0];
				double[] point = new double[] { contour.get(index, 0)[0], contour.get(index, 0)[1]};
				mopHull.put(j, 0, point);
			}
			final double solid = 100 * area / Imgproc.contourArea(mopHull);
			if (solid < solidity[0] || solid > solidity[1]) continue;
			if (contour.rows() < minVertexCount || contour.rows() > maxVertexCount)	continue;
			final double ratio = bb.width / (double)bb.height;
			if (ratio < minRatio || ratio > maxRatio) continue;
			output.add(contour);
		}
	}




}

