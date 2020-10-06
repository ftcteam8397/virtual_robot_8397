package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.apache.commons.math3.linear.*;
import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

/**
 * Represents a 2D Cubic Spline which has numSegments segments. Each segment is a parametric cubic function with
 * parameter s (0 <= s <= 1). There are numSegments+1 control points, and the spline passes through each of
 * those points. The points mark the beginning and end of each segment. The cubic segments (i.e., the coefficients
 * for each segment) are determined by applying the following rules:
 *
 * 1. Segment n evaluates to point n at s=0, and evaluates to point n+1 at s=1;
 * 2. The first and second deriviatives are continuous at the intermediate points;
 * 3. The client specifies the direction of travel at the beginning and end.
 */

public class CubicSpline {

    private int numSegments;
    private VectorF[] points;
    private CubicFunction[] segments;
    private int index = 0;

    /**
     * Constructor
     * @param pts  The control points through which the spline must pass (x0, y0, x1, y1, x2, y2, etc.)
     * @param startDir The required initial direction of travel, in degrees.
     * @param endDir The required ending direction of travel, in degrees.
     */
    public CubicSpline(float[] pts, float startDir, float endDir){
        startDir = startDir * (float)Math.PI/180;
        endDir = endDir * (float)Math.PI/180;
        numSegments = pts.length / 2 - 1;
        points = new VectorF[numSegments+1];
        for (int i=0; i<=numSegments; i++){
            points[i] = new VectorF(pts[2*i], pts[2*i+1]);
        }

        /**
         * The matrix equation to get the coefficients for all of the segments has the form:
         *
         *    MD = V ,
         *
         *    where M = Triadiagonal Matrix
         *    D = array of first derivatives at each control point
         *    V = input array, where each entry is a linear combination of control points.
         *
         *    Start by creating the tridiagonal matrix
         */

        double[][] mData = new double[numSegments+1][numSegments+1];

        for (int i=0; i<=numSegments; i++){
            if (i==0){
                //Use this to specify first derivative (i.e., travel direction) at beginning
                mData[i][i] = 1.0;
                //Would use the code below to get second derivitive of 0 at beginning
//                mData[i][i] = 2.0;
//                mData[i][i+1] = 1.0;
            } else if (i == numSegments) {
                //Use this to specify first derivative (i.e., travel direction) at end
                mData[i][i] = 1.0;
                //Would use the code below to get second derivative of 0 at end
//                mData[i][i] = 2.0;
//                mData[i][i-1] = 1.0;
            } else {
                mData[i][i-1] = 1.0;
                mData[i][i] = 4.0;
                mData[i][i+1] = 1.0;
            }
        }

        RealMatrix matrix = new Array2DRowRealMatrix(mData);

        //Invert the tridiagonal matrix
        RealMatrix invMatrix = MatrixUtils.inverse(matrix);

        //Generate the input vectors (for x and y)
        double[] xData = new double[numSegments+1];
        double[] yData = new double[numSegments+1];

        //The first derivatives at the beginning and end ( D0 and DN are obtained from the begin and end travel directions)
        float startDist = points[1].subtracted(points[0]).magnitude();
        float endDist = points[numSegments].subtracted(points[numSegments-1]).magnitude();
        float D0x = (float)Math.cos(startDir) * startDist;
        float D0y = (float)Math.sin(startDir)*startDist;
        float DNx = (float)Math.cos(endDir) * endDist;
        float DNy = (float)Math.sin(endDir) * endDist;

        for (int i=0; i<=numSegments; i++){
            if (i==0){
                //Use this to specify the beginning derivative explicitly
                xData[i] = D0x;
                yData[i] = D0y;
                //Use the code below to make the beginning second derivative equal to zero
//                xData[i] = 3.0 * (points[i+1].get(0) - points[i].get(0));
//                yData[i] = 3.0 * (points[i+1].get(1) - points[i].get(1));
            } else if (i==numSegments){
                //Use this to specify the ending derivative explicitly
                xData[i] = DNx;
                yData[i] = DNy;
                //Use the code below to make the ending second derivative equal to zero
//                xData[i] = 3.0 * (points[i].get(0) - points[i-1].get(0));
//                yData[i] = 3.0 * (points[i].get(1) - points[i-1].get(1));
            } else {
                xData[i] = 3.0 * (points[i+1].get(0) - points[i-1].get(0));
                yData[i] = 3.0 * (points[i+1].get(1) - points[i-1].get(1));
            }
        }

        RealVector xVec = new ArrayRealVector(xData);
        RealVector yVec = new ArrayRealVector(yData);

        //Compute the X and Y components of the first derivative for all control points
        RealVector Dx = invMatrix.operate(xVec);
        RealVector Dy = invMatrix.operate(yVec);

        /**From the control points themselves, and the first derivatives, compute the coefficients
         *   a, b, c, d
         * for each cubic function and create an array of cubic function objects
         */
        segments = new CubicFunction[numSegments];
        for (int i=0; i<numSegments; i++){
            VectorF a = points[i];
            VectorF b = new VectorF((float)Dx.getEntry(i), (float)Dy.getEntry(i));
            VectorF c = points[i+1].subtracted(points[i]).multiplied(3)
                    .subtracted(new VectorF((float)(2*Dx.getEntry(i)+Dx.getEntry(i+1)), (float)(2*Dy.getEntry(i)+Dy.getEntry(i+1))));
            VectorF d = points[i].subtracted(points[i+1]).multiplied(2)
                    .added(new VectorF((float)(Dx.getEntry(i)+Dx.getEntry(i+1)), (float)(Dy.getEntry(i)+Dy.getEntry(i+1))));
            segments[i] = new CubicFunction(a, b, c, d);
        }
    }

    /**
     * Set the current segment index
     * @param i
     */
    public void setIndex(int i){ index = i; }

    /**
     * Get the current segment index
     * @return current segment index
     */
    public int getIndex(){ return index; }

    /**
     * Get the current CubicFunction object (i.e., corresponding to the current segment index)
     * @return
     */
    public CubicFunction getSegment() { return segments[index]; }

    /**
     * Get the total number of segments
     * @return number of segments
     * Note:  number of control points = number of segments PLUS 1
     */
    public int getNumSegments() { return numSegments; }

    /**
     * Get position (x,y) corresponding to parameter s, and the current segment
     * @param s
     * @return position
     */
    public VectorF p(float s) { return getSegment().p(s); }

    /**
     * Get the first derivative with respect to s for the current segment
     * @param s
     * @return first derivative
     */
    public VectorF d1(float s) { return getSegment().d1(s); }

    /**
     * Get the second derivative with respect to s for the current segment
     * @param s
     * @return second derivative
     */
    public VectorF d2(float s) { return getSegment().d2(s); }

    /**
     * Given a previous value of s (s0), and a point (x0,y0), determine the new value of s that corresponds to
     * the closest point on the current segment to the point (x0,y0). If this value is greater than one (and current
     * segment is not the final segment), increment the segment index (i.e., move to next segment), and (using a seed
     * of 0) find the value of s that corresponds to the closest point on the next segment to (x0,y0). Return the
     * value of s. Note that the only circumstance under which a value s>1 will be returned is if the current segment
     * is the final segment, and the closest point is beyond the final control point.
     *
     * @param x0 x value of test point
     * @param y0 y value of test point
     * @param s0 previous s value (essentially a seed for the numeric computation.
     * @param opMode
     * @return s value giving point on spline that is closest to the test point
     */
    public float nextClosestPt(float x0, float y0, float s0, LinearOpMode opMode){

        float s = segments[index].findClosestPt(x0, y0, s0, opMode);
        if (s > 1 && index < numSegments-1){
            index++;
            s = segments[index].findClosestPt(x0, y0, 0, opMode);
        }
        return s;

    }



}
