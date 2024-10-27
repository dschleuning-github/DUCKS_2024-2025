package org.firstinspires.ftc.teamcode.processors;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class FirstVisionProcessor implements VisionProcessor {
    public Rect rectLeft = new Rect(10,100,200,200);
    public Rect rectMiddle = new Rect(220,100,200,200);
    public Rect rectRight = new Rect(430,100,200,200);
    Selected selection = Selected.NONE;

    public double[] satTest = new double[3];

    Mat submat = new Mat();
    Mat hsvMat = new Mat();

    @Override
    public void init(int width, int height, CameraCalibration calibration){}
    
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos){
        Imgproc.cvtColor(frame,hsvMat,Imgproc.COLOR_RGB2HSV);

        double satRectLeft = getAveSaturation(hsvMat, rectLeft);
        double satRectMiddle = getAveSaturation(hsvMat, rectMiddle);
        double satRectRight = getAveSaturation(hsvMat, rectRight);

//        satTest = 6.2; //getAveSaturation(hsvMat, rectRight);
        satTest[0] = getAveSaturation(hsvMat, rectLeft);
        satTest[1] = getAveSaturation(hsvMat, rectMiddle);
        satTest[2] = getAveSaturation(hsvMat, rectRight);

        if ((satRectLeft > satRectMiddle) && (satRectLeft > satRectRight)){
            return Selected.LEFT;
        } else if ((satRectMiddle > satRectLeft) && (satRectMiddle > satRectRight)){
            return Selected.MIDDLE;
        }
        return Selected.RIGHT;
    }

    protected double getAveSaturation(Mat input, Rect rect){
        submat =input.submat(rect);
        Scalar color = Core.mean(submat);
        return color.val[1];
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToConvasPx){
        int left = Math.round(rect.x * scaleBmpPxToConvasPx);
        int top = Math.round(rect.y * scaleBmpPxToConvasPx);
        int right = left + Math.round(rect.width*scaleBmpPxToConvasPx);
        int bottom = top + Math.round(rect.height*scaleBmpPxToConvasPx);
        return new android.graphics.Rect(left, top, right, bottom);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext){
        Paint selectedPaint = new Paint();
        selectedPaint.setColor(Color.RED);
        selectedPaint.setStyle(Paint.Style.STROKE);
        selectedPaint.setStrokeWidth(scaleCanvasDensity*4);

        Paint nonSelectedPaint = new Paint();
        nonSelectedPaint.setColor(Color.GREEN);
        nonSelectedPaint.setStyle(Paint.Style.STROKE);

        android.graphics.Rect drawRetangleLeft = makeGraphicsRect(rectLeft, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRetangleMiddle = makeGraphicsRect(rectMiddle, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRetangleRight = makeGraphicsRect(rectRight, scaleBmpPxToCanvasPx);

        nonSelectedPaint.setTextSize(24);
        canvas.drawText(String.format("width=%d",onscreenWidth ) ,200,30,nonSelectedPaint);
        canvas.drawText(String.format("height=%d",onscreenHeight ) ,200,60,nonSelectedPaint);
        canvas.drawText(String.format("height=%f",scaleBmpPxToCanvasPx ) ,200,90,nonSelectedPaint);

        canvas.drawText(String.format("satTest=%.1f", satTest[0] ) ,20,400,nonSelectedPaint);


        selection = (Selected) userContext;
        switch(selection){
            case LEFT:
                canvas.drawRect(drawRetangleLeft, selectedPaint);
                canvas.drawRect(drawRetangleMiddle,nonSelectedPaint);
                canvas.drawRect(drawRetangleRight, nonSelectedPaint);
                break;
            case MIDDLE:
                canvas.drawRect(drawRetangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRetangleMiddle,selectedPaint);
                canvas.drawRect(drawRetangleRight, nonSelectedPaint);
                break;
            case RIGHT:
                canvas.drawRect(drawRetangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRetangleMiddle,nonSelectedPaint);
                canvas.drawRect(drawRetangleRight, selectedPaint);
                break;
            case NONE   :
                canvas.drawRect(drawRetangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRetangleMiddle,nonSelectedPaint);
                canvas.drawRect(drawRetangleRight, nonSelectedPaint);
                break;

        }
        //canvas.drawRect(makeGraphicsRect(rect, scaleBmpPxToCanvasPx), rectPaint);
    }
    public Selected getSelection() {
        return selection;
    }

    public double[] getData(){
        return satTest;
    }

    public enum Selected{
        NONE,
        LEFT,
        MIDDLE,
        RIGHT
    }
}
