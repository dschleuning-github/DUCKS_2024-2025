package org.firstinspires.ftc.teamcode.processors_test;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Rect;

public class ThreeRectanglesProcessor implements VisionProcessor {
    public Rect rectLeft = new Rect(110,20,50,50);
    public Rect rectMiddle = new Rect(160,20,50,50);
    public Rect rectRight = new Rect(210,20,50,50);
    Selected selection = Selected.RIGHT;
    @Override
    public void init(int width, int height, CameraCalibration calibration){}
    
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos){
        return null;
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToConvasPx){
        int left = Math.round(rect.x * scaleBmpPxToConvasPx);
        int top = Math.round(rect.y * scaleBmpPxToConvasPx);
        int right = left + Math.round(rect.width*scaleBmpPxToConvasPx);
        int bottom = top + Math.round(rect.height*scaleBmpPxToConvasPx);
        return new android.graphics.Rect(left, top, right, bottom);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext){
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
    public enum Selected{
        NONE,
        LEFT,
        MIDDLE,
        RIGHT
    }
}
