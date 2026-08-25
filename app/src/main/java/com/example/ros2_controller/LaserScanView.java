package com.example.ros2_controller;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.AttributeSet;
import android.util.Log;
import android.view.View;

public class LaserScanView extends View {
    private static final String TAG = "ROS2_DEBUG_LaserScan";

    private Paint gridPaint, pointPaint;
    private float[] ranges;
    private float angleMin = -3.14159f;
    private float angleIncrement = 0.01745f;
    private float maxRange = 6.0f;

    public LaserScanView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    private void init() {
        gridPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        gridPaint.setColor(Color.DKGRAY);
        gridPaint.setStyle(Paint.Style.STROKE);
        gridPaint.setStrokeWidth(2f);

        pointPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        pointPaint.setColor(Color.RED);
        pointPaint.setStyle(Paint.Style.FILL);
        pointPaint.setStrokeWidth(4f);
    }

    public synchronized void updateScan(float[] ranges, float angleMin, float angleIncrement) {
        this.ranges = ranges;
        this.angleMin = angleMin;
        this.angleIncrement = angleIncrement;
        postInvalidate();
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        float cx = getWidth() / 2f;
        float cy = getHeight() / 2f;
        float radius = Math.min(cx, cy) - 10f;

        canvas.drawCircle(cx, cy, radius * 0.33f, gridPaint);
        canvas.drawCircle(cx, cy, radius * 0.66f, gridPaint);
        canvas.drawCircle(cx, cy, radius, gridPaint);
        canvas.drawLine(cx, 0, cx, getHeight(), gridPaint);
        canvas.drawLine(0, cy, getWidth(), cy, gridPaint);

        if (ranges == null || ranges.length == 0) return;

        float scale = radius / maxRange;
        int drawnPoints = 0;
        synchronized (this) {
            float currentAngle = angleMin;
            for (float r : ranges) {
                if (!Float.isInfinite(r) && !Float.isNaN(r) && r > 0.05f && r <= maxRange) {
                    float px = cx - (float) (r * Math.sin(currentAngle)) * scale;
                    float py = cy - (float) (r * Math.cos(currentAngle)) * scale;
                    canvas.drawPoint(px, py, pointPaint);
                    drawnPoints++;
                }
                currentAngle += angleIncrement;
            }
        }
    }
}