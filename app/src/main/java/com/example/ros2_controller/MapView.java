package com.example.ros2_controller;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Matrix;
import android.graphics.Paint;
import android.graphics.Path;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.View;

import java.util.Locale;

public class MapView extends View {
    private Bitmap mapBitmap;
    private float resolution = 0.05f;
    private float originX = 0f, originY = 0f;
    private int mapWidth = 0, mapHeight = 0;

    // 로봇 현재 위치 및 방향 (Yaw)
    private float robotWorldX = 0f, robotWorldY = 0f, robotYaw = 0f;
    private boolean hasRobotPose = false;

    // Nav2 목표 지점 (Goal)
    private float goalWorldX = 0f, goalWorldY = 0f;
    private float goalPixelX = -1f, goalPixelY = -1f;
    private boolean hasGoal = false;

    private Paint goalPaint, robotPaint, robotHeadingPaint, textPaint;
    private final Path arrowPath = new Path();
    private final Matrix matrix = new Matrix();

    public interface OnGoalSelectedListener {
        void onGoalSelected(float worldX, float worldY);
    }
    private OnGoalSelectedListener goalListener;

    public MapView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    private void init() {
        goalPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        goalPaint.setColor(Color.RED);
        goalPaint.setStyle(Paint.Style.FILL_AND_STROKE);
        goalPaint.setStrokeWidth(4f);

        // 로봇 본체 마커 (Cyan)
        robotPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        robotPaint.setColor(Color.parseColor("#00E5FF"));
        robotPaint.setStyle(Paint.Style.FILL);

        // 로봇 전방 헤딩 화살표 (Lime Green)
        robotHeadingPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        robotHeadingPaint.setColor(Color.parseColor("#76FF03"));
        robotHeadingPaint.setStyle(Paint.Style.FILL_AND_STROKE);
        robotHeadingPaint.setStrokeWidth(3f);

        textPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        textPaint.setColor(Color.CYAN);
        textPaint.setTextSize(26f);
    }

    public void setOnGoalSelectedListener(OnGoalSelectedListener listener) {
        this.goalListener = listener;
    }

    /**
     * 2D 점유 격자 지도 업데이트
     */
    public synchronized void updateMap(byte[] data, int width, int height, float resolution, float originX, float originY) {
        this.mapWidth = width;
        this.mapHeight = height;
        this.resolution = resolution;
        this.originX = originX;
        this.originY = originY;

        int[] pixels = new int[width * height];
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int srcIdx = (height - 1 - y) * width + x;
                if (srcIdx < data.length) {
                    byte val = data[srcIdx];
                    int color;
                    if (val == -1) {
                        color = 0xFF373737; // 미탐색 영역 (Dark Gray)
                    } else if (val == 0) {
                        color = 0xFFE0E0E0; // 자유 주행 영역 (Light Gray)
                    } else if (val == 100) {
                        color = 0xFF000000; // 장애물/벽 (Black)
                    } else {
                        color = 0xFFFF5722; // Costmap 위험 영역 (Orange)
                    }
                    pixels[y * width + x] = color;
                }
            }
        }
        mapBitmap = Bitmap.createBitmap(pixels, width, height, Bitmap.Config.ARGB_8888);
        postInvalidate();
    }

    /**
     * 실시간 로봇 위치 및 방향(Yaw) 업데이트
     */
    public synchronized void updateRobotPose(float worldX, float worldY, float yaw) {
        this.robotWorldX = worldX;
        this.robotWorldY = worldY;
        this.robotYaw = yaw;
        this.hasRobotPose = true;
        postInvalidate();
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        if (mapBitmap != null) {
            float scaleX = (float) getWidth() / mapBitmap.getWidth();
            float scaleY = (float) getHeight() / mapBitmap.getHeight();
            float scale = Math.min(scaleX, scaleY);

            float dx = (getWidth() - mapBitmap.getWidth() * scale) / 2f;
            float dy = (getHeight() - mapBitmap.getHeight() * scale) / 2f;

            matrix.reset();
            matrix.postScale(scale, scale);
            matrix.postTranslate(dx, dy);
            canvas.drawBitmap(mapBitmap, matrix, null);

            // 1. 로봇 현재 위치 & 헤딩 화살표 그리기
            if (hasRobotPose && resolution > 0) {
                float robotPixelX = (robotWorldX - originX) / resolution;
                float robotPixelY = mapHeight - 1 - (robotWorldY - originY) / resolution;

                float screenRobotX = dx + robotPixelX * scale;
                float screenRobotY = dy + robotPixelY * scale;

                // 로봇 본체 원
                canvas.drawCircle(screenRobotX, screenRobotY, 12f, robotPaint);

                // 로봇 방향(Yaw) 화살표 그리기
                float arrowLen = 28f;
                float headX = screenRobotX + (float) (arrowLen * Math.cos(robotYaw));
                float headY = screenRobotY - (float) (arrowLen * Math.sin(robotYaw)); // Y축 반전

                canvas.drawLine(screenRobotX, screenRobotY, headX, headY, robotHeadingPaint);
                canvas.drawCircle(headX, headY, 4f, robotHeadingPaint);
            }

            // 2. 선택된 Nav2 Goal 십자선 마커 그리기
            if (hasGoal && goalPixelX >= 0 && goalPixelY >= 0) {
                float screenX = dx + goalPixelX * scale;
                float screenY = dy + goalPixelY * scale;

                canvas.drawCircle(screenX, screenY, 12f, goalPaint);
                canvas.drawLine(screenX - 25, screenY, screenX + 25, screenY, goalPaint);
                canvas.drawLine(screenX, screenY - 25, screenX, screenY + 25, goalPaint);
                canvas.drawText(String.format(Locale.getDefault(), "Goal: (%.2f, %.2f)", goalWorldX, goalWorldY), screenX + 15, screenY - 15, textPaint);
            }
        } else {
            canvas.drawText("맵 수신 대기 중 (/map)...", 40, getHeight() / 2f, textPaint);
        }
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        if (event.getAction() == MotionEvent.ACTION_DOWN && mapBitmap != null) {
            float scaleX = (float) getWidth() / mapBitmap.getWidth();
            float scaleY = (float) getHeight() / mapBitmap.getHeight();
            float scale = Math.min(scaleX, scaleY);

            float dx = (getWidth() - mapBitmap.getWidth() * scale) / 2f;
            float dy = (getHeight() - mapBitmap.getHeight() * scale) / 2f;

            float touchX = (event.getX() - dx) / scale;
            float touchY = (event.getY() - dy) / scale;

            if (touchX >= 0 && touchX < mapWidth && touchY >= 0 && touchY < mapHeight) {
                goalPixelX = touchX;
                goalPixelY = touchY;

                // 픽셀 좌표를 ROS 미터 단위 좌표로 역변환
                float gridY = mapHeight - 1 - touchY;
                goalWorldX = originX + touchX * resolution;
                goalWorldY = originY + gridY * resolution;
                hasGoal = true;

                if (goalListener != null) {
                    goalListener.onGoalSelected(goalWorldX, goalWorldY);
                }
                invalidate();
                return true;
            }
        }
        return super.onTouchEvent(event);
    }
}