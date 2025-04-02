package com.example.mot;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.widget.Button;
import android.widget.TextView;
import androidx.appcompat.app.AppCompatActivity;

import org.apache.commons.math3.filter.DefaultMeasurementModel;
import org.apache.commons.math3.filter.DefaultProcessModel;
import org.apache.commons.math3.filter.KalmanFilter;
import org.apache.commons.math3.filter.MeasurementModel;
import org.apache.commons.math3.filter.ProcessModel;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayDeque;
import java.util.Deque;



public class MainActivity extends AppCompatActivity implements SensorEventListener {
    private SensorManager sensorManager;
    private Sensor linearAccelerometer;
    private String currentLabel = "Idle";
    private FileWriter fileWriter;
    private KalmanFilter kalmanFilterX, kalmanFilterY, kalmanFilterZ;
    private TextView txtRawValues, txtFilteredValues;
    private boolean isSaving = false;  // 🚀 Added flag to control logging

    // Sliding window for dynamic adjustment (size = 50 samples)
    private static final int WINDOW_SIZE = 50;
    private Deque<Float> xWindow = new ArrayDeque<>(WINDOW_SIZE);
    private Deque<Float> yWindow = new ArrayDeque<>(WINDOW_SIZE);
    private Deque<Float> zWindow = new ArrayDeque<>(WINDOW_SIZE);

    // Base noise values
    private double baseQ = 0.01;
    private double baseR = 0.1;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // Initialize SensorManager
        sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        linearAccelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);

        // Initialize UI elements
        txtRawValues = findViewById(R.id.txt_raw_values);
        txtFilteredValues = findViewById(R.id.txt_filtered_values);

        // Initialize Kalman Filters for each axis
        initializeKalmanFilters(baseR);

        // 🚀 Start Saving Button
        Button btnStartSaving = findViewById(R.id.btn_start_saving);
        btnStartSaving.setOnClickListener(v -> {
            isSaving = true;
            openFileWriter();
        });

        // 🚀 Stop Saving Button
        Button btnStopSaving = findViewById(R.id.btn_stop_saving);
        btnStopSaving.setOnClickListener(v -> {
            isSaving = false;
            closeFileWriter();
        });

        // Motion Labels
        Button btnWalking = findViewById(R.id.btn_walking);
        btnWalking.setOnClickListener(v -> currentLabel = "Walking");

        Button btnIdle = findViewById(R.id.btn_idle);
        btnIdle.setOnClickListener(v -> currentLabel = "Idle");

        Button btnClimbingUp = findViewById(R.id.btn_climbing_up);
        btnClimbingUp.setOnClickListener(v -> currentLabel = "Climbing Up");

        Button btnClimbingDown = findViewById(R.id.btn_climbing_down);
        btnClimbingDown.setOnClickListener(v -> currentLabel = "Climbing Down");
    }

    private void initializeKalmanFilters(double R) {
        RealMatrix A = new Array2DRowRealMatrix(new double[][]{{1.0}});
        RealMatrix B = new Array2DRowRealMatrix(new double[][]{{0.0}});
        RealMatrix H = new Array2DRowRealMatrix(new double[][]{{1.0}});
        RealMatrix Q = new Array2DRowRealMatrix(new double[][]{{baseQ}});
        RealMatrix RMatrix = new Array2DRowRealMatrix(new double[][]{{R}});
        RealVector x0 = new ArrayRealVector(new double[]{0.0});
        RealMatrix P0 = new Array2DRowRealMatrix(new double[][]{{1.0}});

        ProcessModel pm = new DefaultProcessModel(A, B, Q, x0, P0);
        MeasurementModel mm = new DefaultMeasurementModel(H, RMatrix);

        kalmanFilterX = new KalmanFilter(pm, mm);
        kalmanFilterY = new KalmanFilter(pm, mm);
        kalmanFilterZ = new KalmanFilter(pm, mm);
    }

    private void openFileWriter() {
        try {
            File file = new File(getExternalFilesDir(null), "accelerometer_data.csv");
            fileWriter = new FileWriter(file, true);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void closeFileWriter() {
        try {
            if (fileWriter != null) {
                fileWriter.close();
                fileWriter = null;
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    protected void onResume() {
        super.onResume();
        if (linearAccelerometer != null) {
            sensorManager.registerListener(this, linearAccelerometer, SensorManager.SENSOR_DELAY_GAME);
        }
    }

    @Override
    protected void onPause() {
        super.onPause();
        sensorManager.unregisterListener(this);
        closeFileWriter();
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (event.sensor.getType() == Sensor.TYPE_LINEAR_ACCELERATION) {
            float xRaw = event.values[0];
            float yRaw = event.values[1];
            float zRaw = event.values[2];

            // Add new data to sliding windows
            if (xWindow.size() >= WINDOW_SIZE) xWindow.pollFirst();
            xWindow.addLast(xRaw);
            if (yWindow.size() >= WINDOW_SIZE) yWindow.pollFirst();
            yWindow.addLast(yRaw);
            if (zWindow.size() >= WINDOW_SIZE) zWindow.pollFirst();
            zWindow.addLast(zRaw);

            // Apply Kalman filter to each axis
            kalmanFilterX.predict();
            kalmanFilterX.correct(new ArrayRealVector(new double[]{xRaw}));
            float xFiltered = (float) kalmanFilterX.getStateEstimation()[0];

            kalmanFilterY.predict();
            kalmanFilterY.correct(new ArrayRealVector(new double[]{yRaw}));
            float yFiltered = (float) kalmanFilterY.getStateEstimation()[0];

            kalmanFilterZ.predict();
            kalmanFilterZ.correct(new ArrayRealVector(new double[]{zRaw}));
            float zFiltered = (float) kalmanFilterZ.getStateEstimation()[0];

            // Update UI with raw and filtered values
            txtRawValues.setText(String.format("Raw: X=%.2f, Y=%.2f, Z=%.2f", xRaw, yRaw, zRaw));
            txtFilteredValues.setText(String.format("Filtered: X=%.2f, Y=%.2f, Z=%.2f", xFiltered, yFiltered, zFiltered));

            // 🚀 Only log data if saving is enabled
            if (isSaving) {
                long timestamp = System.currentTimeMillis();
                try {
                    fileWriter.append(String.format("%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%s\n",
                            timestamp, xRaw, yRaw, zRaw, xFiltered, yFiltered, zFiltered, currentLabel));
                    fileWriter.flush();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // Not used
    }
}