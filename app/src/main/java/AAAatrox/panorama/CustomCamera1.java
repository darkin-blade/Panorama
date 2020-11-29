package AAAatrox.panorama;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.content.Context;
import android.content.pm.PackageManager;
import android.graphics.SurfaceTexture;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CameraMetadata;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.media.MediaRecorder;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.util.Size;
import android.view.Surface;
import android.view.TextureView;
import android.view.View;
import android.widget.Button;

import androidx.annotation.NonNull;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import static AAAatrox.panorama.MainActivity.PERMISSION_CAMERA_REQUEST_CODE;
import static AAAatrox.panorama.MainActivity.appPath;

public class CustomCamera1 extends Activity {
    static public void infoLog(String log) {
        Log.i("fuck", log);
    }

    static public void infoError(Exception e) {
        ByteArrayOutputStream stream = new ByteArrayOutputStream();
        e.printStackTrace(new PrintStream(stream));
        infoLog(stream.toString());
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
        // 权限处理
        if (requestCode == PERMISSION_CAMERA_REQUEST_CODE) {
            if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                // 允许
            } else {
                // 拒绝
                infoLog("mCamera not permitted");
            }
        }
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.custom_camera_1);
        infoLog("create mCamera");

        initCamera();
        initUI();
    }

    @Override
    public void onBackPressed() {
        backToMain(0);// 禁止返回
    }

    @Override
    public void onDestroy() {
        unregisterSensor();
        infoLog("destroy mCamera");

        super.onDestroy();
    }

    /* 常量 */
    double ratio = 0.25d;

    /* UI组件 */
    Button btnStart, btnConfirm, btnCancel, btnDebug;
    TextureView cameraPreview;
    /* 控制 */
    boolean isRecording;

    /* 录制 */
    File videoFile;
    MediaRecorder mediaRecorder;
    /* 相机相关 */
    String cameraId;
    CameraManager cameraManager;
    CameraDevice cameraDevice;// 摄像头设备,(参数:预览尺寸,拍照尺寸等)
    CameraCaptureSession previewSession;// 相机捕获会话,用于处理拍照和预览的工作
    CaptureRequest.Builder mPreviewRequestBuilder;// 捕获请求,定义输出缓冲区及显示界面(TextureView或SurfaceView)
    /* 预览 */
    Size previewSize;// 在textureView预览的尺寸
    Size videoSize;// 拍摄的尺寸
    Handler backgroundHandler;

    /* 传感器 */
    SensorManager mSensorManager;
    Sensor mGravity;// 重力传感器
    Sensor mRotation;// 旋转矢量传感器

    void initCamera() {
        /* 初始化变量*/
        isRecording = false;

        /* 初始化传感器 */
        mSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        mGravity  = mSensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY);// 重力传感器
        mRotation = mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);// 旋转矢量传感器
        /* 注册传感器的监听 */
        mSensorManager.registerListener(mSensorEventListener, mRotation, SensorManager.SENSOR_DELAY_FASTEST);
        mSensorManager.registerListener(mSensorEventListener, mGravity, SensorManager.SENSOR_DELAY_FASTEST);
    }

    void initUI() {
        /* 初始化按钮 */
        btnStart   = findViewById(R.id.start_button);
        btnConfirm = findViewById(R.id.confirm_button);
        btnCancel  = findViewById(R.id.cancel_button);
        btnDebug   = findViewById(R.id.debug_button);

        btnStart.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                if (!isRecording) {
                    startRecord();
                } else {
                    stopRecord();
                }
            }
        });

        btnConfirm.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                backToMain(1);
            }
        });

        btnCancel.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                backToMain(0);
            }
        });

        /* 初始化相机预览 */
        cameraPreview = findViewById(R.id.camera_preview);
        cameraPreview.setSurfaceTextureListener(new TextureView.SurfaceTextureListener() {
            @Override
            public void onSurfaceTextureAvailable(SurfaceTexture surfaceTexture, int width, int height) {
                openCamera(width, height);
            }

            @Override
            public void onSurfaceTextureSizeChanged(SurfaceTexture surfaceTexture, int width, int height) {

            }

            @Override
            public boolean onSurfaceTextureDestroyed(SurfaceTexture surfaceTexture) {
                return false;
            }

            @Override
            public void onSurfaceTextureUpdated(SurfaceTexture surfaceTexture) {

            }
        });
    }

    void backToMain(int result_code) {
        // 返回到MainActivity
        if (!isRecording) {
            infoLog("back pressed");
            setResult(result_code);
            finish();
        }
    }

    void startRecord() {
        if (null == cameraDevice || !cameraPreview.isAvailable() || null == previewSize) {
            infoLog("" + (null == cameraDevice));
            infoLog("can't record");
        } else {
            try {
                // 没有开始 => 开始录制
                isRecording = true;
                btnStart.setText("Stop");

                /* 开始录制 */
                setUpMediaRecorder();
                final CaptureRequest.Builder recordBuilder = cameraDevice.createCaptureRequest(CameraDevice.TEMPLATE_RECORD);
                List<Surface> surfaces = new ArrayList<>();

                /* 预览的surface TODO 会和非录制时段的预览重复, 需要关掉之前的进程 */
                SurfaceTexture texture = cameraPreview.getSurfaceTexture();
                assert texture != null;
                Surface previewSurface = new Surface(texture);
                surfaces.add(previewSurface);
                recordBuilder.addTarget(previewSurface);

                /* mediaRecorder的surface */
                Surface recorderSurface = mediaRecorder.getSurface();
                recordBuilder.addTarget(recorderSurface);
                surfaces.add(recorderSurface);

                /* TODO 更新UI */
                cameraDevice.createCaptureSession(surfaces, new CameraCaptureSession.StateCallback() {
                    @Override
                    public void onConfigured(@NonNull CameraCaptureSession cameraCaptureSession) {
                        if (null == cameraDevice) {
                            infoLog("not camera");
                            return;
                        }
                        try {
                            /* 实时更新预览 */
                            previewSession = cameraCaptureSession;
                            recordBuilder.set(CaptureRequest.CONTROL_MODE, CameraMetadata.CONTROL_MODE_AUTO);
                            previewSession.setRepeatingRequest(recordBuilder.build(), null, backgroundHandler);

                            mediaRecorder.start();
                        } catch (CameraAccessException e) {
                            infoError(e);
                        }
                    }

                    @Override
                    public void onConfigureFailed(@NonNull CameraCaptureSession cameraCaptureSession) {
                        /* TODO 报错 */
                    }
                }, backgroundHandler);
            } catch (Exception e) {
                infoError(e);
            }
        }
    }

    void stopRecord() {
        try {
            // 正在录制 => 停止录制
            isRecording = false;
            btnStart.setText("Start");

            /* 停止录制 */
            if (mediaRecorder != null) {
                mediaRecorder.stop();
                mediaRecorder.reset();
                mediaRecorder.release();
            }

            /* TODO 保存视频 */
            if (videoFile.exists()) {
                videoFile.delete();
            }

            /* TODO 停止预览 */
            try {
                if (previewSession != null) {
                    infoLog("stop preview");
                    previewSession.stopRepeating();
                    previewSession.abortCaptures();
                    previewSession.close();
                    previewSession = null;
                }
                /* 重新开始预览 */
                createCameraPreview();
            } catch (Exception e) {
                infoError(e);
            }
        } catch (Exception e) {
            infoError(e);
        } finally {
            mediaRecorder = null;
        }
    }

    void setUpMediaRecorder() {
        try {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
                /* 初始化录制 */
                if (mediaRecorder == null) {
                    mediaRecorder = new MediaRecorder();
                }

                /* 设置输出参数 */
                mediaRecorder.setVideoSource(MediaRecorder.VideoSource.SURFACE);
                mediaRecorder.setOutputFormat(MediaRecorder.OutputFormat.MPEG_4);

                /* 设置视频输出文件 */
                videoFile = new File(appPath, "video.mp4");
                mediaRecorder.setOutputFile(videoFile);
                mediaRecorder.setVideoFrameRate(30);// fps

                /* 设置编码和码率 */
                mediaRecorder.setVideoSize(videoSize.getWidth(), videoSize.getHeight());
                mediaRecorder.setVideoEncoder(MediaRecorder.VideoEncoder.MPEG_4_SP);

                /* 准备录制 */
                mediaRecorder.prepare();
            } else {
                infoLog("can't save video");
            }
        } catch (Exception e) {
            infoError(e);
        }
    }


    @SuppressLint("MissingPermission")
    void openCamera(int width, int height) {
        cameraManager = (CameraManager) getSystemService(Context.CAMERA_SERVICE);
        try {
            cameraId = cameraManager.getCameraIdList()[CameraCharacteristics.LENS_FACING_FRONT];// 后置摄像头

            setUpCameraOutputs(width, height);

            cameraManager.openCamera(cameraId, deviceCallback, null);
        } catch (CameraAccessException e) {
            infoError(e);
        }
    }

    void setUpCameraOutputs(int width, int height) {
        try {
            CameraCharacteristics characteristics = cameraManager.getCameraCharacteristics(cameraId);
            StreamConfigurationMap map = characteristics.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);// 管理camera的输出格式和尺寸

            if (null == map) {
                return;
            }

            /* 手机旋转处理 */
            boolean swappedDimensions = true;
            int rotation = getDisplay().getRotation();
            int mSensorOrientation = characteristics.get(CameraCharacteristics.SENSOR_ORIENTATION);
            switch (rotation) {
                case Surface.ROTATION_0:
                case Surface.ROTATION_180:
                    if (mSensorOrientation == 90 || mSensorOrientation == 270) {
                        swappedDimensions = true;
                    }
                    break;
                case Surface.ROTATION_90:
                case Surface.ROTATION_270:
                    if (mSensorOrientation == 0 || mSensorOrientation == 180) {
                        swappedDimensions = true;
                    }
                    break;
                default:
                    infoLog("invalid rotation");
            }

            /* 设置尺寸 */
            int rotatedPreviewWidth = width;
            int rotatedPreviewHeight = height;
            if (swappedDimensions) {
                rotatedPreviewWidth = height;
                rotatedPreviewHeight = width;
            }
            previewSize = getPreviewSize(map.getOutputSizes(SurfaceTexture.class), rotatedPreviewWidth, rotatedPreviewHeight);
            videoSize = getPreviewSize(map.getOutputSizes(MediaRecorder.class), rotatedPreviewWidth, rotatedPreviewHeight);
        } catch (Exception e) {
            infoError(e);
        }
    }

    void createCameraPreview() throws Exception {
        SurfaceTexture surfaceTexture = cameraPreview.getSurfaceTexture();
        assert surfaceTexture != null;

        surfaceTexture.setDefaultBufferSize(previewSize.getWidth(), previewSize.getHeight());
        Surface previewSurface = new Surface(surfaceTexture);
        /* 该builder用于非录像时段的预览 */
        mPreviewRequestBuilder = cameraDevice.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW);
        mPreviewRequestBuilder.addTarget(previewSurface);
        /* 用于预览 */
        cameraDevice.createCaptureSession(Arrays.asList(previewSurface), previewCallback, null);
    }

    CameraDevice.StateCallback deviceCallback = new CameraDevice.StateCallback() {
        // 打开相机后调用
        @Override
        public void onOpened(@NonNull CameraDevice camera) {
            cameraDevice = camera;// 获取camera对象
            try {
                createCameraPreview();
            } catch (Exception e) {
                infoError(e);
                camera.close();
                cameraDevice = null;
            }
        }

        @Override
        public void onDisconnected(@NonNull CameraDevice camera) {
            camera.close();
            cameraDevice = null;
        }

        @Override
        public void onError(@NonNull CameraDevice camera, int i) {
            camera.close();
            cameraDevice = null;
        }
    };

    CameraCaptureSession.StateCallback previewCallback = new CameraCaptureSession.StateCallback() {
        @Override
        public void onConfigured(@NonNull CameraCaptureSession session) {
            if (null == cameraDevice) {
                return;
            }
            previewSession = session;
            try {
                previewSession.setRepeatingRequest(mPreviewRequestBuilder.build(), null, backgroundHandler);
            } catch (Exception e) {
                infoError(e);
            }
        }

        @Override
        public void onConfigureFailed(@NonNull CameraCaptureSession cameraCaptureSession) {

        }
    };

    SensorEventListener mSensorEventListener = new SensorEventListener() {
        @Override
        public void onSensorChanged(SensorEvent sensorEvent) {
            if (sensorEvent.sensor.getType() == Sensor.TYPE_GRAVITY) {
                // 重力
            } else if (sensorEvent.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR) {
                // 旋转
            }
        }

        @Override
        public void onAccuracyChanged(Sensor sensor, int i) {

        }
    };

    void unregisterSensor() {
        mSensorManager.unregisterListener(mSensorEventListener);
    }

    Size getPreviewSize(Size[] choices, int width, int height) {
        // 选择合适的尺寸
        List<Size> sameratios = new ArrayList<>();
        List<Size> unequalratios = new ArrayList<>();

        for (Size s : choices) {
            if (s.getWidth() * 1.00d / width == s.getHeight() * 1.00d / height) {
                sameratios.add(s);
            }
        }

        if (!sameratios.isEmpty()) {
            return Collections.max(sameratios, new CompareSizesByArea());
        } else {
            double viewratio = width * 1.00d / height;
            double suitratio = 0.00d;

            for (Size s : choices) {
                double sratio = s.getWidth() * 1.00d / s.getHeight();
                double current = Math.abs(sratio - viewratio);
                if (s.getHeight() > 500 && current <= ratio) {
                    suitratio = sratio;
                }
            }

            for (Size s : choices) {
                double sratio = s.getWidth() * 1.00d / s.getHeight();
                if (sratio == suitratio) {
                    unequalratios.add(s);
                }
            }
            if (!unequalratios.isEmpty()) {
                return Collections.max(unequalratios, new CompareSizesByArea());
            }

        }

        return null;
    }

    static class CompareSizesByArea implements Comparator<Size> {
        @Override
        public int compare(Size lhs, Size rhs) {
            return Long.signum((long) lhs.getWidth() * lhs.getHeight() - (long) rhs.getWidth() * rhs.getHeight());
        }

    }
}
