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
import android.util.SparseIntArray;
import android.view.Surface;
import android.view.TextureView;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

import androidx.annotation.NonNull;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.PrintStream;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.Date;
import java.util.List;
import java.util.TimeZone;

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
    static final int SENSOR_ORIENTATION_DEFAULT_DEGREES = 90;
    static final int SENSOR_ORIENTATION_INVERSE_DEGREES = 270;

    static final SparseIntArray DEFAULT_ORIENTATIONS = new SparseIntArray();
    static final SparseIntArray INVERSE_ORIENTATIONS = new SparseIntArray();

    static {
        DEFAULT_ORIENTATIONS.append(Surface.ROTATION_0, 90);
        DEFAULT_ORIENTATIONS.append(Surface.ROTATION_90, 0);
        DEFAULT_ORIENTATIONS.append(Surface.ROTATION_180, 270);
        DEFAULT_ORIENTATIONS.append(Surface.ROTATION_270, 180);
    }

    static {
        INVERSE_ORIENTATIONS.append(Surface.ROTATION_0, 270);
        INVERSE_ORIENTATIONS.append(Surface.ROTATION_90, 180);
        INVERSE_ORIENTATIONS.append(Surface.ROTATION_180, 90);
        INVERSE_ORIENTATIONS.append(Surface.ROTATION_270, 0);
    }

    double ratio = 0.25d;

    /* UI组件 */
    Button btnStart, btnConfirm, btnCancel, btnDebug;
    TextView text_1, text_2, text_3, text_4;
    TextureView cameraPreview;

    /* 控制 */
    boolean isRecording;
    /* 录像计时 */
    long recordStartTime;
    long recordTimeStamp;

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

    /* 手机旋转角度 */
    int screenRotation;
    int mSensorOrientation;

    /* 传感器角度 */
    long sensorTime;
    double tangent;// 切面角度
    double longitude;
    double latitude;
    /* 传感器 */
    SensorManager mSensorManager;
    Sensor mGravity;// 重力传感器
    Sensor mRotation;// 旋转矢量传感器

    void backToMain(int result_code) {
        // 返回到MainActivity
        if (!isRecording) {
            infoLog("back pressed");
            setResult(result_code);
            finish();
        }
    }

    void initCamera() {
        /* 初始化变量*/
        isRecording = false;
        sensorTime = 0;

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

        /* 初始化文本框 */
        text_1 = findViewById(R.id.text1_1);
        text_2 = findViewById(R.id.text1_2);
        text_3 = findViewById(R.id.text1_3);
        text_4 = findViewById(R.id.text1_4);

        text_4.setText("NAN");
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

    void startRecord() {
        if (null == cameraDevice || !cameraPreview.isAvailable() || null == previewSize) {
            infoLog("" + (null == cameraDevice));
            infoLog("can't record");
        } else {
            try {
                // 没有开始 => 开始录制
                isRecording = true;
                btnStart.setText("Stop");

                /* 初始化 */
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

                /* 更新UI */
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

                            /* 开始录像 */
                            mediaRecorder.start();
                            recordStartTime = System.currentTimeMillis();
                            recordTimeStamp = recordStartTime;
                        } catch (CameraAccessException e) {
                            infoError(e);
                        }
                    }

                    @Override
                    public void onConfigureFailed(@NonNull CameraCaptureSession cameraCaptureSession) { }
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
                infoLog("stop record");
                mediaRecorder.stop();
                mediaRecorder.reset();
                mediaRecorder.release();
            }

            try {
                /* 重新开始预览 */
                assert cameraPreview.isAvailable();
                openCamera(cameraPreview.getWidth(), cameraPreview.getHeight());
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
                if (videoFile.exists()) {
                    videoFile.delete();
                }
                mediaRecorder.setOutputFile(videoFile);
                mediaRecorder.setVideoEncodingBitRate(30 * 1000000);
                mediaRecorder.setVideoFrameRate(30);// fps

                /* 设置编码和码率 */
                mediaRecorder.setVideoSize(videoSize.getWidth(), videoSize.getHeight());
                mediaRecorder.setVideoEncoder(MediaRecorder.VideoEncoder.H264);// 不能使用mp4
                switch (mSensorOrientation) {
                    case SENSOR_ORIENTATION_DEFAULT_DEGREES:
                        mediaRecorder.setOrientationHint(DEFAULT_ORIENTATIONS.get(screenRotation));
                        break;
                    case SENSOR_ORIENTATION_INVERSE_DEGREES:
                        mediaRecorder.setOrientationHint(INVERSE_ORIENTATIONS.get(screenRotation));
                        break;
                }

                /* 准备录制 */
                mediaRecorder.prepare();
            } else {
                infoLog("can't save video");
            }
        } catch (Exception e) {
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
            screenRotation = getDisplay().getRotation();
            mSensorOrientation = characteristics.get(CameraCharacteristics.SENSOR_ORIENTATION);
            switch (screenRotation) {
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
                    infoLog("invalid screenRotation");
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
            infoLog("video size: " + videoSize);
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
                double x = sensorEvent.values[0];
                double y = sensorEvent.values[1];
                double z = sensorEvent.values[2];
                double gravity = Math.sqrt(x*x + y*y + z*z);
                tangent = Math.atan(x / y);// tan = x / y
                if (tangent > 0 && x < 0) {
                    tangent -= Math.PI;
                } else if (tangent < 0 && x > 0) {
                    tangent += Math.PI;
                }
                latitude = Math.acos(z / gravity) - Math.PI / 2;// cos = z / g

                long curTime = System.currentTimeMillis();
                long timeInterval = curTime - sensorTime;
                if (timeInterval > 500) {
                    sensorTime = curTime;
                    text_1.setText("水平: " + (int) Math.toDegrees(longitude));
                    text_2.setText("仰角: " + (int) Math.toDegrees(latitude));
                    text_3.setText("切面: " + (int) Math.toDegrees(tangent));
                }

                /* 更新计时 */
                if (isRecording) {
                    timeInterval = curTime - recordTimeStamp;
                    if (timeInterval > 100) {
                        recordTimeStamp = curTime;

                        /* 将毫秒转换成日期 */
                        SimpleDateFormat simpleDateFormat = new SimpleDateFormat("mm:ss:SSS");
                        simpleDateFormat.setTimeZone(TimeZone.getTimeZone("GMT+0:00"));

                        Date date = new Date(curTime - recordStartTime);
                        String totalTime = simpleDateFormat.format(date);
                        text_4.setText(totalTime);
                    }
                }
            } else if (sensorEvent.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR) {
                // 旋转
                double x = sensorEvent.values[0];
                double y = sensorEvent.values[1];
                longitude = Math.atan(y / x) * 2 + tangent;// tan = y / x
                if (longitude < -Math.PI) {
                    longitude += 2 * Math.PI;
                } else if (longitude > Math.PI) {
                    longitude -= 2 * Math.PI;
                }
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
