package AAAatrox.panorama;

import android.Manifest;
import android.app.Activity;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.graphics.ImageFormat;
import android.graphics.Matrix;
import android.graphics.SurfaceTexture;
import android.graphics.drawable.ColorDrawable;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.media.ImageReader;
import android.media.MediaRecorder;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.util.Size;
import android.view.LayoutInflater;
import android.view.Surface;
import android.view.SurfaceView;
import android.view.TextureView;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;

import androidx.annotation.NonNull;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;
import androidx.fragment.app.DialogFragment;
import androidx.fragment.app.FragmentManager;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;

import static AAAatrox.panorama.MainActivity.PERMISSION_CAMERA_REQUEST_CODE;
import static AAAatrox.panorama.MainActivity.appPath;

public class CustomCamera1 extends Activity {
    static public void infoLog(String log) {
        Log.i("fuck", log);
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
        // 权限处理
        if (requestCode == PERMISSION_CAMERA_REQUEST_CODE) {
            if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                // 允许
            } else {
                // 拒绝
                infoLog("camera not permitted");
            }
        }
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.custom_camera_1);
        infoLog("create camera");

        initCamera();
        initUI();
    }

    @Override
    public void onBackPressed() {
        backToMain(0);// TODO 禁止返回
    }

    @Override
    public void onDestroy() {
        unregisterSensor();
        infoLog("destroy camera");

        super.onDestroy();
    }

    /* UI组件 */
    Button btnStart, btnConfirm, btnCancel, btnDebug;
    SurfaceView cameraPreview;
    /* 控制 */
    boolean isRecording;

    /* 录制 */
    MediaRecorder mediaRecorder;
    /* 相机相关 */
    CameraDevice mCameraDevice;// 摄像头设备,(参数:预览尺寸,拍照尺寸等)
    CameraCaptureSession mCameraCaptureSession;// 相机捕获会话,用于处理拍照和预览的工作
    CaptureRequest.Builder captureRequestBuilder;// 捕获请求,定义输出缓冲区及显示界面(TextureView或SurfaceView)
    /* 预览 */
    Size previewSize;// 在textureView预览的尺寸
    Size captureSize;// 拍摄的尺寸
    ImageReader mImageReader;
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
                startPreview(view);
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

        /* 初始化相机预览 TODO */
        cameraPreview = findViewById(R.id.camera_preview);
        Matrix matrix = cameraPreview.getMatrix();
        infoLog(matrix + "");
        matrix.preRotate(90);
        infoLog(matrix + "");
    }

    void backToMain(int result_code) {
        // 返回到MainActivity
        if (!isRecording) {
            infoLog("back pressed");
            setResult(result_code);
            finish();
        }
    }

    void startPreview(View view) {
        if (!isRecording) {
            // 没有开始 => 开始录制
            isRecording = true;
            btnStart.setText("Stop");

            /* 初始化录制 */
            if (mediaRecorder == null) {
                mediaRecorder = new MediaRecorder();
            }
            mediaRecorder.reset();
            mediaRecorder.setOnErrorListener(new MediaRecorder.OnErrorListener() {
                @Override
                public void onError(MediaRecorder mediaRecorder, int i, int i1) {
                    infoLog("mediaRecorder error");
                    mediaRecorder.reset();
                    mediaRecorder = null;
                }
            });

            /* 设置参数 */
            mediaRecorder.setVideoSource(MediaRecorder.VideoSource.CAMERA);
            mediaRecorder.setOutputFormat(MediaRecorder.OutputFormat.MPEG_4);
            mediaRecorder.setVideoEncoder(MediaRecorder.VideoEncoder.MPEG_4_SP);
            mediaRecorder.setPreviewDisplay(cameraPreview.getHolder().getSurface());
            mediaRecorder.setVideoFrameRate(30);// fps

            /* 设置分辨率 TODO */
//            mediaRecorder.setVideoSize(1920, 1080);

            /* 设置视频输出文件 */
            File videoFile = new File(appPath, "video.mp4");
            if (videoFile.exists()) {
                videoFile.delete();
            }
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
                mediaRecorder.setOutputFile(videoFile);
            } else {
                infoLog("can't save video");
            }

            /* 开始录制 */
            try {
                mediaRecorder.prepare();
                mediaRecorder.start();
            } catch (IOException e) {
                e.printStackTrace();
            }
        } else {
            // 正在录制 => 停止录制
            isRecording = false;
            btnStart.setText("Start");

            /* 停止录制 */
            if (mediaRecorder != null) {
                mediaRecorder.stop();
            }
        }
    }

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
}
