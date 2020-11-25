package AAAatrox.panorama;

import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import android.Manifest;
import android.content.DialogInterface;
import android.content.pm.PackageManager;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

import org.opencv.android.OpenCVLoader;

public class MainActivity extends AppCompatActivity implements DialogInterface.OnDismissListener {

    static public final int PERMISSION_CAMERA_REQUEST_CODE = 0x00000012;// 相机权限的 request code
    static public String appPath;

    static {
        // 在程序执行前需要加载opencv的java库
        if (!OpenCVLoader.initDebug()) {
            infoLog("opencv init failed");
        } else {
            infoLog("opencv init succeed");
        }
        // 在程序执行前需要加载自己的cpp库
        System.loadLibrary("native-lib");
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
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        initApp();
        initUI();
    }

    @Override
    public void onDismiss(DialogInterface dialogInterface) {
        if (customCamera.dismiss_result == 0) {
            // 保存视频
        } else if (customCamera.dismiss_result == 1) {
            // 取消保存
        }
    }


    /* UI组件 */
    Button btnCamera, btnSave;
    /* 相机 */
    CustomCamera1 customCamera = new CustomCamera1();

    void initApp() {
        appPath = getExternalFilesDir("").getAbsolutePath();

        // 只适用 SDK > 23
        int hasCameraPermission = ContextCompat.checkSelfPermission(getApplication(), Manifest.permission.CAMERA);
        if (hasCameraPermission == PackageManager.PERMISSION_GRANTED) {
            // 有调用相机权限
            infoLog("camera is ready");
        } else {
            ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.CAMERA}, PERMISSION_CAMERA_REQUEST_CODE);
        }
    }

    void initUI() {
        btnCamera = findViewById(R.id.camera_button);
        btnSave   = findViewById(R.id.save_button);

        btnCamera.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                customCamera.show(getSupportFragmentManager(), "camera");
            }
        });
    }

    /**
     * A native method that is implemented by the 'native-lib' native library,
     * which is packaged with this application.
     */
    public native String stringFromJNI();

    static public void infoLog(String log) {
        Log.i("fuck", log);
    }
}
