package AAAatrox.panorama;

import android.app.Activity;
import android.content.Context;
import android.content.DialogInterface;
import android.graphics.drawable.ColorDrawable;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;

import androidx.fragment.app.DialogFragment;
import androidx.fragment.app.FragmentManager;

public class CustomCamera1 extends DialogFragment {
    @Override
    public void show(FragmentManager fragmentManager, String tag) {
        super.show(fragmentManager, tag);
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setCancelable(false);
        setStyle(STYLE_NO_FRAME, android.R.style.Theme);// 无背景
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        View view = inflater.inflate(R.layout.custom_camera_1, container);
        getDialog().getWindow().setBackgroundDrawable(new ColorDrawable(0x00000000));// 背景透明

        initCamera();
        initUI(view);

        return view;
    }

    @Override
    public void onDismiss(final DialogInterface dialogInterface) {
        super.onDismiss(dialogInterface);

        unregisterSensor();// 取消传感器的监听

        Activity activity = getActivity();
        if (activity instanceof DialogInterface.OnDismissListener) {
            ((DialogInterface.OnDismissListener) activity).onDismiss(dialogInterface);
        }
    }

    /* UI组件 */
    Button btnStart, btnConfirm, btnCancel, btnDebug;
    /* 传感器 */
    SensorManager mSensorManager;
    Sensor mGravity;// 重力传感器
    Sensor mRotation;// 旋转矢量传感器
    /* 公共成员 */
    public int dismiss_result;

    void initCamera() {
        dismiss_result = 0;

        /* 初始化传感器 */
        mSensorManager = (SensorManager) getActivity().getSystemService(Context.SENSOR_SERVICE);
        mGravity  = mSensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY);// 重力传感器
        mRotation = mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);// 旋转矢量传感器
        /* 注册传感器的监听 */
        mSensorManager.registerListener(mSensorEventListener, mRotation, SensorManager.SENSOR_DELAY_FASTEST);        mSensorManager.registerListener(mSensorEventListener, mRotation, SensorManager.SENSOR_DELAY_FASTEST);
        mSensorManager.registerListener(mSensorEventListener, mGravity, SensorManager.SENSOR_DELAY_FASTEST);
    }

    void initUI(View view) {
        btnStart   = view.findViewById(R.id.start_button);
        btnConfirm = view.findViewById(R.id.confirm_button);
        btnCancel  = view.findViewById(R.id.cancel_button);
        btnDebug   = view.findViewById(R.id.cancel_button);

        btnConfirm.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                dismiss_result = 0;
                dismiss();
            }
        });

        btnCancel.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                dismiss_result = 1;
                dismiss();
            }
        });

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
