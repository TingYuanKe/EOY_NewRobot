package com.example.weichun.deviceposition;

import android.app.ActivityManager;
import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.os.Environment;
import android.os.Handler;
import android.support.v7.app.AppCompatActivity;
import android.text.InputType;
import android.util.Log;
import android.widget.CompoundButton;
import android.widget.EditText;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;

import android.view.KeyEvent;

import java.io.DataOutputStream;
import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;

import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;
import java.util.GregorianCalendar;
import java.util.List;
import java.util.SimpleTimeZone;


public class MainActivity extends AppCompatActivity {
    //Client socket set
    private ClientSocket client;
    private final int PORT = 8888;
    private boolean connectedAvailable = false;
    private String defaultName = "";
    private String defaultIP = "";
    private String dir_path_arguements = "";
    private File dir_arguements;
    private File file_arguements;

    private Handler handler;

    private SensorManager mSensorManager;
    private Sensor mSensor;
    private Context mContext;

    private DataOutputStream out_acc;
    private DataOutputStream out_arguments;
    private DataInputStream in_arguments;

    private TextView text;
    private ToggleButton btn_record;

    private boolean startRec = false;

    private Calendar calendar;

    private float[] gravityValues = null;
    private float[] magneticValues = null;

    private String rawAcc="";
    private String rawGyro="";
    private boolean gyroReadFlag= false;
    private boolean accReadFlag = false;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        mContext = this.getApplicationContext();
        text = (TextView) findViewById(R.id.log);
        btn_record = (ToggleButton) findViewById(R.id.btn_record);
        dir_path_arguements = Environment.getExternalStorageDirectory().getAbsolutePath() + "/AbsAccCollection";
        dir_arguements = new File(dir_path_arguements);
        if (!dir_arguements.exists()) {
            dir_arguements.mkdir();
        }
        //initial default name and IP
        try {
            file_arguements = new File(dir_arguements, "raw_arguements.txt");
            in_arguments = new DataInputStream(new FileInputStream(file_arguements));
            defaultName = in_arguments.readLine();
            defaultIP = in_arguments.readLine();
        } catch (Exception e) {
            e.printStackTrace();
        }


        ConnectCheck();
        inputName();

        mSensorManager = (SensorManager )getSystemService(Context.SENSOR_SERVICE);
        List<Sensor> deviceSensors = mSensorManager.getSensorList(Sensor.TYPE_ALL);

        // Show all supported sensor
        for (int i = 0; i < deviceSensors.size(); i++) {
            Log.d("[SO]", deviceSensors.get(i).getName());
        }
        /**
         *              Set all sensor
         */
        if ((mSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION)) != null) {
            mSensorManager.registerListener(mSensorListener, mSensor, 10000);
        } else {
            Toast.makeText(mContext, "ACCELEROMETER is not supported!", Toast.LENGTH_SHORT).show();
        }

        if ((mSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY)) != null) {
            mSensorManager.registerListener(mSensorListener, mSensor, 10000);
        } else {
            Toast.makeText(mContext, "GRAVITY is not supported!", Toast.LENGTH_SHORT).show();
        }

        if ((mSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD)) != null) {
            mSensorManager.registerListener(mSensorListener, mSensor, 10000);
        } else {
            Toast.makeText(mContext, "MAGNETOMETER is not supported!", Toast.LENGTH_SHORT).show();
        }
        if((mSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE))!=null){
            mSensorManager.registerListener(mSensorListener, mSensor, 10000);
        }else{
            Toast.makeText(mContext, "GYROSCOPE is not supported!", Toast.LENGTH_SHORT).show();
        }
  /* get time */

    }

    private SensorEventListener mSensorListener = new SensorEventListener() {
        public final void onSensorChanged(SensorEvent event) {

            String time;

            if (startRec) {
                if((event.sensor.getType() == Sensor.TYPE_GYROSCOPE)){
                    double[] deviceGyroscope = new double[4];
                    deviceGyroscope[0] = event.values[0];
                    deviceGyroscope[1] = event.values[1];
                    deviceGyroscope[2] = event.values[2];
                    deviceGyroscope[3] = 0;

                    rawGyro= String.valueOf(deviceGyroscope[0]) + "," + String.valueOf(deviceGyroscope[1]) + "," + String.valueOf(deviceGyroscope[2]);
                    Log.d("Gyro",rawGyro);
                    gyroReadFlag=true;

                }
                if ((gravityValues != null) && (magneticValues != null)
                        && (event.sensor.getType() == Sensor.TYPE_LINEAR_ACCELERATION) ) {
                    try {

                        float[] deviceRelativeAcceleration = new float[4];
                        deviceRelativeAcceleration[0] = event.values[0];
                        deviceRelativeAcceleration[1] = event.values[1];
                        deviceRelativeAcceleration[2] = event.values[2];
                        deviceRelativeAcceleration[3] = 0;

//                        Log.d("sensors",event.sensor.getName());
                        // Change the device relative acceleration values to earth relative values
                        // X axis -> East
                        // Y axis -> North Pole
                        // Z axis -> Sky

                        //calibrate to earth coordination
                        float[] R = new float[16], I = new float[16], earthAcc = new float[16];
                        SensorManager.getRotationMatrix(R, I, gravityValues, magneticValues);
                        float[] inv = new float[16];
                        android.opengl.Matrix.invertM(inv, 0, R, 0);
                        android.opengl.Matrix.multiplyMV(earthAcc, 0, inv, 0, deviceRelativeAcceleration, 0);
                        //Log.d("Acceleration", "Values: (" + earthAcc[0] + ", " + earthAcc[1] + ", " + earthAcc[2] + ")");
                        //text.append("Values: (" + earthAcc[0] + ", " + earthAcc[1] + ", " + earthAcc[2] + ")\n");

                        SimpleDateFormat formatter = new SimpleDateFormat("HH:mm:ss:SSS");
                        Date curDate = new Date(System.currentTimeMillis()); // 獲取當前時間
                        time = formatter.format(curDate);

                        rawAcc = String.valueOf(earthAcc[0]) + "," + String.valueOf(earthAcc[1]) + "," + String.valueOf(earthAcc[2]) + ",";
                        accReadFlag=true;

                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                } else if (event.sensor.getType() == Sensor.TYPE_GRAVITY) {
                    gravityValues = event.values;
                } else if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
                    magneticValues = event.values;
                }
                if(accReadFlag && gyroReadFlag){
                    SimpleDateFormat formatter = new SimpleDateFormat("HH:mm:ss:SSS");
                    Date curDate = new Date(System.currentTimeMillis()); // 獲取當前時間
                    time = formatter.format(curDate);

                    String raw = rawAcc +rawGyro +"," +time +"\n";
                    client.sendDataString(raw);
                    Log.d("raw",raw+" ");
                    Log.d("acc",rawAcc+" ");
                    Log.d("gyro",rawGyro+" ");
                    gyroReadFlag= false;
                    accReadFlag =false;
                }
            }
        }

        @Override
        public final void onAccuracyChanged(Sensor sensor, int accuracy) {
            // Do something here if sensor accuracy changes.
        }
    };
    private void ConnectCheck(){
        //建立一個POP OUT視窗要求使用者輸入IP Address
        final AlertDialog.Builder builder = new AlertDialog.Builder(this,R.style.AlertDialogCustom);
        builder.setCancelable(false);
        builder.setTitle("請輸入Server IP位置");

        // 設定開始畫面Input
        final EditText input = new EditText(this);
        input.setInputType(InputType.TYPE_CLASS_PHONE);
        input.setText(defaultIP);
        // 建立開始畫面Connect Button
        this.runOnUiThread(new Runnable() {
            public void run() {
                builder.setView(input)
                        .setPositiveButton("Connect", new DialogInterface.OnClickListener() {
                            @Override
                            public void onClick(DialogInterface dialog, int which) {
                            defaultIP = input.getText().toString();
                            try {
                                out_arguments = new DataOutputStream(new FileOutputStream(file_arguements, true));
                                String ipTemp = (defaultIP + "\n");
                                out_arguments.write(ipTemp.getBytes());
                            } catch (Exception e) {
                                e.printStackTrace();
                            }
                            //建立與client端地連線
                            connectClient();
                            //check if connected
                            if (!connectedAvailable ) //returns true if internet available
                            {
                                Toast.makeText(MainActivity.this, "唉呦  好像沒有連上喔", Toast.LENGTH_LONG).show();
                                ConnectCheck();
                            } else {
                                Toast.makeText(MainActivity.this, "Connected showed from MainActivity!!", Toast.LENGTH_LONG).show();
                            }
                            }
                        })
                        .create().setOnShowListener(new DialogInterface.OnShowListener() {
                    @Override
                    public void onShow(DialogInterface dialog) {
                    }
                });

                    builder.show().getWindow().setLayout(800, 600);
            }
        });
    }
    private void inputName(){
        //建立一個POP OUT視窗要求使用者輸入User name
        final AlertDialog.Builder builder = new AlertDialog.Builder(this,R.style.AlertDialogCustom);
        builder.setCancelable(false);
        builder.setTitle("Enter your name:");

        // 設定開始畫面Input
        final EditText input = new EditText(this);
        input.setInputType(InputType.TYPE_CLASS_TEXT);
        input.setText(defaultName);
        // 建立開始畫面name Button
        this.runOnUiThread(new Runnable() {
            public void run() {
                builder.setView(input)
                        .setPositiveButton("Confirm", new DialogInterface.OnClickListener() {
                            @Override
                            public void onClick(DialogInterface dialog, int which) {
                                defaultName = input.getText().toString();
                                try {
                                    out_arguments = new DataOutputStream(new FileOutputStream(file_arguements, false));
                                    String nameTemp = (defaultName + "\n");
                                    out_arguments.write(nameTemp.getBytes());
                                } catch (Exception e) {
                                    e.printStackTrace();
                                }
                            }
                        })
                        .create().setOnShowListener(new DialogInterface.OnShowListener() {
                    @Override
                    public void onShow(DialogInterface dialog) {
                    }
                });
                builder.show().getWindow().setLayout(800, 600);
            }
        });
    }
    private void connectClient() {
        //新增一個ClientSocket為client
        setClient(new ClientSocket(defaultIP, PORT, defaultName));
        //將client連  線設定為背景執行
        getClient().execute();
        connectedAvailable =true;

    }

    private void setClient(ClientSocket client) {
        this.client = client;
    }

    private ClientSocket getClient() {
        return client;
    }

    @Override
    protected void onResume() {
        super.onResume();
        mSensorManager.registerListener(mSensorListener, mSensor, SensorManager.SENSOR_DELAY_NORMAL);
        SimpleTimeZone pdt = new SimpleTimeZone(8 * 60 * 60 * 1000, "Asia/Taipei");
        calendar = new GregorianCalendar(pdt);
        handler = new Handler();

        btn_record.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if (isChecked) {
                    text.setText("");
                    client.checkSocketConnection();
                    startRecording();
                } else {
                    endRecording();
                    connectClient();

                }
            }
        });
    }

    private void startRecording(){
        try {
            //initial name
            String nameTemp = (defaultName + "\n");
            client.sendDataString(nameTemp);
            System.out.println("name has been sent");
        } catch (Exception e) {
            e.printStackTrace();
        }

        Date trialTime = new Date();
        calendar.setTime(trialTime);
        text.append(" Start: " + calendar.get(Calendar.HOUR_OF_DAY) + ":" +
                calendar.get(Calendar.MINUTE) + ":" +
                calendar.get(Calendar.SECOND));

        startRec = true;
    }
    private void endRecording(){
        startRec = false;
        client.disconnect();

        Date trialTime = new Date();
        calendar.setTime(trialTime);
        text.append(" End: " + calendar.get(Calendar.HOUR_OF_DAY) + ":" +
                calendar.get(Calendar.MINUTE) + ":" +
                calendar.get(Calendar.SECOND));


    }

    @Override
    public boolean dispatchKeyEvent(KeyEvent event) {
        if (event.getKeyCode() == KeyEvent.KEYCODE_POWER) {
            Log.i("", "Dispath event power");
            Intent closeDialog = new Intent(Intent.ACTION_CLOSE_SYSTEM_DIALOGS);
            sendBroadcast(closeDialog);
            return true;
        }

        return super.dispatchKeyEvent(event);
    }

    @Override
    protected void onPause() {
        super.onPause();

    }
}
