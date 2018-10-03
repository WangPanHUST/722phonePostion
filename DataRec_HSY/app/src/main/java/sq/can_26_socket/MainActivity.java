package sq.can_26_socket;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.InputStreamReader;
import java.net.SocketException;
import java.text.DecimalFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.SyncStats;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.net.wifi.WifiInfo;
import android.net.wifi.ScanResult;
import android.net.wifi.WifiManager;
import android.os.Environment;
import android.os.Handler;
import android.os.Message;
//import android.os.StrictMode;
import android.os.SystemClock;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.text.Html;
import android.text.format.DateUtils;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.SeekBar;
import android.widget.TextView;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.Socket;
import java.util.Locale;

import jeigen.DenseMatrix;

import static android.hardware.Sensor.TYPE_ACCELEROMETER;
import static android.hardware.Sensor.TYPE_MAGNETIC_FIELD;
import static android.hardware.Sensor.TYPE_ROTATION_VECTOR;
import static android.hardware.Sensor.TYPE_STEP_COUNTER;
import static android.hardware.Sensor.TYPE_STEP_DETECTOR;

public class MainActivity extends AppCompatActivity {

    private Button btn1,btn2;
    private EditText et;
    private TextView tv;
    private TextView DispInfo;

    private int RssiNum;
    private String dataString = null;
    private String IP;
    private String AP_0Name;
    private String AP_1Name;
    private String [] Data;

    private WifiManager wifiManager;
    private WifiInfo connectedWifiInfo;
    public  List<ScanResult> list;

    private final String TAG = "RSSI";
    private IntentFilter RssiIntentFilter;
    private BroadcastReceiver RssiIntentReceiver;

    // sensor listener
    mySensorListener mSensorListener;
    SensorManager mSensorManager;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // text view for display rssi
        DispInfo = (TextView) findViewById(R.id.DispInfo);


        // sensor manager algorithm
        mSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        mSensorListener = new mySensorListener(mSensorManager, DispInfo);

        //注册两个广播，第二个用于判断wifi状态
        RssiIntentFilter = new IntentFilter();
        RssiIntentFilter.addAction(WifiManager.SCAN_RESULTS_AVAILABLE_ACTION);
        RssiIntentFilter.addAction(String.valueOf(WifiManager.WIFI_STATE_CHANGED_ACTION ));

        RssiIntentReceiver = new RssiIntentReceiver();
        registerReceiver(RssiIntentReceiver, RssiIntentFilter);

        btn1 = (Button) findViewById(R.id.btn1);
        btn2 = (Button) findViewById(R.id.btn2);
        et = (EditText) findViewById(R.id.et_send);
        tv = (TextView) findViewById(R.id.tv_js);

        btn1.setEnabled(false);
        et.setEnabled(false);
        wifiManager = (WifiManager) getApplicationContext().getSystemService(Context.WIFI_SERVICE);

        Intent intent = getIntent();
        Data = intent.getStringArrayExtra("Data");
        IP = Data[0];
        AP_0Name = Data[1];
        AP_1Name = Data[2];

        //监听发送按钮
        btn1.setOnClickListener(new View.OnClickListener() {
            @Override
                public void onClick(View v) {

                    new Thread(new SendRSSI()).start();//使用new方法，每次都会新建一个Thread

                    tv.setText("Data is being uploaded, please wait...");
                    btn1.setEnabled(false);
                    btn2.setEnabled(true);
                    et.setEnabled(false);
                }
        });

        //监听开始按钮，并设置位置信息
        btn2.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                et.setEnabled(true);
                btn1.setEnabled(true);
                btn2.setEnabled(false);
            }
        });
    }

    // 添加OnResume和OnStop，注册和取消Sensor服务，在activity和sensor listener里面都需要注册
    @Override
    protected void onResume() {
        // 监听服务，在activity和Listener里面都要注册，这是一个难点。不要只在一个地方注册。
        // TODO Auto-generated method stub
        super.onResume();
        mSensorListener.mySensorRegister();
    }

    @Override
    protected void onStop() {
        super.onStop();
        mSensorListener.mySensorUnregister();
    }


    //在子线程中发送数据，避免造成界面假死
    private class SendRSSI implements Runnable  {
        @Override
        public void run(){
            try {
                DatagramSocket UDP_Socket = new DatagramSocket();
                InetAddress LocalIP = InetAddress.getByName(IP);//如果出了问题，把IP设置为广播地址试一试，如IP为192.168.1.187，广播地址为192.168.1.255

                for (RssiNum = 0; RssiNum < 2000; RssiNum++) {
                    try {
                        if(et.getText().length()>0){
                            dataString = "1 _" + getHostIp() + "_" + "Location:" + et.getText().toString() + "_" + ScanWifiInfo();
                        }else{
                            dataString = "1 _" + getHostIp() + "_" + "Location:0_" + ScanWifiInfo();
                        }
                        System.out.println(dataString);
                        // receive the distance from server when the RssiNum >= 20

                        // fusion

                        // send the final distance

                        // display the RSSI infomation
                        // runOnUiThread is essential to display
                        runOnUiThread(new Runnable(){
                            @Override
                            public void run() {
                                DispInfo.setText(dataString);
                            }
                        });

                        System.out.println("NO." + RssiNum + ":" + dataString);
                        byte[] Byte_Send_Message = dataString.getBytes();
                        DatagramPacket SendPack;
                        SendPack = new DatagramPacket(Byte_Send_Message,
                                Byte_Send_Message.length,
                                LocalIP,
                                2048);

                        SendPack.setAddress(InetAddress.getByName(IP));
                        SendPack.setPort(2048);
                        UDP_Socket.send(SendPack);
                    } catch (IOException e){e.printStackTrace();}
                    try {Thread.sleep(800);} catch (InterruptedException e){e.printStackTrace();}
                }
                RssiNum = 0;
            }catch(IOException e){ e.printStackTrace();}

            //传输完成后，更新UI，显示传输完毕
            runOnUiThread(new Runnable(){
                @Override
                public void run() {
                    tv.setText(ScanWifiInfo());
                }
            });
        }
    }

    //获取每次发送的RSSI信息
    private String ScanWifiInfo(){
        wifiManager.startScan();
        list = wifiManager.getScanResults();
        System.out.println(list.size());
        String upData = "";
        int i =0;
        for(int j = 0;j < list.size() && i<2;j++) {
            ScanResult scanResult = list.get(j);
            if (AP_0Name.equals(scanResult.SSID)) {
                upData += "AP_0" + ":" + scanResult.SSID + "_RSSI:" + scanResult.level + ";";
                i++;

            } else if (AP_1Name.equals(scanResult.SSID)) {
                upData += "AP_1" + ":" + scanResult.SSID + "_RSSI:" + scanResult.level + ";";
                i++;
            }
        }
        return upData;
    }


//    Thread Send = new Thread(new Runnable(){
//        @Override
//        public void run() {
//            try {
//                //在数据包中指定IP和端口，三个构造方法之一
//                DatagramSocket UDP_Socket = new DatagramSocket();
//                InetAddress LocalIP = InetAddress.getByName(IP);//广播地址设定好，获取服务器主机名
//                for(int i = 0;i < 100;i++){
//                    try {
//                        dataString = "1 _" + getHostIp() + "_" + "Location:" + et.getText().toString() + "_" + linkRssi();
//                        //System.out.println("NO." + RssiNum + ":" + dataString);
//                        byte[] Byte_Send_Message = dataString.getBytes();   // 将字符串转化为字节数组
//                        DatagramPacket SendPack;
//                        SendPack = new DatagramPacket(Byte_Send_Message,
//                                Byte_Send_Message.length,
//                                LocalIP,
//                                2048);
//
//                        SendPack.setAddress(InetAddress.getByName(IP));
//                        SendPack.setPort(2048);
//                        UDP_Socket.send(SendPack);
//                    } catch (IOException e){e.printStackTrace();}
//                }
//            }catch(IOException e){ e.printStackTrace();}
//        }
//    });
//    private String linkRssi(){
//        Runtime mRuntime = Runtime.getRuntime();
//        try {
//            //Process中封装了返回的结果和执行错误的结果
//            Process mProcess = mRuntime.exec("su || iw dev wlan0 link | grep signal");
//            //Process mProcess = mRuntime.exec("su || iw dev wlan0 scan freq 2412");
//            BufferedReader mReader = new BufferedReader(new InputStreamReader(mProcess.getInputStream()));
//            StringBuffer mRespBuff = new StringBuffer();
//            char[] buff = new char[1024];
//            int ch = 0;
//            while ((ch = mReader.read(buff)) != -1) {
//                mRespBuff.append(buff, 0, ch);
//            }
//            mReader.close();
//            String linkResult = mRespBuff.toString();
//            String time1 = new SimpleDateFormat("yyyyMMddHHmmssSSS").format(new Date());
//            System.out.println(linkResult + "#" + time1);
//            return linkResult;
//            //final String rssiMango = getRssi(scanResult) + "##" + time1;
//            //System.out.println(rssiMango);
//        } catch (IOException e) {
//            e.printStackTrace();
//        }
//    }

    //获取IP信息
    private String getHostIp() {
        WifiManager mg = (WifiManager) getApplicationContext().getSystemService(Context.WIFI_SERVICE);
        if (mg == null){
            return "";
        }

        WifiInfo wifiInfo = mg.getConnectionInfo();
        int ip = wifiInfo.getIpAddress();
        return ((ip & 0xff) + "." + (ip >> 8 & 0xff) + "." + (ip >> 16 & 0xff) + "." + (ip >> 24 & 0xff));
    }

    //广播接受者，主要用来提醒wifi状态
    private class RssiIntentReceiver extends BroadcastReceiver{

        public void onReceive(Context context, Intent intent) {
            final String action = intent.getAction();
            if (action.equals(WifiManager.SCAN_RESULTS_AVAILABLE_ACTION)){
                wifiManager.startScan();
                String time1 = new SimpleDateFormat("yyyyMMddHHmmssSSS").format(new Date());
                Log.d(TAG, wifiManager.getScanResults().size() + ":" + time1);
            }
            else if (WifiManager.WIFI_STATE_CHANGED_ACTION.equals(action)) {//这个监听wifi的打开与关闭，与wifi的连接无关
                int wifiState = intent.getIntExtra(WifiManager.EXTRA_WIFI_STATE, -1);
                switch (wifiState) {
                    case WifiManager.WIFI_STATE_DISABLED:
                        String warning = "<font color = '#FF0033'>PLEASE OPEN WIFI FIRST!";
                        tv.setText(Html.fromHtml(warning));
                        btn2.setEnabled(false);
                        Log.e("WIFI状态", "wifiState:WIFI_STATE_DISABLED");
                        break;
                    case WifiManager.WIFI_STATE_DISABLING:
                        Log.e("WIFI状态", "wifiState:WIFI_STATE_DISABLING");
                        break;
                    case WifiManager.WIFI_STATE_ENABLED:
                        String ava = "<font color = '#339900'>WIFI IS AVALIABLE! PLEASE START!";
                        tv.setText(Html.fromHtml(ava));
                        if(!btn2.isEnabled()){
                            btn2.setEnabled(true);
                        }
                        Log.e("WIFI状态", "wifiState:WIFI_STATE_ENABLED");
                        break;
                    case WifiManager.WIFI_STATE_ENABLING:
                        Log.e("WIFI状态", "wifiState:WIFI_STATE_ENABLING");
                        break;
                    case WifiManager.WIFI_STATE_UNKNOWN:
                        Log.e("WIFI状态", "wifiState:WIFI_STATE_UNKNOWN");
                        break;
                }
            }

        }
    }

}

class mySensorListener implements SensorEventListener {

    // sensors manager
    SensorManager mSensorManager;
    Sensor mStepDetector;
    Sensor mStepCounter;
    Sensor mAccelerometer;
    Sensor mMagneticField;
    Sensor mRotationVector;

    Boolean isSensorStepDetectorPresent = false;
    Boolean isSensorStepCounterPresent = false;
    Boolean isSensorAccelerometerPresent = false;
    Boolean isSensorMagneticFieldPresent = false;
    Boolean isSensorRotationVectorPresent = false;
    Boolean lastAccelerometerSet = false;
    Boolean lastMagnetometerSet = false;

    // step detector / step counter
    static double stepLengthConstant = 75;
    double distancePDR = 0;
    int stepDetect = 0;
    float stepCount = 0;
    float lastStepCount = 0;

    // magnetometer
    float [] lastMagnetometer = new float[3];

    int orientationInDegrees = 0;
    int azimuthInDegress = 0;

    Boolean initSign = false;
    float initRadians = 0;
    float azimuthInRadians = 0;
    int initCount = 0;
    int initNums = 100;

    static final float ALPHA = 0.25f;    // low pass filter for magnetometer

    // accelerometer
    private float [] gravity = {0, 0, (float)9.72};

    float [] lastAccelerometer = new float[3];
    private float lastAccelerometerTimestamp = 0;

    private float [] currWorldAcce = new float[3];
    private float [] currInitAcce = new float[3];
    private float [] velocity = {0, 0, 0};
    private float [] positionFromAccelerometer = {0, 0, 0};

//    private int stepCountAcce = 0;
//    private boolean toggle = true;
//    private boolean ignore;     // used to step count
//    private double prevY;
//    private double threshold = 0.8;
//    private int countdown;

    // rotation vector
    float [] mRotationMatrix = new float[9];
    float [] mOrientationAngles = new float[3];
    float [] mRotationMatrixFromVector = new float[16];

    // text view
    TextView myTextView;

    int pdrNum = 0;

    public mySensorListener(SensorManager mSensorManager, TextView disp_view) {
        super();

        myTextView = disp_view;

        this.mSensorManager = mSensorManager;

        //ensure sensors
        if (mSensorManager.getDefaultSensor(TYPE_STEP_DETECTOR) != null) {
            mStepDetector = mSensorManager.getDefaultSensor(TYPE_STEP_DETECTOR);
            isSensorStepDetectorPresent = true;
        }
        if (mSensorManager.getDefaultSensor(TYPE_ACCELEROMETER) != null) {
            mAccelerometer = mSensorManager.getDefaultSensor(TYPE_ACCELEROMETER);
            isSensorAccelerometerPresent = true;
        }
        if (mSensorManager.getDefaultSensor(TYPE_MAGNETIC_FIELD) != null) {
            mMagneticField = mSensorManager.getDefaultSensor(TYPE_MAGNETIC_FIELD);
            isSensorMagneticFieldPresent = true;
        }
        if (mSensorManager.getDefaultSensor(TYPE_ROTATION_VECTOR) != null) {
            mRotationVector = mSensorManager.getDefaultSensor(TYPE_ROTATION_VECTOR);
            isSensorRotationVectorPresent = true;
        }
        if (mSensorManager.getDefaultSensor(TYPE_STEP_COUNTER) != null) {
            mStepCounter = mSensorManager.getDefaultSensor(TYPE_STEP_COUNTER);
            isSensorStepCounterPresent = true;
        }
    }

    @Override
    public void onAccuracyChanged (Sensor sensor, int accuracy) {    }

    @Override
    //value changed
    public void onSensorChanged(SensorEvent event) {

        switch (event.sensor.getType()) {
            case TYPE_STEP_DETECTOR:

                stepDetect ++;

                break;

            case TYPE_STEP_COUNTER:

                pdrNum ++;
                final long timestamp = event.timestamp;

                if (lastStepCount == 0) {
                    stepCount = 1;
                }
                else {
                    //TYPE_STEP_COUNTER.values[0]:Number of steps taken by the user since the last reboot while the sensor was activated
                    stepCount = event.values[0] - lastStepCount;
                }
                lastStepCount = event.values[0];

                if (lastAccelerometerSet && lastMagnetometerSet) {
                    mSensorManager.getRotationMatrix(mRotationMatrix, null, lastAccelerometer, lastMagnetometer);
                    mSensorManager.getOrientation(mRotationMatrix, mOrientationAngles);

                    azimuthInRadians = mOrientationAngles[0];

                    azimuthInDegress = (int) (((azimuthInRadians * 180 / (float) Math.PI) + 360) % 360);
                }

                        //distancePDR += Math.cos(azimuthInRadians - initRadians) * stepCount * stepLengthConstant / 100;
                        distancePDR = Math.cos(azimuthInRadians - initRadians) * stepCount * stepLengthConstant / 100;

                // create a new temporary thread to send distance from PDR to the server
                new Thread(new Runnable() {
                    @Override
                    public void run() {
                        try {
                            InetAddress remoteIP = InetAddress.getByName("192.168.1.123");
                            int remotePort = 2048;

                            //final String Send_Message = "2 " + Double.toString(distancePDR) + "_" + Double.toString(timestamp);

                            final String Send_Message = "2 " + String.format("%.2f",distancePDR) + "#" + pdrNum;
                            byte[] Byte_Send_Message = Send_Message.getBytes();

                            DatagramSocket UDP_Socket = new DatagramSocket();
                            DatagramPacket SendPack = new DatagramPacket(Byte_Send_Message,
                                    Byte_Send_Message.length,
                                    remoteIP,
                                    remotePort);

                            SendPack.setAddress(remoteIP);
                            SendPack.setPort(remotePort);

                            //send 5 packages to avoid missing UDP packages
                            for (int i = 0; i < 5; i ++){
                                UDP_Socket.send(SendPack);
                            }

                        }
                        catch (IOException e){
                            e.printStackTrace();
                        }
                    }
                }).start(); // the new thread need to start
                break;

            case TYPE_ACCELEROMETER:

//                //step detector
//                // we need to use a low pass filter to make data smoothed
//                gravity = lowPass(event.values, gravity);
//
//                if(ignore) {
//                    countdown--;
//                    ignore = (countdown < 0)? false : ignore;
//                }
//                else
//                    countdown = 22;
//                if(toggle && (Math.abs(prevY - gravity[1]) > threshold) && !ignore){
//                    stepCountAcce++;
//
//                    ignore = true;
//                }
//                prevY = gravity[1];
//
//                // step detector end

                double dt = (event.timestamp - lastAccelerometerTimestamp) / 1e9;

                // time is not start at zero;
                if (lastAccelerometerTimestamp == 0){
                    lastAccelerometerTimestamp = event.timestamp;
                    break;
                }

                lastAccelerometerTimestamp = event.timestamp;

                lastAccelerometerSet = true;

                System.arraycopy(event.values, 0, lastAccelerometer, 0, event.values.length);

                // body acceleration to world acceleration
                if (mRotationMatrixFromVector.length == 9) {
                    currWorldAcce[0] = mRotationMatrixFromVector[0] * event.values[0]
                            + mRotationMatrixFromVector[1] * event.values[1]
                            + mRotationMatrixFromVector[2] * event.values[2];
                    currWorldAcce[1] = mRotationMatrixFromVector[3] * event.values[0]
                            + mRotationMatrixFromVector[4] * event.values[1]
                            + mRotationMatrixFromVector[5] * event.values[2];
                    currWorldAcce[2] = mRotationMatrixFromVector[6] * event.values[0]
                            + mRotationMatrixFromVector[7] * event.values[1]
                            + mRotationMatrixFromVector[8] * event.values[2];
                }
                else if (mRotationMatrixFromVector.length == 16) {
                    currWorldAcce[0] = mRotationMatrixFromVector[0] * event.values[0]
                            + mRotationMatrixFromVector[1] * event.values[1]
                            + mRotationMatrixFromVector[2] * event.values[2];
                    currWorldAcce[1] = mRotationMatrixFromVector[4] * event.values[0]
                            + mRotationMatrixFromVector[5] * event.values[1]
                            + mRotationMatrixFromVector[6] * event.values[2];
                    currWorldAcce[2] = mRotationMatrixFromVector[8] * event.values[0]
                            + mRotationMatrixFromVector[9] * event.values[1]
                            + mRotationMatrixFromVector[10] * event.values[2];
                }

                currWorldAcce[0] = currWorldAcce[0] - gravity[0];
                currWorldAcce[1] = currWorldAcce[1] - gravity[1];
                currWorldAcce[2] = currWorldAcce[2] - gravity[2];

                // rotate to init angle
                currInitAcce[0] = currWorldAcce[0] * (float) Math.sin(initRadians) + currWorldAcce[1] * (float) Math.cos(initRadians);
                currInitAcce[1] = currWorldAcce[0] * (float) Math.cos(initRadians) - currWorldAcce[1] * (float) Math.sin(initRadians);
                currInitAcce[2] = currWorldAcce[2];

                // velocity
                velocity[0] += currInitAcce[0] * dt;
                velocity[1] += currInitAcce[1] * dt;
                velocity[2] += currInitAcce[2] * dt;

                // position
                positionFromAccelerometer[0] += velocity[0] * dt + 0.5 * currInitAcce[0] * dt * dt;
                positionFromAccelerometer[1] += velocity[1] * dt + 0.5 * currInitAcce[1] * dt * dt;
                positionFromAccelerometer[2] += velocity[2] * dt + 0.5 * currInitAcce[2] * dt * dt;

                final String positionString;

                positionString = String.format(Locale.ENGLISH, "(%f, %f, %f), (%f, %f, %f) at %f\n",
                        positionFromAccelerometer[0], positionFromAccelerometer[1], positionFromAccelerometer[2],
                        currInitAcce[0], currInitAcce[1], currInitAcce[2],
                        dt);

//                WriteToFile("Acce.txt", writeString);

                // create a new temporary thread to send distance from accelerometer to the server
//                new Thread(new Runnable() {
//                    @Override
//                    public void run() {
//                        try {
//                            InetAddress remoteIP = InetAddress.getByName("192.168.1.123");
//                            int remotePort = 2048;
//
//                            String Send_Message = "3 " + positionString;
//                            byte[] Byte_Send_Message = Send_Message.getBytes();
//
//                            DatagramSocket UDP_Socket = new DatagramSocket();
//                            DatagramPacket SendPack = new DatagramPacket(Byte_Send_Message,
//                                    Byte_Send_Message.length,
//                                    remoteIP,
//                                    remotePort);
//
//                            SendPack.setAddress(remoteIP);
//                            SendPack.setPort(remotePort);
//                            UDP_Socket.send(SendPack);
//                        }
//                        catch (IOException e){
//                            e.printStackTrace();
//                        }
//                    }
//                }).start(); // the new thread need to be start

                break;

            case TYPE_MAGNETIC_FIELD:

                float azimuthInRadians = 0;

                lastMagnetometerSet = true;

                System.arraycopy(event.values, 0, lastMagnetometer, 0, event.values.length);

                lastMagnetometer = lowPass(event.values.clone(), lastMagnetometer);

                if (lastAccelerometerSet && lastMagnetometerSet)
                {
                    mSensorManager.getRotationMatrix(mRotationMatrix, null, lastAccelerometer, lastMagnetometer);
                    mSensorManager.getOrientation(mRotationMatrix, mOrientationAngles);

                    azimuthInRadians = mOrientationAngles[0];

                    azimuthInDegress = ((int)(azimuthInRadians * 180/(float) Math.PI) + 360) % 360;

                }

                // get init radians
                if (!initSign) {
                    initCount ++;
                    if (initCount <= initNums) {
                        initRadians = (initRadians * (initCount - 1) + azimuthInRadians) / initCount;
                    }
                    else {
                        initSign = true;
                    }
                }

                break;

            case TYPE_ROTATION_VECTOR:

                SensorManager.getRotationMatrixFromVector(mRotationMatrixFromVector, event.values);
                SensorManager.getOrientation(mRotationMatrixFromVector, mOrientationAngles);

                orientationInDegrees = ((int)(mOrientationAngles[0] * 180/(float) Math.PI) + 360) % 360;

                break;
        }
//        myTextView.setText(pdrNum + "distance:" + Double.toString(distancePDR));
//        myTextView.setText("init degress: " + Double.toString(((int)(initRadians * 180/(float) Math.PI) + 360) % 360) + "\n");
//        myTextView.append("azimuthInDegress: " + Double.toString(azimuthInDegress) + "\n");
//        myTextView.append("last step count: " + Double.toString(lastStepCount) + "\n");
//        myTextView.append("steps count: " + Double.toString(stepCount) + "\n");
//        myTextView.append("distance: " + Double.toString(distancePDR) + "\n");
//        myTextView.append("step Length: " + Double.toString(stepLengthConstant) + "\n");

    }

    protected float[] lowPass( float[] input, float[] output ) {
        if ( output == null ) return input;

        for ( int i=0; i<input.length; i++ ) {
            output[i] = output[i] + ALPHA * (input[i] - output[i]);
        }
        return output;
    }


    public void mySensorRegister(){
        if (isSensorStepDetectorPresent) {
            mSensorManager.registerListener(this, mStepDetector,
                    SensorManager.SENSOR_DELAY_FASTEST);
//            myTextView.setText("step detector is ready\n");
        }
        if (isSensorAccelerometerPresent) {
            mSensorManager.registerListener(this, mAccelerometer,
                    SensorManager.SENSOR_DELAY_FASTEST);
//            myTextView.append("accelerometer is ready\n");
        }
        if (isSensorMagneticFieldPresent) {
            mSensorManager.registerListener(this, mMagneticField,
                    SensorManager.SENSOR_DELAY_FASTEST);
//            myTextView.append("magnetic is ready\n");
        }
        if (isSensorRotationVectorPresent) {
            mSensorManager.registerListener(this, mRotationVector,
                    SensorManager.SENSOR_DELAY_FASTEST);
//            myTextView.append("rotation vector is ready\n");
        }
        if (isSensorStepCounterPresent) {
            mSensorManager.registerListener(this, mStepCounter,
                    SensorManager.SENSOR_DELAY_FASTEST);
//            myTextView.append("step counter is ready\n");
        }
    }

    public void mySensorUnregister(){
        mSensorManager.unregisterListener(this);
    }

    private void WriteToFile(String filename, String messageWriteToFile)
    {

        // save the message to file
        String state = Environment.getExternalStorageState();

        if (Environment.MEDIA_MOUNTED.equals(state)) {
            Log.d("Write to file","Environment ready");
        }
        else {
            Log.e("Write to file","Can't write to file");
        }

        // this code will cause error if the directory 'document' is not exist
        File file = new File(Environment.getExternalStoragePublicDirectory(
                Environment.DIRECTORY_DOCUMENTS), filename);


        if(!file.exists()) {
            try {
                if(file.createNewFile()){
                    Log.d("Write to file", "create new file success");
                }
                else{
                    Log.e("Write to file", "create new file failed\n");
                }
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        else {
            Log.d("Write to file", "The file is existed\n");
        }

        if(file.canWrite()) {
            try {
                // create FileWriter to write to a file
                // buff the output to a file
                FileWriter fw = new FileWriter(file, true);
                BufferedWriter bw = new BufferedWriter(fw);

                bw.write(messageWriteToFile); // write a message
                bw.newLine();
                bw.flush(); // flush the buffer
                bw.close();
                fw.close();
            } catch (IOException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        }
        else {
            Log.e("Write to file", "Error: The File cannot be Writen!\n");
        }

    }
}