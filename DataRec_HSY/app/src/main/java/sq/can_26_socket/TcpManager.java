package sq.can_26_socket;

import android.net.wifi.WifiManager;
import android.os.Handler;

import java.net.Socket;

/**
 * Created by sp01 on 2017/4/26.
 */

public class TcpManager {
    public static final int OK = 0;
    private static WifiManager wifiManager;
    private static String dsName = "192.168.1.187";
    private static int dsPort = 10086;
    private static Socket socket;

    private static TcpManager instance;
    private Handler mHandler;
    private boolean isExitTcp = false;
    private Thread tcpThread;

    public static TcpManager getInstance(){
        if (instance == null){
            synchronized (TcpManager.class){
                if (instance == null){
                    instance = new TcpManager();
                }
            }
        }
        return instance;
    }

    public boolean connection(Handler handler){
        mHandler = handler;

        if (socket == null){
            tcpThread = new Thread(new Runnable() {
                @Override
                public void run() {
                    //

                }
            });
        }
        return true;
    }


}
                         /*
                    for(RssiNum = 0;RssiNum < 50; RssiNum++){
                        dataString = ScanWifiInfo();
                        System.out.println("NO."+ RssiNum + ":" + dataString);
                        new Thread(new Runnable() {
                            @Override
                            public void run() {
                            try {
                                //发送
                                OutputStream outputStream = null;
                                outputStream = socket.getOutputStream();
                                outputStream.write(dataString.getBytes("utf-8"));
                                outputStream.flush();
                            } catch (IOException e) {e.printStackTrace();}
                            }
                        }).start();
                        try{Thread.sleep(900);}catch(InterruptedException e){e.printStackTrace();}
                    } */
