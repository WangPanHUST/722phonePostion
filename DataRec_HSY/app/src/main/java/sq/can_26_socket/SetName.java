package sq.can_26_socket;

import android.content.Intent;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStreamReader;

public class SetName extends AppCompatActivity {
    private Button btn1;
    private EditText et_ip,et_ap1,et_ap2;

    @Override
    protected void onCreate(Bundle savedInstanceState){
        super.onCreate(savedInstanceState);
        setContentView(R.layout.setname);

        btn1 = (Button) findViewById(R.id.btn);
        et_ip = (EditText) findViewById(R.id.et_ip);
        et_ap1 = (EditText)findViewById(R.id.et_ap1);
        et_ap2 = (EditText)findViewById(R.id.et_ap2);
        init();//初始化，读取文件中的数据

        btn1.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                String ip = et_ip.getText().toString();
                String ap1 = et_ap1.getText().toString();
                String ap2 = et_ap2.getText().toString();
                String [] Names = {ip,ap1,ap2};

                write(ip,ap1,ap2);//写入数据

                Intent intent = new Intent(SetName.this, MainActivity.class);
                Bundle bundle = new Bundle();// 创建Bundle对象
                intent.putExtra("Data",Names);//赋予Bundle要传输的值
                intent.putExtras(bundle);// 将Bundle对象放入到Intent上
                startActivity(intent);
            }
        });
    }

    // 读取缓存内的数据
    private void init(){
        File file = new File(this.getCacheDir(),"Namelog.txt");

        if(file.exists() && file.length() > 0){
            try {
                //读流
                FileInputStream fis = new FileInputStream(file);
                BufferedReader br = new BufferedReader(new InputStreamReader(fis));
                String Namelog = br.readLine();

                //用#分隔，所以用#切割
                String ip = Namelog.split("#")[0];
                String ap1 = Namelog.split("#")[1];
                String ap2 = Namelog.split("#")[2];
                //并将缓存的帐号密码显现
                et_ip.setText(ip);
                et_ap1.setText(ap1);
                et_ap2.setText(ap2);
                fis.close();
                br.close();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    //保存数据
    private void write(String ip,String ap1,String ap2){
        File file = new File(this.getCacheDir(),"Namelog.txt");

        try {
            //写流
            FileOutputStream fos = new FileOutputStream(file);
            fos.write((ip + "#" + ap1 +"#" + ap2).getBytes());
            fos.close();
        } catch (Exception e) {
            e.printStackTrace();
        }

    }

}
