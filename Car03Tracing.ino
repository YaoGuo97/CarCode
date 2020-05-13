#include <Servo.h>

// 创建舵机对象
Servo servo1;
Servo servo2;

/* 引脚 */
// 电机
const int negR = 13; // 右轮马达负极
const int posR = 12; // 右轮马达正极
const int negL = 7; // 左轮马达负极
const int posL = 4; // 左轮马达正极
const int pwmR = 6;  // 右轮PWM转速控制
const int pwmL = 5;  // 左轮PWM转速控制

// 超声波
const int TrgPin = A0;  // 超声波输入脚 回声
const int EcoPin = A1; // 超声波输出脚 触发

// 红外线
const int TraPin2 = A2; // 右红外线寻迹模块
const int TraPin3 = A3; // 中红外线寻迹模块
const int TraPin4 = A4; // 左红外线寻迹模块
byte Tradata = 0; //红外线寻迹模块感应值

const int la = 3; // 灯光

const int fm = 2; // 蜂鸣器引脚控制

// 舵机
const int se1 = 11; // 舵机左右摆动
const int se2 = 10; // 舵机上下摆动

/* 常量 变量  */
// 电机
const int speed_max = 400;  // 限速 马达正转最大速度
const int speed_min = 0; // 限速 马达反转最大速度
int lspeed = 200; // 左马达初始速度
int rspeed = 200; // 右马达初始速度
const int obspeed01 = 190; // 避障/寻迹 模式转弯速度
const int obspeed02 = 200; // 避障/寻迹 模式转弯速度
const int speed_step = 10; // 电机修改步长

// 舵机
const int camera_left = 135;  // 舵机右转动限定
const int camera_right = 35; // 舵机左转动限定
const int camera_up = 0; // 舵机向上转动限定
const int camera_down = 110; // 舵机向下转动限定
int updowncamera = 90; // 舵机初始角度
int leftrightcamera = 90; // 舵机初始角度
const int camera_step = 5; // 舵机修改步长

// 障碍物距离 cm
float distance; // 超声波测障碍物距离 cm
float rdistance; // 右侧障碍物距离 cm
float ldistance; // 左侧障碍物距离 cm
float cdistance; // 中间障碍物距离 cm
const float distance_max = 40; // 判定最大距离 cm

// 字符串解析 变量
String inputString;
boolean stringComplete;	

//初始化函数
void setup() {

  // 端口输入输出模式
  pinMode(EcoPin, INPUT);
  pinMode(TrgPin, OUTPUT);
  pinMode(negR, OUTPUT);
  pinMode(posR, OUTPUT);
  pinMode(negL, OUTPUT);
  pinMode(posL, OUTPUT);
  pinMode(pwmR, OUTPUT);
  pinMode(pwmL, OUTPUT);
  pinMode(fm, OUTPUT);
  pinMode(la, OUTPUT);
  pinMode(TraPin2, INPUT_PULLUP); // 设置模拟引脚A2为含提升电阻的输入引脚
  pinMode(TraPin3, INPUT_PULLUP); // 设置模拟引脚A2为含提升电阻的输入引脚
  pinMode(TraPin4, INPUT_PULLUP); // 设置模拟引脚A2为含提升电阻的输入引脚

  // 串口波特率配置
  Serial.begin(9600);

  // 舵机
  servo1.attach(se1);
  servo1.write(leftrightcamera);
  delay(500);
  servo2.attach(se2);
  servo2.write(updowncamera);
  buzzer(4);
}

// 主函数
void loop() {

  serialEvent();
  
}

// 串口主要处理事件
void serialEvent() {
  while (Serial.available()) {

    // 获取新字节
    char inchar = (char)Serial.read();

    // 添加到新字符串
    if (inchar != '\n') {
      inputString += inchar;
    }

    // 为空就设置标志符
    if (inchar == '\n') {
      stringComplete = true;
      //Serial.println("I am in");
    }
	
	// 标志位判断
    if (stringComplete) {

      Serial.println(inputString);
      inputString = String(inputString);
	  
	  // 前进
      if (inputString == "cfw") {
        Serial.println("OK!");
        forward(lspeed, rspeed);
      }
	  
	  // 后退
      if (inputString == "cbo") {
        Serial.println("OK!");
        back(lspeed, rspeed);
      }
	  
	  // 左转
      if (inputString == "clt") {
        Serial.println("OK!");
        lefttrun(lspeed, rspeed);
      }
	  
	  // 右转
      if (inputString == "crt") {
        Serial.println("OK!");
        righttrun(lspeed, rspeed);
      }
	  
	  // 电机加速
      if (inputString == "emlfw") {
        Serial.println("OK!");
        if (lspeed < speed_max) {
          lspeed += speed_step;
          rspeed += speed_step;
        }
      }
	
	  // 电机减速
      if (inputString == "emlbo") {
        Serial.println("OK!");
        if (lspeed > speed_min) {
          lspeed -= speed_step;
          rspeed -= speed_step;
        }
      }
	  
	  // 停止
      if (inputString == "cop") {
        Serial.println("STOP!");
        pause(lspeed, rspeed);
      }
	  
	  // 避障模式
      if (inputString == "oa") {
        Serial.println("STOP!");
        buzzer(1);
        obsmodel();
      }

	  // 控制模式
      if (inputString == "ct") {
        Serial.println("OK!");
        buzzer(2);
      }
	
	  // 寻迹模式
      if (inputString == "tr") {
        Serial.println("OK!");
        buzzer(3);
        tracing();
      }
		
	 // 舵机左右摆动 减
      if (inputString == "sert") {
        Serial.println("OK!");
        if (leftrightcamera > camera_right) {
          leftrightcamera -= camera_step;
          servo1.write(leftrightcamera);
        }
      }

	  // 舵机左右摆动 加
      if (inputString == "selt") {
        Serial.println("OK!");
        if (leftrightcamera < camera_left) {
          leftrightcamera += camera_step;
          servo1.write(leftrightcamera);
        }
      }

	  // 舵机上下摆动 减
      if (inputString == "sefw") {
        Serial.println("OK!");
        if (updowncamera > camera_up) {
          updowncamera -= camera_step;
          servo2.write(updowncamera);
        }
      }
	  
	  // 舵机上下摆动 加
      if (inputString == "sebo") {
        Serial.println("OK!");
        if (updowncamera < camera_down) {
          updowncamera += camera_step;
          servo2.write(updowncamera);
        }
      }
	  
	  // 灯光 开灯
      if (inputString == "lao") {
        onlights();
      }

	  // 灯光 关灯
      if (inputString == "las") {
        offlights();
      }

      // 清除字符串
      clearchar();

    }
  }
}

// 避障模式
void obsmodel() {

  // 清除字符串
  clearchar();
  while (true) {
	
	// 测距
    ranging();
    cdistance = distance;

    if (cdistance <= distance_max) {
      pause(lspeed, rspeed);
      buzzer(1);
      servo1.write(45);
      delay(500);
      ranging();
      rdistance = distance;
      // Serial.println(rdistance);
      servo1.write(135);
      delay(500);
      ranging();
      ldistance = distance;
      // Serial.println(ldistance);
      servo1.write(90);

      if (rdistance <= distance_max && ldistance <= distance_max) {
        buzzer(1);
        back(obspeed01, obspeed01);
        delay(1000);
        righttrun(obspeed02, obspeed01);
        delay(1000);
        forward(lspeed, rspeed);
      }

      else if (rdistance > ldistance) {
        buzzer(1);
        righttrun(obspeed02, obspeed01);
        delay(1000);
      }

      else if (rdistance < ldistance) {
        buzzer(1);
        lefttrun(obspeed01, obspeed02);
        delay(1000);
      }
    }

    else {
      forward(obspeed01, obspeed01);
    }

    // 字符获取解析
    charpar();

    if (inputString == "ct" || inputString == "tr")

    {
      serialEvent(); //add
      pause(lspeed, rspeed);
      buzzer(2);
      // 清除字符串
      clearchar();
      break;
    }
    
    // 清除字符
    clearchar();


  }
}

// 寻迹模式
void tracing() {
  
  // 清除字符串
  clearchar();
  
  while(true) {
  infrared();
  // 字符获取解析
  charpar();

  if (inputString == "ct" || inputString == "oa") {
      serialEvent(); //add
      pause(lspeed, rspeed);
      buzzer(2);
    
      // 清除字符串
      clearchar();
      break;
    }
    
    // 清除字符
    clearchar();
  }
  
  
}

// 寻迹模式马达控制函数
void trastart(byte Tradata) {
	switch(Tradata) {
		case 0: 
		forward(lspeed, rspeed);
		break;
		case 1:
		righttra(1, lspeed, rspeed);
		break;
		case 2:
		forward(lspeed, rspeed);
		break;
		case 3:
		righttra(0, 250, rspeed);
		break;
		case 4:
		lefttra(1, lspeed, rspeed);
		break;
		case 5:
		pause(0, 0);
		break;
		case 6:
		lefttra(0, lspeed, 250);
		break;
		case 7:
		pause(0, 0);
		break;
    default:
    break;
	}
}

// 字符解析
void charpar() {
  while (Serial.available()) {

    // 获取新字节
    char inchar = (char)Serial.read();

    // 添加到新字符串
    if (inchar != '\n') {
      inputString += inchar;
    }

    // 为空就设置标志符
    if (inchar == '\n') {
      stringComplete = true;
      //Serial.println("I am in");
    }

    if (stringComplete) {

      Serial.println(inputString);
      inputString = String(inputString);
    }

  }
}

// 清除字符
void clearchar() {
  inputString = "";
  stringComplete = false;
}

// 前进函数
void forward(int Lemspeed, int Remspeed) {
  analogWrite(pwmR, Remspeed);
  analogWrite(pwmL, Lemspeed);
  digitalWrite(posR, HIGH);
  digitalWrite(negR, LOW);
  digitalWrite(posL, HIGH);
  digitalWrite(negL, LOW);
}

// 后退函数
void back(int Lemspeed, int Remspeed) {
  analogWrite(pwmR, Remspeed);
  analogWrite(pwmL, Lemspeed);
  digitalWrite(posR, LOW);
  digitalWrite(negR, HIGH);
  digitalWrite(posL, LOW);
  digitalWrite(negL, HIGH);
}

// 左转函数
void lefttrun(int Lemspeed, int Remspeed) {
  analogWrite(pwmR, Remspeed);
  analogWrite(pwmL, Lemspeed);
  digitalWrite(posR, HIGH);
  digitalWrite(negR, LOW);
  digitalWrite(posL, LOW);
  digitalWrite(negL, HIGH); 
}

// 寻迹模式左转函数
void lefttra(byte flag, int Lemspeed, int Remspeed) {
  analogWrite(pwmR, Remspeed);
  analogWrite(pwmL, Lemspeed);
  if (flag == 1) {
	  digitalWrite(posR, HIGH);
	  digitalWrite(negR, LOW);
	  digitalWrite(posL, LOW);
	  digitalWrite(negL, HIGH); 
  }
  else {
	  digitalWrite(posR, HIGH);
	  digitalWrite(negR, LOW);
	  digitalWrite(posL, LOW);
	  digitalWrite(negL, LOW); 
  }  
}

// 右转函数
void righttrun(int Lemspeed, int Remspeed) {
  analogWrite(pwmR, Remspeed);
  analogWrite(pwmL, Lemspeed);
  digitalWrite(posR, LOW);
  digitalWrite(negR, HIGH); 
  digitalWrite(posL, HIGH);
  digitalWrite(negL, LOW); 
}

// 寻迹模式右转函数
void righttra(byte flag, int Lemspeed, int Remspeed) {
  analogWrite(pwmR, Remspeed);
  analogWrite(pwmL, Lemspeed);	
  if(flag == 1) {
	  digitalWrite(posR, LOW);
	  digitalWrite(negR, HIGH);
	  digitalWrite(posL, HIGH);
	  digitalWrite(negL, LOW);	  
  }
  else {
	  digitalWrite(posR, LOW);
	  digitalWrite(negR, LOW);
	  digitalWrite(posL, LOW);
	  digitalWrite(negL, HIGH);	
  }
}

// 停止函数
void pause(int Lemspeed, int Remspeed) {
  analogWrite(pwmR, Remspeed);
  analogWrite(pwmL, Lemspeed);
  digitalWrite(posR, LOW);
  digitalWrite(negR, LOW);
  digitalWrite(posL, LOW);
  digitalWrite(negL, LOW);
}

// 蜂鸣器提示
void buzzer(int fre) {

  for (int i = 0; i < fre; i++) {
    digitalWrite(fm, HIGH);
    delay(200);
    digitalWrite(fm, LOW);
    delay(200);
  }

}

// 开灯
void onlights() {
  digitalWrite(la, HIGH);
}

// 关灯
void offlights() {
  digitalWrite(la, LOW);
}

// 超声波测距函数
int ranging() {
  digitalWrite(TrgPin, LOW);
  delayMicroseconds(10);
  digitalWrite(TrgPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrgPin, LOW);
  distance = pulseIn(EcoPin, HIGH) / 58.00;
}

// 红外线检测
void infrared() {
	int val; //输入模拟信号值
	Tradata = 0;
	val = analogRead(TraPin4); //左
	if(val >= 150)
	Tradata = (Tradata + 4);
	val = analogRead(TraPin3); //中
	if(val >= 150)
	Tradata = (Tradata + 2);
	val = analogRead(TraPin2); //右
	if(val >= 150)
	Tradata = (Tradata + 1);
	trastart(Tradata); //计算完毕 小车开始行使
}
