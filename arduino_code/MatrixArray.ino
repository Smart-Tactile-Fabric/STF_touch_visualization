#define SCAN_RATE                 115200
#define MATRIX_SIZE               15

#define SENSOR_PIN                A0
#define SR_DATA_PIN               2
#define SR_CLOCK_PIN              3
#define MUX_S0                    4
#define MUX_S1                    5
#define MUX_S2                    6
#define MUX_S3                    7
#define MUX_ENABLE                8

// 快速端口操作
#define SR_DATA_HIGH()            PORTD |= (1<<2)
#define SR_DATA_LOW()             PORTD &= ~(1<<2)
#define SR_CLOCK_HIGH()           PORTD |= (1<<3)
#define SR_CLOCK_LOW()            PORTD &= ~(1<<3)

void setup() {
  Serial.begin(SCAN_RATE);

  // 配置引脚
  pinMode(SENSOR_PIN, INPUT);
  pinMode(SR_DATA_PIN, OUTPUT);
  pinMode(SR_CLOCK_PIN, OUTPUT);
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);
  pinMode(MUX_ENABLE, OUTPUT);

  // 启用第一个多路复用器
  digitalWrite(MUX_ENABLE, LOW);

  // 提高ADC速度
  ADCSRA = (ADCSRA & 0xF8) | 0x04; // 16分频
}

void loop() {
  scanMatrix();
  Serial.println("---"); // 帧分隔符
  delay(10);
}

void scanMatrix() {
  for (byte row = 0; row < MATRIX_SIZE; row++) {
    selectRow(row);

    // 重置列选择
    resetColumns();

    for (byte col = 0; col < MATRIX_SIZE; col++) {
      // 选择当前列
      advanceColumn();

      // 读取并处理传感器数据
      int sensorValue = analogRead(SENSOR_PIN);
      byte processedValue = processReading(sensorValue);

      Serial.print(processedValue);
      if (col < MATRIX_SIZE - 1) Serial.print(",");
    }
    Serial.println();
  }
}

void selectRow(byte row) {
  // 设置多路复用器通道
  digitalWrite(MUX_S0, row & 0x01);
  digitalWrite(MUX_S1, row & 0x02);
  digitalWrite(MUX_S2, row & 0x04);
  digitalWrite(MUX_S3, row & 0x08);
}

void resetColumns() {
  // 重置移位寄存器
  SR_DATA_HIGH();
  pulseClock();
  SR_DATA_LOW();

  // 填充剩余位
  for (byte i = 0; i < MATRIX_SIZE; i++) {
    pulseClock();
  }
}

void advanceColumn() {
  pulseClock();
}

void pulseClock() {
  SR_CLOCK_HIGH();
  SR_CLOCK_LOW();
}

byte processReading(int raw) {
  // 转换10位ADC到8位并应用阈值
  byte value = (raw >> 2) & 0xFF;
  return (value > 45) ? (value - 45) : 0;
}