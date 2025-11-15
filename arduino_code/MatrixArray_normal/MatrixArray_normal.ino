#define BAUD_RATE                 1000000
#define ROW_COUNT                 16
#define COLUMN_COUNT              16

#define PIN_ADC_INPUT             A0
#define PIN_SHIFT_REGISTER_DATA   2
#define PIN_SHIFT_REGISTER_CLOCK  3
#define PIN_MUX_CHANNEL_0         4
#define PIN_MUX_CHANNEL_1         5
#define PIN_MUX_CHANNEL_2         6
#define PIN_MUX_CHANNEL_3         7
#define PIN_MUX_INHIBIT_0         8
// #define PIN_MUX_INHIBIT_1         9

// 移位寄存器控制宏定义（直接操作端口以提高速度）
#define SET_SR_DATA_HIGH()        PORTD|=B00000100
#define SET_SR_DATA_LOW()         PORTD&=~B00000100
#define SET_SR_CLK_HIGH()         PORTD|=B00001000
#define SET_SR_CLK_LOW()          PORTD&=~B00001000

#define ROWS_PER_MUX              16
#define MUX_COUNT                 1
#define CHANNEL_PINS_PER_MUX      4

// 点偏移配置：为每个传感器点设置要减去的数值
// 格式：POINT_OFFSETS[行索引][列索引] = 要减去的值
// 行索引：0-15 对应第1-16行
// 列索引：0-15 对应第1-16列

// 16*16的零矩阵：标定初始化时释放注释并烧入
int POINT_OFFSETS[ROW_COUNT][COLUMN_COUNT] = {
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};

// 使用multi_thread_contact_v0.py代码：标定完成时覆盖触觉矩阵后释放注释并烧入
// int POINT_OFFSETS[ROW_COUNT][COLUMN_COUNT] = {
// { 68, 65, 64, 63, 63, 63, 63, 64, 66, 66, 68, 68, 69, 70, 72, 67 },
// { 90, 82, 74, 71, 73, 68, 66, 68, 72, 72, 72, 71, 72, 73, 77, 67 },
// { 82, 76, 70, 68, 69, 66, 66, 67, 71, 71, 72, 71, 73, 75, 81, 67 },
// { 104, 92, 77, 75, 77, 72, 69, 71, 76, 74, 74, 73, 74, 75, 79, 67 },
// { 108, 93, 77, 75, 77, 72, 70, 71, 77, 75, 74, 73, 74, 74, 78, 67 },
// { 109, 93, 76, 76, 78, 74, 72, 74, 81, 78, 77, 74, 76, 77, 81, 67 },
// { 112, 90, 75, 76, 77, 75, 73, 76, 82, 80, 79, 75, 77, 78, 82, 67 },
// { 95, 78, 69, 70, 70, 70, 69, 71, 75, 75, 77, 74, 78, 82, 94, 67 },
// { 64, 63, 63, 62, 62, 62, 63, 64, 65, 66, 68, 68, 69, 70, 69, 67 },
// { 131, 89, 75, 76, 70, 76, 74, 72, 72, 71, 72, 71, 72, 73, 73, 67 },
// { 135, 87, 73, 74, 70, 74, 73, 72, 72, 72, 72, 71, 72, 73, 73, 67 },
// { 127, 82, 72, 72, 69, 73, 72, 72, 72, 72, 72, 71, 72, 73, 73, 67 },
// { 124, 83, 72, 73, 70, 74, 73, 73, 74, 73, 73, 71, 72, 73, 74, 67 },
// { 114, 79, 69, 70, 68, 71, 71, 71, 72, 72, 72, 71, 72, 73, 74, 67 },
// { 121, 87, 73, 74, 73, 75, 74, 74, 77, 75, 76, 73, 74, 76, 78, 67 },
// { 81, 70, 66, 66, 65, 66, 66, 68, 70, 71, 72, 72, 74, 80, 89, 67 }
// };

// AVR寄存器操作宏定义
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

int current_enabled_mux = MUX_COUNT - 1;  // 当前使能的MUX编号

void setup()
{
  Serial.begin(BAUD_RATE);

  // 初始化所有引脚
  pinMode(PIN_ADC_INPUT, INPUT);
  pinMode(PIN_SHIFT_REGISTER_DATA, OUTPUT);
  pinMode(PIN_SHIFT_REGISTER_CLOCK, OUTPUT);
  pinMode(PIN_MUX_CHANNEL_0, OUTPUT);
  pinMode(PIN_MUX_CHANNEL_1, OUTPUT);
  pinMode(PIN_MUX_CHANNEL_2, OUTPUT);
  pinMode(PIN_MUX_CHANNEL_3, OUTPUT);
  pinMode(PIN_MUX_INHIBIT_0, OUTPUT);
  // pinMode(PIN_MUX_INHIBIT_1, OUTPUT);

  // 设置ADC预分频器为CLK/16，提高ADC采样速度
  sbi(ADCSRA,ADPS2);
  cbi(ADCSRA,ADPS1);
  cbi(ADCSRA,ADPS0);
}
void loop()
{
  for(int i = 0; i < ROW_COUNT; i ++)
  {
    setRow(i);
    shiftColumn(true);
    shiftColumn(false);                    
    for(int j = 0; j < COLUMN_COUNT; j ++)
    {
      int raw_reading = analogRead(PIN_ADC_INPUT);
      byte send_reading = (byte) (lowByte(raw_reading >> 2));

      // 应用点偏移：从读取值中减去对应点的偏移值
      int offset_reading = send_reading - POINT_OFFSETS[i][j] - 15;
      offset_reading = constrain(offset_reading, 0, 255);  // 限制在0-255范围内
      
      shiftColumn(false);
      // Serial.print(send_reading);
      Serial.print(offset_reading);
      Serial.print(" ");
    }
    Serial.println();
  }
  Serial.println();
  // delay(2000);
}

/**
 * 设置当前扫描的行
 * @param row_number 行编号 (0-15)
 */
void setRow(int row_number)
{
  // 每扫描完一个MUX的所有行后，切换到下一个MUX
  if((row_number % ROWS_PER_MUX) == 0)
  {
    // 禁用当前MUX
    digitalWrite(PIN_MUX_INHIBIT_0 + current_enabled_mux, HIGH);

    // 切换到下一个MUX
    current_enabled_mux ++;
    if(current_enabled_mux >= MUX_COUNT)
    {
      current_enabled_mux = 0;  // 循环回到第一个MUX
    }

    // 使能新的MUX
    digitalWrite(PIN_MUX_INHIBIT_0 + current_enabled_mux, LOW);
  }

  // 设置MUX通道选择引脚
  for(int i = 0; i < CHANNEL_PINS_PER_MUX; i ++)
  {
    if(bitRead(row_number, i))
    {
      digitalWrite(PIN_MUX_CHANNEL_0 + i, HIGH);
    }
    else
    {
      digitalWrite(PIN_MUX_CHANNEL_0 + i, LOW);
    }
  }
}

/**
 * 控制移位寄存器进行列切换
 * @param is_first 是否为第一次移位（需要设置数据线为高电平）
 */
void shiftColumn(boolean is_first)
{
  if(is_first)
  {
    SET_SR_DATA_HIGH();  // 第一次移位前设置数据线为高
  }

  // 产生时钟脉冲
  SET_SR_CLK_HIGH();
  SET_SR_CLK_LOW();

  if(is_first)
  {
    SET_SR_DATA_LOW();   // 第一次移位后恢复数据线为低
  }
}
