/*
 * ========================================================
 * ESP32-S3 录音机程序 - INMP441麦克风 + MAX98357功放
 * ========================================================
 * 
 * 硬件连接说明：
 * --------------------------------------------------------
 * 组件           引脚       ESP32-S3引脚       说明
 * --------------------------------------------------------
 * INMP441麦克风
 *                VDD        3.3V              电源
 *                GND        GND               地线
 *                SD         GPIO16            数据输出
 *                WS         GPIO17            字选择
 *                SCK        GPIO18            时钟
 *                L/R        GND               左声道选择
 * 
 * MAX98357功放
 *                VIN        3.3V              电源
 *                GND        GND               地线
 *                SD         3.3V              使能引脚（常开）
 *                GAIN       悬空              15dB增益
 *                DIN        GPIO40            数据输入
 *                BCK        GPIO41            位时钟
 *                LRC        GPIO42            帧时钟
 * 
 * LED指示灯
 *                正极       GPIO11            录音状态指示灯
 *                负极       GND               接地
 * 
 * 外部按钮
 *                信号线     GPIO21            控制按钮（高电平触发）
 *                VCC        3.3V              按钮电源
 *                GND        GND               按钮地线
 * --------------------------------------------------------
 * 
 * 功能说明：
 * 1. 使用INMP441数字麦克风进行录音
 * 2. 录音时LED常亮，播放时LED闪烁
 * 3. 通过外部按钮控制录音/播放
 * 4. 录音完成自动播放
 * 5. 支持音频质量分析和错误提示
 * 
 * 作者：未知
 * 日期：未知
 * 版本：优化版
 * ========================================================
 */

#include <driver/i2s.h>

// ========================================================
// 硬件引脚定义
// ========================================================

// 按钮和LED引脚
#define BUTTON_PIN     21    // 外部按钮信号线
#define LED_PIN        11    // 录音状态指示灯

// INMP441麦克风引脚
#define MIC_DATA_PIN   16    // INMP441 SD引脚（数据输出）
#define MIC_WS_PIN     17    // INMP441 WS引脚（字选择）
#define MIC_SCK_PIN    18    // INMP441 SCK引脚（时钟）
#define I2S_MIC        I2S_NUM_0  // 麦克风使用的I2S端口

// MAX98357功放引脚
#define SPK_DATA_PIN   40    // MAX98357 DIN引脚（数据输入）
#define SPK_WS_PIN     42    // MAX98357 LRC引脚（帧时钟）
#define SPK_SCK_PIN    41    // MAX98357 BCK引脚（位时钟）
#define I2S_SPK        I2S_NUM_1  // 功放使用的I2S端口

// ========================================================
// 音频参数配置
// ========================================================

#define SAMPLE_RATE    16000   // 采样率：16kHz（适合语音）
#define RECORD_TIME    5       // 录音时间：5秒
#define TOTAL_SAMPLES  (SAMPLE_RATE * RECORD_TIME)  // 总样本数

// INMP441专用参数
#define INMP441_VOLUME  8      // INMP441的软件增益（硬件增益已由GAIN引脚设置）
#define DC_OFFSET_ALPHA 0.001f // DC偏移消除系数（用于高通滤波）

// ========================================================
// 全局变量声明
// ========================================================

int16_t audio_data[TOTAL_SAMPLES];  // 音频数据缓冲区
bool recording = false;             // 录音状态标志
bool playing = false;               // 播放状态标志
bool has_recorded = false;          // 是否录到有效声音的标志

float dc_offset = 0.0f;             // DC偏移量（用于消除直流分量）

// ========================================================
// 主程序：初始化
// ========================================================

void setup() {
    Serial.begin(115200);
    delay(3000);  // 等待串口初始化
    
    Serial.println("开始初始化INMP441录音机...");
    
    // 初始化按钮和LED引脚
    pinMode(BUTTON_PIN, INPUT_PULLUP);  // 按钮使用内部上拉电阻
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    
    // 重要：给INMP441足够的上电时间（需要500ms稳定时间）
    Serial.println("等待INMP441麦克风上电...");
    delay(500);
    
    // 初始化功放
    initSpeaker();
    
    Serial.println("\n系统就绪！按按钮开始录音");
    Serial.println("等待按钮按下...");
}

// ========================================================
// 主程序：循环处理
// ========================================================

void loop() {
    static bool last_button_state = HIGH;  // 保存上一次按钮状态
    bool current_button_state = digitalRead(BUTTON_PIN);
    
    // 检测按钮按下（下降沿触发）
    if (last_button_state == HIGH && current_button_state == LOW && !recording && !playing) {
        Serial.println("\n按钮按下，开始录音...");
        
        // 设置录音状态
        recording = true;
        digitalWrite(LED_PIN, HIGH);  // 录音时LED常亮
        has_recorded = false;
        
        // 执行录音
        record();
        
        // 录音完成
        recording = false;
        
        // 根据录音结果决定是否播放
        if (has_recorded) {
            Serial.println("录音完成，开始播放...");
            playing = true;
            play();
            playing = false;
        } else {
            Serial.println("录音失败，播放提示音");
            playErrorBeep();
        }
        
        digitalWrite(LED_PIN, LOW);
        Serial.println("完成！等待下一次录音...");
        delay(300);  // 防抖延迟
    }
    
    // 更新按钮状态
    last_button_state = current_button_state;
    delay(10);  // 降低CPU使用率
}

// ========================================================
// 功放初始化函数
// ========================================================

void initSpeaker() {
    // I2S配置结构体
    i2s_config_t cfg = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),  // 主模式+发送模式
        .sample_rate = SAMPLE_RATE,                           // 采样率
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,         // 16位采样
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,          // 左声道（单声道）
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,    // 标准I2S格式
        .intr_alloc_flags = 0,                                // 无中断
        .dma_buf_count = 4,                                   // DMA缓冲区数量
        .dma_buf_len = 256,                                   // 每个缓冲区长度
        .use_apll = false,                                    // 不使用APLL
        .tx_desc_auto_clear = true,                           // 自动清除发送描述符
        .fixed_mclk = 0                                       // 不固定主时钟
    };
    
    // 引脚配置结构体
    i2s_pin_config_t pins = {
        .bck_io_num = SPK_SCK_PIN,    // 位时钟引脚
        .ws_io_num = SPK_WS_PIN,      // 字选择（帧时钟）引脚
        .data_out_num = SPK_DATA_PIN, // 数据输出引脚
        .data_in_num = -1             // 无数据输入
    };
    
    // 安装I2S驱动
    i2s_driver_install(I2S_SPK, &cfg, 0, NULL);
    
    // 设置引脚
    i2s_set_pin(I2S_SPK, &pins);
    
    // 设置时钟
    i2s_set_clk(I2S_SPK, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
    
    Serial.println("MAX98357功放初始化完成");
}

// ========================================================
// 麦克风初始化函数
// ========================================================

void initMicrophone() {
    // INMP441使用标准I2S协议（非PDM）
    i2s_config_t cfg = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),  // 主模式+接收模式
        .sample_rate = SAMPLE_RATE,                           // 采样率
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,         // INMP441输出24位，用32位传输
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,         // 右声道（L/R引脚接地）
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,    // 标准I2S格式
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,             // 中断级别1
        .dma_buf_count = 8,                                   // DMA缓冲区数量
        .dma_buf_len = 512,                                   // 每个缓冲区长度
        .use_apll = true,                                     // 使用APLL获得更精确的时钟
        .tx_desc_auto_clear = false,                          // 不自动清除发送描述符
        .fixed_mclk = 0                                       // 不固定主时钟
    };
    
    // 引脚配置结构体
    i2s_pin_config_t pins = {
        .bck_io_num = MIC_SCK_PIN,    // 位时钟引脚
        .ws_io_num = MIC_WS_PIN,      // 字选择（帧时钟）引脚
        .data_out_num = -1,           // 无数据输出
        .data_in_num = MIC_DATA_PIN   // 数据输入引脚
    };
    
    // 先卸载驱动（防止重复初始化）
    i2s_driver_uninstall(I2S_MIC);
    
    // 安装I2S驱动
    esp_err_t err = i2s_driver_install(I2S_MIC, &cfg, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("麦克风驱动安装失败: %d\n", err);
        return;
    }
    
    // 设置引脚
    err = i2s_set_pin(I2S_MIC, &pins);
    if (err != ESP_OK) {
        Serial.printf("麦克风引脚设置失败: %d\n", err);
        return;
    }
    
    // 设置时钟
    i2s_set_clk(I2S_MIC, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_MONO);
    
    Serial.println("INMP441麦克风初始化完成");
    
    // 读取一些初始样本以稳定DC偏移
    int32_t dummy_buffer[64];
    size_t bytes_read;
    for (int i = 0; i < 10; i++) {
        i2s_read(I2S_MIC, dummy_buffer, sizeof(dummy_buffer), &bytes_read, 10);
    }
}

// ========================================================
// INMP441专用数据处理函数
// ========================================================

/**
 * @brief 处理INMP441的原始样本数据
 * @param raw_sample 原始32位I2S数据
 * @return 处理后的16位音频样本
 * 
 * INMP441输出格式说明：
 * - 24位有符号数据，左对齐在32位中
 * - 高24位是有效音频数据，低8位是0
 * - 需要转换为16位用于播放
 */
int16_t processINMP441Sample(int32_t raw_sample) {
    // 方法：取高24位并转换为16位
    int32_t sample_24bit = raw_sample >> 8;    // 移除低8位（右移8位）
    
    // 转换为16位（保留符号）
    int16_t sample_16bit = (int16_t)(sample_24bit >> 8);  // 再取高16位
    
    return sample_16bit;
}

// ========================================================
// 录音函数
// ========================================================

void record() {
    // 初始化麦克风
    initMicrophone();
    
    // INMP441需要更长的稳定时间
    Serial.println("等待INMP441稳定...");
    delay(100);
    
    // 缓冲区定义
    int32_t buffer[256];       // I2S接收缓冲区
    size_t bytes_read;         // 读取字节数
    int index = 0;             // 音频数据索引
    
    // 计时变量
    unsigned long start = millis();
    unsigned long last_print = start;
    
    Serial.println("录音开始（请说话，距离麦克风10-30cm）...");
    
    // 重置DC偏移
    dc_offset = 0.0f;
    
    // 统计参数
    int32_t sum_amplitude = 0;    // 幅度总和（用于计算平均值）
    int32_t max_amplitude = 0;    // 最大幅度
    int noise_samples = 0;        // 有声音的样本数
    
    // 录音主循环
    while (millis() - start < RECORD_TIME * 1000 && index < TOTAL_SAMPLES) {
        // 从I2S读取数据
        esp_err_t result = i2s_read(I2S_MIC, buffer, sizeof(buffer), &bytes_read, portMAX_DELAY);
        
        if (result == ESP_OK && bytes_read > 0) {
            int samples = bytes_read / 4;  // 每个样本4字节（32位）
            
            for (int i = 0; i < samples && index < TOTAL_SAMPLES; i++) {
                // 步骤1：处理INMP441样本
                int16_t sample = processINMP441Sample(buffer[i]);
                
                // 步骤2：消除DC偏移（高通滤波）
                dc_offset = dc_offset * (1.0f - DC_OFFSET_ALPHA) + sample * DC_OFFSET_ALPHA;
                int16_t dc_corrected = sample - (int16_t)dc_offset;
                
                // 步骤3：应用增益（INMP441输出较小，需要放大）
                int32_t amplified = (int32_t)dc_corrected * INMP441_VOLUME;
                
                // 步骤4：限制范围（防止削波）
                if (amplified > 32767) amplified = 32767;
                if (amplified < -32768) amplified = -32768;
                
                int16_t final_sample = (int16_t)amplified;
                audio_data[index] = final_sample;
                
                // 统计计算
                int16_t abs_sample = abs(final_sample);
                sum_amplitude += abs_sample;
                if (abs_sample > max_amplitude) max_amplitude = abs_sample;
                
                // 检测是否有有效声音（阈值：500）
                if (abs_sample > 500) {
                    has_recorded = true;
                    noise_samples++;
                }
                
                index++;
            }
        }
        
        // 显示进度（每500ms更新一次）
        if (millis() - last_print >= 500) {
            last_print = millis();
            int progress = (millis() - start) * 100 / (RECORD_TIME * 1000);
            Serial.printf("录音进度: %d%%\n", progress);
        }
    }
    
    Serial.printf("\n录音完成，采集了 %d 个样本\n", index);
    
    // 录音质量分析
    if (index > 0) {
        // 计算统计数据
        float avg_amplitude = (float)sum_amplitude / index;
        float noise_ratio = (float)noise_samples / index * 100.0f;
        
        Serial.printf("音频统计:\n");
        Serial.printf("  平均幅度: %.1f\n", avg_amplitude);
        Serial.printf("  最大幅度: %d\n", max_amplitude);
        Serial.printf("  有声音比例: %.1f%%\n", noise_ratio);
        Serial.printf("  DC偏移: %.1f\n", dc_offset);
        
        // 音量警告
        if (avg_amplitude < 50) {
            Serial.println("  警告：音量太小，请靠近麦克风说话");
        } else if (avg_amplitude > 20000) {
            Serial.println("  警告：音量太大，可能削波");
        }
        
        // 削波警告
        if (max_amplitude > 30000) {
            Serial.println("  检测到削波，建议减小增益");
        }
    }
    
    // 无效录音警告
    if (!has_recorded && max_amplitude < 100) {
        Serial.println("警告：未检测到有效声音信号！");
        Serial.println("可能的原因：");
        Serial.println("1. INMP441电源问题（需要3.3V稳定供电）");
        Serial.println("2. 麦克风距离太远（建议10-30cm）");
        Serial.println("3. 说话声音太小");
        Serial.println("4. 接线错误（检查SD、WS、SCK连接）");
    }
    
    // 卸载麦克风驱动
    i2s_driver_uninstall(I2S_MIC);
}

// ========================================================
// 播放函数
// ========================================================

void play() {
    size_t bytes_written;  // 写入字节数
    int index = 0;         // 播放索引
    
    unsigned long last_blink = millis();  // LED闪烁计时
    bool led_state = HIGH;                // LED状态
    
    Serial.print("播放中 ");
    
    // 播放主循环
    while (index < TOTAL_SAMPLES) {
        // 计算剩余样本数
        int remain = TOTAL_SAMPLES - index;
        int chunk = min(256, remain);  // 每次写入最多256个样本
        
        // 写入I2S
        i2s_write(I2S_SPK, &audio_data[index], chunk * sizeof(int16_t), &bytes_written, portMAX_DELAY);
        index += chunk;
        
        // LED闪烁（每200ms切换一次）
        if (millis() - last_blink > 200) {
            last_blink = millis();
            led_state = !led_state;
            digitalWrite(LED_PIN, led_state);
        }
        
        // 播放进度显示（每1600个样本显示一个#）
        if (index % 1600 == 0) {
            Serial.print("#");
        }
    }
    
    // 清空DMA缓冲区
    i2s_zero_dma_buffer(I2S_SPK);
    delay(100);  // 等待播放完成
    
    Serial.println(" 播放完成");
}

// ========================================================
// 错误提示音函数
// ========================================================

void playErrorBeep() {
    Serial.println("播放错误提示音");
    
    // 生成800Hz的正弦波提示音
    int16_t beep[16000];  // 1秒的提示音
    
    for (int i = 0; i < 16000; i++) {
        // 淡入淡出处理（前800ms淡入，后800ms淡出）
        float fade = 1.0f;
        if (i < 800) fade = i / 800.0f;
        if (i > 15200) fade = (16000 - i) / 800.0f;
        
        // 生成正弦波：800Hz，幅度8000
        beep[i] = 8000 * fade * sin(2 * 3.14159 * 800 * i / SAMPLE_RATE);
    }
    
    // 播放提示音
    size_t bytes_written;
    i2s_write(I2S_SPK, beep, sizeof(beep), &bytes_written, portMAX_DELAY);
    
    delay(100);  // 等待播放完成
    
    // 清空缓冲区
    i2s_zero_dma_buffer(I2S_SPK);
}
