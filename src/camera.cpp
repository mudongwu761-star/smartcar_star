#include "camera.h"
#include "GPIO.h"

// 全局变量定义
cv::VideoCapture cap;
std::mutex speedMutex;
double kp = 0;
double ki = 0;
double kd = 0;
int screenWidth, screenHeight;
int newWidth, newHeight;
int fb;
uint16_t *fb_buffer;
PwmController servo(1, 0);
bool streamCaptureRunning = false;
int saved_frame_count = 0;
std::mutex frameMutex;
cv::Mat pubframe;

double mortor_kp = 0;
double mortor_ki = 0.1;
double mortor_kd = 0;
double stop_kp = 0;
double stop_ki = 0.1;
double stop_kd = 0;
int sign = 0;
int stop_sign = 0;
// int banmaxian_num = 0;
double start_time = -15;

// 斑马线检测相关全局变量
std::atomic<int> banmaxian_num(0);  // 斑马线计数
std::atomic<bool> is_stopped{false};  // 停车状态标志
std::atomic<double> stop_start_time{0.0};  // 停车开始时间
std::atomic<bool> last_zebra_detected{false};  // 上次检测结果

// 在全局变量区域添加
std::atomic<double> zebra_cooldown_end{10.0};  // 冷却结束时间


#define calc_scale 2

// static PIDController BrakePID(2.0, 0.5, 1.0, 0.0, INCREMENTAL, 100.0); // 刹车PID控制器
// static PIDController SpeedPID(0, 0, 0, 0, INCREMENTAL, 100.0);  // 速度环（增量式）

// 斑马线检测函数
bool detectZebraCrossing(const cv::Mat& inputImage) {
    // 1. 参数定义
    const cv::Scalar lowerWhite(0, 0, 200);    // HSV白色下限
    const cv::Scalar upperWhite(180, 30, 255); // HSV白色上限
    const int MIN_STRIPE_COUNT = 3;            // 最小条纹数
    const int MIN_AREA = 50;                   // 最小区域面积
    const float MIN_ASPECT_RATIO = 3.0f;       // 最小长宽比
    const int MIN_WIDTH = 30;                  // 最小宽度
    const float SPACING_VARIANCE = 0.3f;       // 最大间距变化率

    // 2. 转换为HSV颜色空间
    cv::Mat hsvImage;
    cvtColor(inputImage, hsvImage, cv::COLOR_BGR2HSV);

    // 3. 创建白色区域掩膜
    cv::Mat whiteMask;
    inRange(hsvImage, lowerWhite, upperWhite, whiteMask);

    // 4. 形态学处理 - 增强水平条纹
    cv::Mat kernelH = getStructuringElement(cv::MORPH_RECT, cv::Size(15, 1));
    morphologyEx(whiteMask, whiteMask, cv::MORPH_CLOSE, kernelH);

    // 5. 查找轮廓
    std::vector<std::vector<cv::Point>> contours;
    findContours(whiteMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 6. 斑马线特征检测
    int stripeCount = 0;
    std::vector<cv::Rect> stripeRects;
    
    for (const auto& contour : contours) {
        double area = contourArea(contour);
        if (area < MIN_AREA) continue;
        
        cv::Rect rect = boundingRect(contour);
        float aspectRatio = (float)rect.width / rect.height;
        
        // 检查是否为长条形(宽度远大于高度)
        if (aspectRatio > MIN_ASPECT_RATIO && rect.width > MIN_WIDTH) {
            stripeCount++;
            stripeRects.push_back(rect);
        }
    }

    // 7. 检查条纹特征
    if (stripeCount >= MIN_STRIPE_COUNT) {
        // 按y坐标排序
        std::sort(stripeRects.begin(), stripeRects.end(), 
            [](const cv::Rect& a, const cv::Rect& b) { return a.y < b.y; });
        
        // 计算平均间距
        float avgGap = 0;
        for (size_t i = 1; i < stripeRects.size(); ++i) {
            avgGap += (stripeRects[i].y - stripeRects[i-1].y);
        }
        avgGap /= (stripeRects.size() - 1);
        
        // 检查间距均匀性
        bool isZebra = true;
        for (size_t i = 1; i < stripeRects.size(); ++i) {
            float gap = stripeRects[i].y - stripeRects[i-1].y;
            if (fabs(gap - avgGap) > avgGap * SPACING_VARIANCE) {
                isZebra = false;
                break;
            }
        }
        
        return isZebra;
    }
    
    return false;
}

// 在CameraHandler函数中的斑马线处理逻辑
void handleZebraCrossing(const cv::Mat& frame) {
    // 1. 检测斑马线
    bool current_zebra = detectZebraCrossing(frame);
    bool new_zebra_detected = current_zebra && !last_zebra_detected.load();
    const int STOP_TIME_SECONDS = 3;  // 停车时间(秒)

    // 2. 新检测到斑马线时的处理
    if (new_zebra_detected && !is_stopped) {
        is_stopped = true;
        stop_start_time = cv::getTickCount() / cv::getTickFrequency();
        
        std::lock_guard<std::mutex> lock(speedMutex);
        target_speed = 0.0;
        sign = 1;  // 设置停车标志
        
        printf("[%s] 检测到斑马线，停车%d秒\n", 
              getCurrentTime().c_str(), STOP_TIME_SECONDS);
        
        // 可选：触发语音提示
        int ret = wonderEchoSend(0xFF, 0x10);
        if (ret != 0) {
            printf("语音发送失败，错误码：%d\n", ret);
        }
    }

    // 3. 更新上次检测结果
    last_zebra_detected = current_zebra;

    // 4. 停车超时恢复
    if (is_stopped) {
        const double current_time = cv::getTickCount() / cv::getTickFrequency();
        if (current_time - stop_start_time >= STOP_TIME_SECONDS) {
            is_stopped = false;
            sign = 0;  // 清除停车标志
            banmaxian_num++;  // 增加斑马线计数
            
            const double new_speed = readDoubleFromFile(speed_file);
            {
                std::lock_guard<std::mutex> lock(speedMutex);
                target_speed = new_speed;
            }
            printf("[%s] 停车结束，恢复速度至%.1f\n", 
                  getCurrentTime().c_str(), new_speed);
        }
    }
}

int CameraInit(uint8_t camera_id, double dest_fps, int width, int height) {
    servo.setPeriod(3000000);
    servo.setDutyCycle(1500000);
    servo.enable();

    fb = open("/dev/fb0", O_RDWR);
    if (fb == -1) {
        std::cerr << "无法打开帧缓冲区设备" << std::endl;
        return -1;
    }

    struct fb_var_screeninfo vinfo;
    if (ioctl(fb, FBIOGET_VSCREENINFO, &vinfo) == -1) {
        std::cerr << "无法获取帧缓冲区信息" << std::endl;
        close(fb);
        return -1;
    }

    screenWidth = vinfo.xres;
    screenHeight = vinfo.yres;
    size_t fb_size = vinfo.yres_virtual * vinfo.xres_virtual * vinfo.bits_per_pixel / 8;

    fb_buffer = (uint16_t *)mmap(NULL, fb_size, PROT_READ | PROT_WRITE, MAP_SHARED, fb, 0);
    if (fb_buffer == MAP_FAILED) {
        std::cerr << "无法映射帧缓冲区到内存" << std::endl;
        close(fb);
        return -1;
    }

    cap.open(camera_id);
    if (!cap.isOpened()) {
        printf("无法打开摄像头\n");
        munmap(fb_buffer, fb_size);
        close(fb);
        return -1;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, -1);

    int cameraWidth = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int cameraHeight = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    printf("摄像头分辨率: %d x %d\n", cameraWidth, cameraHeight);

    double widthRatio = static_cast<double>(screenWidth) / cameraWidth;
    double heightRatio = static_cast<double>(screenHeight) / cameraHeight;
    double scale = std::min(widthRatio, heightRatio);

    newWidth = static_cast<int>(cameraWidth * scale);
    newHeight = static_cast<int>(cameraHeight * scale);
    printf("自适应分辨率: %d x %d\n", newWidth, newHeight);

    double fps = cap.get(cv::CAP_PROP_FPS);
    printf("Camera fps:%lf\n", fps);

    line_tracking_width = newWidth / calc_scale;
    line_tracking_height = newHeight / calc_scale;
    
    // line_tracking_width = 240;
    // line_tracking_height = 120;

    return static_cast<int>(1000.0 / std::min(fps, dest_fps));
}

void cameraDeInit(void) {
    cap.release();

    struct fb_var_screeninfo vinfo;
    if (ioctl(fb, FBIOGET_VSCREENINFO, &vinfo) == -1) {
        std::cerr << "无法获取帧缓冲区信息" << std::endl;
    } else {
        size_t fb_size = vinfo.yres_virtual * vinfo.xres_virtual * vinfo.bits_per_pixel / 8;
        munmap(fb_buffer, fb_size);
    }

    close(fb);
}

bool saveCameraImage(const cv::Mat& frame, const std::string &directory) {
    if (frame.empty()) {
        std::cerr << "Save Error: Frame is empty." << std::endl;
        return false;
    }

    std::ostringstream filename;
    filename << directory << "/image_" << std::setw(5) << std::setfill('0') << saved_frame_count << ".jpg";
    saved_frame_count++;
    return cv::imwrite(filename.str(), frame);
}

void streamCapture(void) {
    cv::Mat frame;
    while (streamCaptureRunning) {
        cap.read(frame);
        frameMutex.lock();
        pubframe = frame;
        frameMutex.unlock();
    }
}

PIDController ServoControl(1.0, 0.0, 2.0, 0.0, POSITION, 1250000);
// PIDController MotorControl(0, 0, 0, 0, INCREMENTAL, 1250000);

int CameraHandler(void) {
    cv::Mat resizedFrame;
    static std::atomic<bool> is_stopped{false};
    static std::atomic<double> stop_start_time{0.0};
    static std::atomic<bool> last_zebra_detected{false};

     // 帧率计算相关变量
     static double last_time = cv::getTickCount() / cv::getTickFrequency();
     static int frame_count = 0;
     static double fps = 0.0;

    frameMutex.lock();
    raw_frame = pubframe.clone();
    frameMutex.unlock();

    if (raw_frame.empty()) {
        printf("无法捕获图像\n");
        return -1;
    }

     // 帧率计算
     frame_count++;
     if (frame_count >= 10) {  // 每10帧计算一次帧率
         double current_time = cv::getTickCount() / cv::getTickFrequency();
         fps = frame_count / (current_time - last_time);
         printf("当前帧率: %.2f FPS\n", fps);
        //  std::cout << "stop_sign : " << stop_sign << std::endl;
        //  std::cout << "banmaxian_num : " << banmaxian_num << std::endl;
         frame_count = 0;
         last_time = current_time;
     }

    if (readFlag(saveImg_file)) {
        if (saveCameraImage(raw_frame, std::string("./image"))) {
            printf("图像%d已保存\n", saved_frame_count);
        } else {
            printf("图像保存失败\n");
            return -1;
        }
    }

    { image_main(); }

    // // 显示处理结果
    // if (readFlag(showImg_file)) {
    //     // 创建全黑背景的帧缓冲图像
    //     cv::Mat fbImage(screenHeight, screenWidth, CV_8UC3, cv::Scalar(0, 0, 0));
        
    //     // 尺寸调整和颜色空间转换
    //     cv::Mat processedFrame;
    //     cv::resize(track, processedFrame, cv::Size(newWidth, newHeight));
    //     cv::cvtColor(processedFrame, processedFrame, cv::COLOR_GRAY2BGR);

    //     // 车道线绘制（在原始方向）
    //     const int lineThickness = calc_scale;
    //     for (int y = 0; y < line_tracking_height; ++y) {
    //         // 计算缩放后坐标
    //         const int scaledY = y * calc_scale;
    //         const int scaledLeft = left_line[y] * calc_scale;
    //         const int scaledRight = right_line[y] * calc_scale;
    //         const int scaledMid = mid_line[y] * calc_scale;

    //         // 绘制车道标记（单点线段）
    //         cv::line(processedFrame, {scaledLeft, scaledY}, {scaledLeft, scaledY},
    //                 cv::Scalar(0, 0, 255), lineThickness);  // 左线-红色
    //         cv::line(processedFrame, {scaledRight, scaledY}, {scaledRight, scaledY},
    //                 cv::Scalar(0, 255, 0), lineThickness);   // 右线-绿色
    //         cv::line(processedFrame, {scaledMid, scaledY}, {scaledMid, scaledY},
    //                 cv::Scalar(255, 0, 0), lineThickness);   // 中线-蓝色
    //     }

    //     // 图像旋转180度
    //     cv::rotate(processedFrame, processedFrame, cv::ROTATE_180);

    //     // 居中显示处理后的图像
    //     const cv::Rect displayArea(
    //         (screenWidth - newWidth) / 2,    // x起始位置
    //         (screenHeight - newHeight) / 2,  // y起始位置
    //         newWidth,                        // 显示宽度
    //         newHeight                        // 显示高度
    //     );
    //     processedFrame.copyTo(fbImage(displayArea));

    //     // 转换到RGB565格式并更新帧缓冲
    //     convertMatToRGB565(fbImage, fb_buffer, screenWidth, screenHeight);
    // }

    // double dest_frame_duration1;
    // { // 计算帧率
    //     double dest_fps = readDoubleFromFile(destfps_file);
    //     double fps = cap.get(cv::CAP_PROP_FPS);
    //     printf("Camera fps:%lf\n", fps);

    //     // 计算每帧的延迟时间（us）
    //     dest_frame_duration1 = static_cast<double>(1000000 / std::min(fps, dest_fps));
    // }

    
    cv::Mat gray, binary;
    cvtColor(raw_frame, gray, cv::COLOR_BGR2GRAY);

     // 斑马线检测处理
    //  handleZebraCrossing(raw_frame);

    // 斑马线检测核心逻辑

      // 获取当前时间
      double current_time = cv::getTickCount() / cv::getTickFrequency();
    
      // 检查是否在冷却期内
      bool in_cooldown = current_time - start_time< zebra_cooldown_end.load();

    static bool first_run = true;
    if (first_run) {
        const int debug_y = gray.rows - std::max(20, gray.rows/8);
        const int debug_x1 = gray.cols * 0.15;
        const int debug_x2 = debug_x1 + 50;
        printf("[调试] 跑道灰度：%d，白线灰度：%d\n", 
               gray.at<uchar>(debug_y, debug_x1),
               gray.at<uchar>(debug_y, debug_x2));
        first_run = false;
    }

    const int height = gray.rows;
    const int width = gray.cols;
    const int scan_y = height - std::max(20, height/2);
    const int x_start = width * 0.1;
    const int x_end = width * 0.9;
    const int GRAY_THRESHOLD = 160;
    const int MIN_TRANSITIONS = 10;
    const int STOP_TIME_SECONDS = 3;

    if (!in_cooldown)
    {
    int total_transitions = 0;
    const int scan_lines = 5;
    for (int i = 0; i < scan_lines; ++i) 
    {
        int current_y = scan_y - i * 5;
        bool current_white = gray.at<uchar>(current_y, x_start) > GRAY_THRESHOLD;
        int line_transitions = 0;

        for (int x = x_start; x < x_end; x += 3) {
            bool pixel_white = gray.at<uchar>(current_y, x) > GRAY_THRESHOLD;
            if (pixel_white != current_white) 
            {
                line_transitions++;
                current_white = pixel_white;
            }
        }
        total_transitions += line_transitions;
    }
    
    const bool current_zebra = (total_transitions/scan_lines >= MIN_TRANSITIONS);
    const bool new_zebra_detected = current_zebra && !last_zebra_detected.load();
    

        
    
    if (new_zebra_detected && !is_stopped) 
    {
        is_stopped = true;
        stop_start_time = cv::getTickCount() / cv::getTickFrequency();
        
        std::lock_guard<std::mutex> lock(speedMutex);
        target_speed = 0.0;
        sign = 1;
        printf("[%s] 检测到斑马线（跳变次数：%d），停车%d秒\n", 
              getCurrentTime().c_str(), total_transitions/scan_lines, STOP_TIME_SECONDS);
        int ret = wonderEchoSend(0xFF, 0x10);
        if (ret != 0) {
            printf("语音发送失败，错误码：%d\n", ret);
        }
    }
    last_zebra_detected = current_zebra;
}

    if (is_stopped) 
    {
        // const double current_time = cv::getTickCount() / cv::getTickFrequency();
        if (current_time - stop_start_time >= STOP_TIME_SECONDS) 
        {
            is_stopped = false;
            sign = 0;
            const double new_speed = readDoubleFromFile(speed_file);
            {
                std::lock_guard<std::mutex> lock(speedMutex);
                target_speed = new_speed;
            }
            printf("[%s] 停车结束，恢复速度至%.1f\n", getCurrentTime().c_str(), new_speed);
            start_time = cv::getTickCount() / cv::getTickFrequency();
            banmaxian_num++;
        } else {
            return 0;
        }
    }

    // // 白线检测核心逻辑
    // // 1. 定义检测区域（底部水平带状区域）
    // cv::Rect roi(0, gray.rows - DETECTION_ROI_HEIGHT, gray.cols, DETECTION_ROI_HEIGHT);
    // cv::Mat detectionZone = gray(roi);
 
    // // 2. 二值化处理
    // cv::threshold(detectionZone, binary, WHITE_LINE_THRESHOLD, 255, cv::THRESH_BINARY);
 
    // // 3. 检测白线位置（找最上方的白色像素）
    // static std::atomic<bool> whiteLineDetected{false};
    // bool currentDetection = false;
 
    // for (int y = binary.rows - 1; y >= 0; --y) 
    // {
    //     uchar* row = binary.ptr<uchar>(y);
    //     for (int x = 0; x < binary.cols; ++x) 
    //     {
    //         if (row[x] == 255) {
    //             if ((binary.rows - y) <= STOP_DISTANCE_PX) 
    //             {
    //                 currentDetection = true;
    //                 break;
    //             }
    //         }
    //     }
    //     if (currentDetection) break;
    // }

    // // 4. 状态更新和停车控制
    // static auto stopStartTime = std::chrono::steady_clock::now();
    // if (currentDetection && !whiteLineDetected.load()) 
    // {
    //     whiteLineDetected = true;
    //     stopStartTime = std::chrono::steady_clock::now();
     
    //     std::lock_guard<std::mutex> lock(speedMutex);
    //     // target_speed = 0.0;
    //     stop_sign = 1;  // 设置停车标志
     
    //     printf("[%s] 检测到白线，触发停车\n", getCurrentTime().c_str());
     
    //     // 可选：触发语音提示
    //     // int ret = wonderEchoSend(0xFF, 0x11);
    //     // if (ret != 0) {
    //     //     printf("语音发送失败，错误码：%d\n", ret);
    //     // }
    // }

    // // 5. 停车超时恢复
    // if (whiteLineDetected.load()) 
    // {
    //     auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
    //         std::chrono::steady_clock::now() - stopStartTime).count();
     
    //     if (elapsed >= STOP_DURATION) 
    //     {
    //         whiteLineDetected = false;
    //         stop_sign = 0;  // 清除停车标志
         
    //         const double new_speed = readDoubleFromFile(speed_file);
    //         {
    //             std::lock_guard<std::mutex> lock(speedMutex);
    //             target_speed = new_speed;
    //         }
    //         printf("[%s] 停车结束，恢复行驶\n", getCurrentTime().c_str());
    //     }
    //  }


// 控制
    if (readFlag(start_file)) 
    {
        int foresee = readDoubleFromFile(foresee_file);
        if (mid_line[foresee] != 255) {
            ServoControl.setPID(kp, ki, kd);
            
            const double servo_input = mid_line[foresee / calc_scale] * calc_scale - newWidth / 2;

            // double servoduty = -ServoControl.updateservo(servo_input);
            // double servo_duty = Turn_Out_Filter(servoduty);
               
            double servo_duty = -ServoControl.updateservo(servo_input);
            
            servo_duty = std::clamp(servo_duty, -7.0, 7.0);
            int servo_mid = readDoubleFromFile(servo_mid_file);
            double servoduty_ns = (servo_duty/100) * servo.readPeriod() + servo_mid;

            servo.setDutyCycle(servoduty_ns);
            
            const double new_speed = readDoubleFromFile(speed_file);
            {
                std::lock_guard<std::mutex> lock(speedMutex);
                // target_speed = new_speed;
            }

            // double target_speed = readDoubleFromFile(speed_file);
        if(sign == 0)
        {
            // 使用PID控制每个电机（示例）
            for (int i = 0; i < 2; ++i) 
            {
                target_speed = readDoubleFromFile(speed_file);
                double edcoder_speed = -motorController[i]->encoderSpeed();
                double speed_error = target_speed - edcoder_speed;

                SpeedPID.setPID(mortor_kp, mortor_ki, mortor_kd);
                // PID控制
                double duty_adjustment = SpeedPID.updatemortor(speed_error);
            
                motorController[i]->updateduty(duty_adjustment);
                // motorController[i]->updateduty(15);
                // std::cout << "current_speed : " << current_speed << std::endl;
                // std::cout << "edcoder_speed : " << edcoder_speed << std::endl;
                // // std::cout << "last_duty : " << last_duty << std::endl;
                // std::cout << "duty_adjustment : " << duty_adjustment << std::endl;
            }
        }
        // else
        // {
        //     target_speed = 0.0;
        //     // 使用PID控制每个电机（示例）
        //     for (int i = 0; i < 2; ++i) 
        //     {
        //         motorController[i]->updateduty(0);
        // //         double edcoder_speed = -motorController[i]->encoderSpeed();
        // //         double speed_error = target_speed - edcoder_speed;

        // //         BrakePID.setPID(stop_kp, stop_ki, stop_kd);
        // //         // PID控制
        // //         double duty_stop = BrakePID.updatemortor(speed_error);
            
        // //         motorController[i]->updateduty(duty_stop);
        //     }
    // }

        // mortorEN.setValue(1);

        
        
    }
}

    return 0;
}

std::string getCurrentTime() 
{
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X");
    return ss.str();
}