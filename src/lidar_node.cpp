/**
 * @file lidar_node.cpp
 * @brief ROS 2 版激光雷达驱动节点 (带OpenCV可视化 + 离群点滤波)
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <cerrno>
#include <limits>

// Linux 串口底层头文件
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <asm/termbits.h>

// OpenCV 头文件
#include <opencv2/opencv.hpp>

// --- 配置宏 ---
#define LIDAR_BAUDRATE 150000
#define RETRY_COUNT 3

class LidarNode : public rclcpp::Node
{
public:
    LidarNode() 
        : Node("lidar_node"), fd_(-1), is_shutdown_(false), last_point_angle_(0.0), scan_count_(0)
    {
        // 1. 声明并获取参数
        this->declare_parameter<std::string>("port_name", "/dev/ttyACM0");
        this->declare_parameter<std::string>("frame_id", "laser_frame");
        
        // 滤波相关参数
        this->declare_parameter<bool>("filter.enabled", true);          // 是否开启滤波
        this->declare_parameter<double>("filter.radius", 0.10);         // 搜索半径 (米)，默认10cm
        this->declare_parameter<int>("filter.min_neighbors", 2);        // 半径内最少邻居数 (包含自己)

        this->get_parameter("port_name", port_name_);
        this->get_parameter("frame_id", frame_id_);
        
        // 角度裁剪/有效范围过滤 (支持仅保留前方/上侧 180 度有效扇区: 270° ~ 90°)
        this->declare_parameter<bool>("angle_crop.enabled", true);
        this->declare_parameter<double>("angle_crop.min_deg", 270.0);
        this->declare_parameter<double>("angle_crop.max_deg", 90.0);

        // 获取滤波参数
        this->get_parameter("filter.enabled", filter_enabled_);
        this->get_parameter("filter.radius", filter_radius_);
        this->get_parameter("filter.min_neighbors", filter_min_neighbors_);

        // 获取角度裁剪参数
        this->get_parameter("angle_crop.enabled", angle_crop_enabled_);
        this->get_parameter("angle_crop.min_deg", angle_crop_min_deg_);
        this->get_parameter("angle_crop.max_deg", angle_crop_max_deg_);

        // 波特率参数 (默认 150000，部分雷达如 Delta-2A 为 230400 或 115200)
        this->declare_parameter<int>("baudrate", 150000);
        this->get_parameter("baudrate", baudrate_);

        // 2. 初始化发布者 (ROS 2 写法)
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
        fov_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("lidar_fov_marker", 10);
        full_scan_buffer_.reserve(1000);

        // 3. 初始化OpenCV可视化
        initVisualization();

        // 4. 打开并配置串口
        if (!openSerial(port_name_, baudrate_)) {
            RCLCPP_FATAL(this->get_logger(), "Failed to open serial port: %s", port_name_.c_str());
            exit(1);
        }

        // 5. 等待 USB-CDC 握手稳定，并连续发送启动指令 (A5 60)
        usleep(150000);
        for (int i = 0; i < 3; ++i) {
            sendCmd({0xA5, 0x60});
            usleep(50000);
        }

        RCLCPP_INFO(this->get_logger(), "Lidar started. Port: %s, Baud: %d", port_name_.c_str(), baudrate_);
        RCLCPP_INFO(this->get_logger(), "Filter Config -> Enabled: %s, Rad: %.2fm, Neighbors: %d", 
            filter_enabled_ ? "true" : "false", filter_radius_, filter_min_neighbors_);
        RCLCPP_INFO(this->get_logger(), "Angle Crop -> Enabled: %s, Range: [%.1f deg, %.1f deg]",
            angle_crop_enabled_ ? "true" : "false", angle_crop_min_deg_, angle_crop_max_deg_);
    }

    ~LidarNode()
    {
        shutdown();
        cv::destroyAllWindows();
    }

    /**
     * @brief 节点主循环，由外部 main 函数调用
     */
    void run_loop()
    {
        uint8_t buffer[1024];
        // 使用 Rate 控制循环频率
        rclcpp::Rate r(500); 

        while (rclcpp::ok() && !is_shutdown_)
        {
            int n = read(fd_, buffer, sizeof(buffer));
            if (n > 0)
            {
                for (int i = 0; i < n; i++)
                {
                    processByte(buffer[i]);
                }
            }
            else if (n < 0)
            {
                if (errno != EAGAIN) {
                    RCLCPP_WARN(this->get_logger(), "Serial read error: %s", strerror(errno));
                }
            }

            // OpenCV GUI 事件处理
            //cv::waitKey(1); 
            
            // ROS 2 处理回调
            rclcpp::spin_some(this->get_node_base_interface());
            
            r.sleep();
        }
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr fov_marker_pub_;
    std::string port_name_;
    std::string frame_id_;
    int baudrate_ = 150000;
    
    // 滤波控制变量
    bool filter_enabled_;
    double filter_radius_;
    int filter_min_neighbors_;

    // 角度裁剪变量
    bool angle_crop_enabled_ = true;
    double angle_crop_min_deg_ = 270.0;
    double angle_crop_max_deg_ = 90.0;

    int fd_; 
    bool is_shutdown_;

    // 扫描圈数计数器
    int scan_count_ = 0;

    // --- 解析相关变量 ---
    enum State {
        WAIT_HEADER1,
        WAIT_HEADER2,
        READ_META,
        READ_PAYLOAD
    };
    
    State state_ = WAIT_HEADER1;
    std::vector<uint8_t> packet_buffer_; 
    uint8_t current_lsn_ = 0;
    size_t target_payload_size_ = 0;

    // --- 点云处理相关变量 ---
    struct LidarPoint
    {
        double angle_rad; // 原代码这里叫 angle_red，建议改为 angle_rad
        double distance_m;
        // 辅助转笛卡尔坐标，用于加速计算
        double x; 
        double y;
    };
    std::vector<LidarPoint> full_scan_buffer_;
    double last_point_angle_;
    rclcpp::Time scan_start_time_;
    bool first_packet_of_scan_ = true;

    // --- OpenCV 可视化相关变量 ---
    cv::Mat map_image_;
    const int img_size_ = 800;    
    const double max_range_ = 3.0; 
    double px_scale_;             
    const std::string win_name_ = "Lidar Scan Monitor";

    // ================= 初始化与辅助 =================

    void initVisualization()
    {
        map_image_ = cv::Mat::zeros(img_size_, img_size_, CV_8UC3);
        px_scale_ = (img_size_ / 2.0) / max_range_;
    }

    void resetImage()
    {
        map_image_ = cv::Scalar(0, 0, 0);
        cv::line(map_image_, cv::Point(img_size_/2, 0), cv::Point(img_size_/2, img_size_), cv::Scalar(50, 50, 50), 1);
        cv::line(map_image_, cv::Point(0, img_size_/2), cv::Point(img_size_, img_size_/2), cv::Scalar(50, 50, 50), 1);
        
        // 在界面上显示滤波状态
        std::string status = filter_enabled_ ? "Filter: ON" : "Filter: OFF";
        cv::putText(map_image_, status, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

        if (angle_crop_enabled_) {
            cv::putText(map_image_, "FOV: Front 180 deg (Crop: ON)", cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 200, 255), 2);
            // 绘制盲区基线
            cv::line(map_image_, cv::Point(0, img_size_/2), cv::Point(img_size_, img_size_/2), cv::Scalar(0, 150, 255), 2);
        }
    }

    // ================= 串口底层操作 =================

    bool openSerial(const std::string& port, int baudrate)
    {
        fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Open Error: %s", strerror(errno));
            return false;
        }

        struct termios2 tio;
        if (ioctl(fd_, TCGETS2, &tio) != 0) {
            close(fd_);
            return false;
        }

        tio.c_cflag &= ~CBAUD;
        tio.c_cflag |= BOTHER;
        tio.c_ispeed = baudrate;
        tio.c_ospeed = baudrate;

        tio.c_cflag &= ~PARENB;
        tio.c_cflag &= ~CSTOPB;
        tio.c_cflag &= ~CSIZE;
        tio.c_cflag |= CS8;
        tio.c_cflag |= (CLOCAL | CREAD);

        tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tio.c_iflag &= ~(IXON | IXOFF | IXANY);
        tio.c_oflag &= ~OPOST;

        if (ioctl(fd_, TCSETS2, &tio) != 0) {
            close(fd_);
            return false;
        }

        // 显式拉高 DTR 和 RTS 控制线 (很多 CDC 串口驱动板以此作为电机启动使能信号)
        int status = 0;
        if (ioctl(fd_, TIOCMGET, &status) == 0) {
            status |= TIOCM_DTR | TIOCM_RTS;
            ioctl(fd_, TIOCMSET, &status);
        }

        ioctl(fd_, TCFLSH, TCIOFLUSH);
        return true;
    }

    void shutdown()
    {
        if (is_shutdown_) return;
        is_shutdown_ = true;

        if (fd_ != -1)
        {
            std::vector<uint8_t> stop_cmd = {0xA5, 0x00, 0xA5, 0x65, 0xA5, 0x65};
            write(fd_, stop_cmd.data(), stop_cmd.size());
            ioctl(fd_, TCSBRK, 1);
            close(fd_);
            fd_ = -1;
        }
    }

    void sendCmd(const std::vector<uint8_t>& cmd)
    {
        if (fd_ != -1) {
            write(fd_, cmd.data(), cmd.size());
        }
    }

    // ================= 协议解析状态机 =================

    void processByte(uint8_t byte)
    {
        switch (state_)
        {
        case WAIT_HEADER1:
            if (byte == 0xAA) {
                state_ = WAIT_HEADER2;
                packet_buffer_.clear();
                packet_buffer_.push_back(byte);
            }
            break;

        case WAIT_HEADER2:
            if (byte == 0x55) {
                state_ = READ_META;
                packet_buffer_.push_back(byte);
            } else if (byte == 0xAA) {
                state_ = WAIT_HEADER2; 
                packet_buffer_.clear();
                packet_buffer_.push_back(byte);
            } else {
                state_ = WAIT_HEADER1;
                packet_buffer_.clear();
            }
            break;

        case READ_META:
            packet_buffer_.push_back(byte);
            if (packet_buffer_.size() == 4) {
                current_lsn_ = packet_buffer_[3];
                target_payload_size_ = 4 + (current_lsn_ * 3);
                state_ = READ_PAYLOAD;
            }
            break;

        case READ_PAYLOAD:
            packet_buffer_.push_back(byte);
            if (packet_buffer_.size() == 4 + target_payload_size_) {
                parsePacket(packet_buffer_);
                state_ = WAIT_HEADER1; 
            }
            break;
        }
    }

    // ================= 点云转换算法 =================
    
    inline uint16_t bytesToUint16(const std::vector<uint8_t> &data, size_t index)
    {
        return (static_cast<uint16_t>(data[index + 1]) << 8) | data[index];
    }

    void parsePacket(const std::vector<uint8_t> &packet_data)
    {
        if (first_packet_of_scan_) {
            scan_start_time_ = this->now(); 
            first_packet_of_scan_ = false;
        }

        uint8_t lsn = packet_data[3];
        if (lsn == 0) return;

        uint16_t fsangle_raw = bytesToUint16(packet_data, 4);
        uint16_t lsangle_raw = bytesToUint16(packet_data, 6);

        double angle_start_deg = static_cast<double>(fsangle_raw >> 1) / 64.0;
        double angle_end_deg = static_cast<double>(lsangle_raw >> 1) / 64.0;

        double diff_angle_deg = 0.0;
        if (lsn > 1) {
            diff_angle_deg = angle_end_deg - angle_start_deg;
            if (diff_angle_deg < 0) {
                diff_angle_deg += 360.0;
            }
        }

        for (int i = 0; i < lsn; ++i)
        {
           size_t offset = 8 + i * 3;
           if (offset + 1 >= packet_data.size()) break; 

           uint16_t dist_raw = bytesToUint16(packet_data, offset);
           
           double distance_m = static_cast<double>(dist_raw) / 4.0 / 1000.0;
           double distance_mm = static_cast<double>(dist_raw) / 4.0;

           double angle_deg = angle_start_deg;
           if (lsn > 1) {
               angle_deg = (diff_angle_deg / (lsn - 1)) * i + angle_start_deg;
           }

           double angle_correct_deg = 0.0;
           if (distance_mm != 0) {
               double numerator = 21.8 * (155.3 - distance_mm);
               double denominator = 155.3 * distance_mm;
               double angle_correct_rad = std::atan(numerator / denominator);
               angle_correct_deg = angle_correct_rad * 180.0 / M_PI;
           }

           double final_angle_deg = angle_deg + angle_correct_deg;
           final_angle_deg = std::fmod(final_angle_deg, 360.0);
           if (final_angle_deg < 0) final_angle_deg += 360.0;

           double angle_rad = M_PI * final_angle_deg / 180.0;

            if (distance_m > 0.01) 
            {
                // 检测过零 (一圈结束)
                if (angle_rad < last_point_angle_ - M_PI) 
                {
                    if (scan_count_ > 0) {
                        // 在发布前，调用发布函数，发布函数内部会进行滤波
                        publishScan();
                        //cv::imshow(win_name_, map_image_);
                    } else {
                        RCLCPP_INFO(this->get_logger(), "Skipping first partial scan...");
                    }
                    
                    scan_count_++;
                    resetImage();
                    full_scan_buffer_.clear();
                    first_packet_of_scan_ = true;
                    scan_start_time_ = this->now();
                }
                last_point_angle_ = angle_rad;

                // 角度裁剪过滤: 判断是否处于有效扇区 (默认上侧左右180°: [270°, 360°] U [0°, 90°])
                bool is_valid_angle = true;
                if (angle_crop_enabled_) {
                    if (angle_crop_min_deg_ > angle_crop_max_deg_) {
                        is_valid_angle = (final_angle_deg >= angle_crop_min_deg_ || final_angle_deg <= angle_crop_max_deg_);
                    } else {
                        is_valid_angle = (final_angle_deg >= angle_crop_min_deg_ && final_angle_deg <= angle_crop_max_deg_);
                    }
                }

                if (!is_valid_angle) {
                    continue;
                }

                // 注意：这里OpenCV画的是原始数据（包含噪点），因为滤波是整圈完成后才进行的。
                if (distance_m <= max_range_) 
                {
                    int center_xy = img_size_ / 2;
                    int px = center_xy + static_cast<int>(distance_m * px_scale_ * std::sin(angle_rad));
                    int py = center_xy - static_cast<int>(distance_m * px_scale_ * std::cos(angle_rad));

                    if (px >= 0 && px < img_size_ && py >= 0 && py < img_size_) {
                        cv::circle(map_image_, cv::Point(px, py), 1, cv::Scalar(0, 0, 255), -1);
                    }
                }

                LidarPoint p;
                p.angle_rad = angle_rad;
                p.distance_m = distance_m;
                // 预先计算坐标，加速后续滤波
                p.x = distance_m * std::cos(angle_rad);
                p.y = distance_m * std::sin(angle_rad);
                
                full_scan_buffer_.push_back(p);
            }
        }
    }

    /**
     * @brief 半径滤波算法
     * 遍历所有点，如果某点在 radius 范围内的邻居数量少于 min_neighbors，则视为噪点剔除
     */
    void removeOutliers(std::vector<LidarPoint>& points)
    {
        if (points.empty()) return;

        std::vector<LidarPoint> clean_points;
        clean_points.reserve(points.size());

        double r2 = filter_radius_ * filter_radius_; // 比较距离的平方，避免开根号

        // 由于一圈只有几百个点，双重循环暴力匹配也是很快的 (400*400 = 16万次操作，对CPU可忽略)
        for (size_t i = 0; i < points.size(); ++i) {
            int neighbors = 0;
            const auto& p1 = points[i];

            for (size_t j = 0; j < points.size(); ++j) {
                // 距离平方计算
                double dx = p1.x - points[j].x;
                double dy = p1.y - points[j].y;
                if ((dx*dx + dy*dy) < r2) {
                    neighbors++;
                }
                // 如果有足够的邻居，直接通过，不需要数完
                if (neighbors >= filter_min_neighbors_) break;
            }

            // 如果邻居足够（注意：neighbors包含点自己，所以至少是1），则保留
            if (neighbors >= filter_min_neighbors_) {
                clean_points.push_back(p1);
            }
        }

        // 替换原始数据
        points = std::move(clean_points);
    }

    void publishFovMarker(const rclcpp::Time& stamp)
    {
        if (!fov_marker_pub_ || !angle_crop_enabled_) return;

        // 1. 半透明扇形区域 (TRIANGLE_LIST) 显示 180° 限定区块
        visualization_msgs::msg::Marker fan;
        fan.header.stamp = stamp;
        fan.header.frame_id = frame_id_;
        fan.ns = "restricted_fov_zone";
        fan.id = 0;
        fan.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        fan.action = visualization_msgs::msg::Marker::ADD;
        fan.scale.x = 1.0;
        fan.scale.y = 1.0;
        fan.scale.z = 1.0;
        fan.color.r = 0.0f;
        fan.color.g = 0.9f;
        fan.color.b = 0.5f;
        fan.color.a = 0.20f; // 半透明高亮显示限定区块

        const double visual_r = 3.0; // 区块半径 3.0 米
        const int SEG = 36;
        double step = M_PI / SEG;

        geometry_msgs::msg::Point o;
        o.x = 0; o.y = 0; o.z = 0;

        for (int i = 0; i < SEG; ++i) {
            double a1 = -M_PI / 2.0 + i * step;
            double a2 = a1 + step;
            geometry_msgs::msg::Point p1, p2;
            p1.x = visual_r * std::cos(a1); p1.y = visual_r * std::sin(a1); p1.z = 0;
            p2.x = visual_r * std::cos(a2); p2.y = visual_r * std::sin(a2); p2.z = 0;
            fan.points.push_back(o);
            fan.points.push_back(p1);
            fan.points.push_back(p2);
        }
        fov_marker_pub_->publish(fan);

        // 2. 扇形外轮廓边界线 (LINE_STRIP)
        visualization_msgs::msg::Marker border;
        border.header.stamp = stamp;
        border.header.frame_id = frame_id_;
        border.ns = "restricted_fov_zone";
        border.id = 1;
        border.type = visualization_msgs::msg::Marker::LINE_STRIP;
        border.action = visualization_msgs::msg::Marker::ADD;
        border.scale.x = 0.025; // 边界线宽 2.5cm
        border.color.r = 0.0f;
        border.color.g = 1.0f;
        border.color.b = 0.8f;
        border.color.a = 0.95f;

        border.points.push_back(o);
        for (int i = 0; i <= SEG; ++i) {
            double a = -M_PI / 2.0 + i * step;
            geometry_msgs::msg::Point p;
            p.x = visual_r * std::cos(a); p.y = visual_r * std::sin(a); p.z = 0;
            border.points.push_back(p);
        }
        border.points.push_back(o);
        fov_marker_pub_->publish(border);
    }

    void publishScan()
    {
        if (full_scan_buffer_.empty()) return;

        // 在转换为 ROS 消息前，先执行滤波
        if (filter_enabled_) {
            removeOutliers(full_scan_buffer_);
        }

        sensor_msgs::msg::LaserScan scan;
        rclcpp::Time stamp = (scan_start_time_.nanoseconds() == 0) ? this->now() : scan_start_time_;
        scan.header.stamp = stamp;
        scan.header.frame_id = frame_id_;
        scan.range_min = 0.05;
        scan.range_max = 8.0;

        if (angle_crop_enabled_) {
            // 纯粹的 180° 前向扇区 LaserScan: [-90°, +90°] (完全限定于有效区块内)
            scan.angle_min = -M_PI / 2.0;
            scan.angle_max = M_PI / 2.0;
            const int SCAN_SIZE = 360;
            scan.angle_increment = M_PI / SCAN_SIZE;
            scan.ranges.assign(SCAN_SIZE, std::numeric_limits<float>::infinity());

            for (const auto& point : full_scan_buffer_) {
                double raw_angle = point.angle_rad;
                double ros_angle = 0.0;
                if (raw_angle >= 1.5 * M_PI) {
                    ros_angle = 2.0 * M_PI - raw_angle; // [0, +PI/2] (左)
                } else if (raw_angle <= 0.5 * M_PI) {
                    ros_angle = -raw_angle;              // [-PI/2, 0] (右)
                } else {
                    continue;
                }

                int index = static_cast<int>((ros_angle - scan.angle_min) / scan.angle_increment);
                if (index >= 0 && index < SCAN_SIZE) {
                    if (scan.ranges[index] == std::numeric_limits<float>::infinity() || point.distance_m < scan.ranges[index]) {
                        scan.ranges[index] = static_cast<float>(point.distance_m);
                    }
                }
            }
            scan.scan_time = 1.0 / 7.0;
            scan.time_increment = scan.scan_time / SCAN_SIZE;
        } else {
            // 全向 360° 模式
            scan.angle_min = 0.0;
            scan.angle_max = 2.0 * M_PI;
            const int SCAN_SIZE = 720;
            scan.angle_increment = (2.0 * M_PI) / SCAN_SIZE;
            scan.ranges.assign(SCAN_SIZE, std::numeric_limits<float>::infinity());

            for (const auto& point : full_scan_buffer_) {
                double raw_angle = point.angle_rad;
                double index_angle = 2.0 * M_PI - raw_angle;
                if (index_angle >= 2.0 * M_PI) index_angle -= 2.0 * M_PI;
                if (index_angle < 0) index_angle += 2.0 * M_PI;
                int index = static_cast<int>(index_angle / scan.angle_increment);
                if (index >= 0 && index < SCAN_SIZE) {
                    if (scan.ranges[index] == std::numeric_limits<float>::infinity() || point.distance_m < scan.ranges[index]) {
                        scan.ranges[index] = static_cast<float>(point.distance_m);
                    }
                }
            }
            scan.scan_time = 1.0 / 7.0;
            scan.time_increment = scan.scan_time / SCAN_SIZE;
        }

        scan_pub_->publish(scan);
        publishFovMarker(stamp);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarNode>();
    try {
        node->run_loop();
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}