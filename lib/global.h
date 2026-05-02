#ifndef GLOBAL_H
#define GLOBAL_H

#include <string>
#include <iostream>
#include <fstream>
#include <atomic>

const std::string kp_file = "./kp";
const std::string ki_file = "./ki";
const std::string kd_file = "./kd";

const std::string servo_median_file = "./servo_median";

const std::string mortor_kp_file = "./mortor_kp";
const std::string mortor_ki_file = "./mortor_ki";
const std::string mortor_kd_file = "./mortor_kd";

const std::string stop_kp_file = "./stop_kp";
const std::string stop_ki_file = "./stop_ki";
const std::string stop_kd_file = "./stop_kd";

const std::string start_file = "./start";
const std::string showImg_file = "./showImg";
const std::string destfps_file = "./destfps";
const std::string foresee_file = "./foresee";
const std::string saveImg_file = "./saveImg";
const std::string speed_file = "./speed";
const std::string servo_mid_file = "./servo_mid";
const std::string limit_file = "limit";
const std::string target_file = "taeget";

// 从文件读取双精度值
double readDoubleFromFile(const std::string &filename);

// 从文件中读取标志
bool readFlag(const std::string &filename);

extern std::atomic<double> PID_rotate;

extern double target_speed;

#endif
