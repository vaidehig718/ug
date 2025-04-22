#ifndef EBUS_VNS_LOGGER_H
#define EBUS_VNS_LOGGER_H

#include <iostream>
#include <fstream>
#include <ctime>

enum class LogLevel {
  Verbose,
  Debug,
  Info,
  Off,
  Warning,
  Error
};

class Logger {
public:
// Modified constructor
    explicit Logger(bool console_output = false) : console_output_(console_output) {}

    // Destructor
    ~Logger() {
        if (log_file_.is_open()) {
            log_file_.close();
        }
    }

    // Set the file path for logging
    void set_file_path(const std::string& filename) {
        if (log_file_.is_open()) {
            log_file_.close();
        }
        log_file_.open(filename, std::ios::app);
    }

    void log(LogLevel level, const std::string& message) {
        if (level >= log_level_threshold_) {
            std::time_t now = std::time(nullptr);
            std::string log_entry = "[" + timestamp(now) + "] " + level_to_string(level) + ": " + message;

            if (console_output_) {
                std::cout << log_entry << std::endl;
            }

            if (log_file_.is_open()) {
                log_file_ << log_entry << std::endl;
            }
        }
    }

    void set_log_level_threshold(LogLevel threshold) {
        log_level_threshold_ = threshold;
    }

private:
    std::ofstream log_file_;
    bool console_output_;
    LogLevel log_level_threshold_ = LogLevel::Info;

    std::string timestamp(std::time_t t)
    {
        char buffer[20];
        strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", std::localtime(&t));
        return buffer;
    }

    std::string level_to_string(LogLevel level)
    {
        switch (level) {
        case LogLevel::Off: return "OFF";
        case LogLevel::Info: return "INFO";
        case LogLevel::Debug: return "DEBUG";
        case LogLevel::Verbose: return "VERBOSE";
        case LogLevel::Warning: return "WARNING";
        case LogLevel::Error: return "ERROR";
        default: return "UNKNOWN";
        }
    }
};

extern Logger logger;
#endif //EBUS_VNS_LOGGER_H
