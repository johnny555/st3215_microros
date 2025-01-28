#ifndef CONFIG_ST3215_HPP
#define CONFIG_ST3215_HPP 

#include <vector>
#include <map>
#include <string>
#include <cstdint>
#include <SCServo.h>


// the uart used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19

// Configuration constants
constexpr double MAX_ENCODER_VALUE = 4096.0;
constexpr int HOME_POSITION = 2048;
constexpr uint16_t SERVO_INIT_ACC = 100;
constexpr uint16_t SERVO_MAX_SPEED = 4000;
constexpr uint16_t SERVO_INIT_SPEED = 2000;

struct ServoData {
    bool isAvailable = false;
    int id = -1;
    std::string name;
    double position = 0.0;
    double speed = 0.0;
    int16_t load = 0;
    uint8_t voltage = 0;
    int current = 0;
    int16_t temperature = 0;
    int16_t mode = 0;
};

class ST3215Controller {
public:
    ST3215Controller();
    ~ST3215Controller() = default;

    // Initialization
    bool initialize();
    bool setMiddle(uint8_t servoId);
    bool setMiddleAll();
    
    // Motor control
    bool moveMotor(int id, double position);
    bool setMode(uint8_t servoId, uint8_t mode);
    bool setId(uint8_t currentId, uint8_t newId);
    bool setTorque(uint8_t servoId, bool enable);
    bool stopServo(uint8_t servoId);

    // Feedback and status
    ServoData getFeedback(uint8_t servoId);
    std::vector<ServoData> getFeedbackAll();
    int getAvailableServos() ;
    std::vector<uint8_t> getAvailableServoIds() ;
    int getMotorId(const std::string& name) const;

    // Utility functions
    static double encoderToRadians(int position);
    static int radiansToEncoder(double radians);

    std::vector<int> jointIds;
    std::vector<std::string> jointNames;

private:

    void initializeNameMaps();
    bool isValidServoId(uint8_t servoId) const;

    SMS_STS st;


    std::vector<bool> invertFlags;
    std::map<std::string, int> jointNameToId;
    std::map<int, std::string> jointIdToName;

};


#endif // CONFIG_ST3215_HPP

