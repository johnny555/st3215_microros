#include "st3215.hpp"
#include <cmath>
#include <memory>
#include <map>

#include <SCServo.h>
#include <math.h>

ST3215Controller::ST3215Controller() {
    jointIds = { JOINT_IDS };
    jointNames = { JOINT_NAMES };
    invertFlags = { INVERT_MOTOR };
}

bool ST3215Controller::initialize() {
    Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
    st.pSerial = &Serial1;
    
    if (!Serial1) {
        return false;
    }
    
    initializeNameMaps();
    return true;
}

bool ST3215Controller::moveMotor(int id, double position) {
    if (!isValidServoId(id)) {
        return false;
    }

    int encoderPosition = radiansToEncoder(position);
    st.RegWritePosEx(id, encoderPosition, SPEED, ACCELERATION);
    st.RegWriteAction();
    return true;
}

ServoData ST3215Controller::getFeedback(uint8_t servoId) {
    ServoData data;
    data.id = servoId;
    
    if (st.FeedBack(servoId) != -1) {
        data.isAvailable = true;
        data.position = encoderToRadians(st.ReadPos(-1));
        data.speed = encoderToRadians(st.ReadSpeed(-1));
        data.load = st.ReadLoad(-1);
        data.voltage = st.ReadVoltage(-1);
        data.current = st.ReadCurrent(-1);
        data.temperature = st.ReadTemper(-1);
        data.mode = st.ReadMode(servoId);
        data.name = jointIdToName[servoId];
    }
    
    return data;
}

// Utility functions
bool ST3215Controller::isValidServoId(uint8_t servoId) const {
    return std::find(jointIds.begin(), jointIds.end(), servoId) != jointIds.end();
}

void ST3215Controller::initializeNameMaps() {
    for (size_t i = 0; i < jointNames.size(); ++i) {
        jointNameToId[jointNames[i]] = jointIds[i];
        jointIdToName[jointIds[i]] = jointNames[i];
    }

}

double ST3215Controller::encoderToRadians(int position) {
    int relativePosition = position - HOME_POSITION;
    return (static_cast<double>(relativePosition) / MAX_ENCODER_VALUE) * 2 * M_PI;
}

int ST3215Controller::radiansToEncoder(double radians) {
    int encoderPosition = static_cast<int>((radians) * MAX_ENCODER_VALUE / (2 * M_PI));
    encoderPosition = (encoderPosition + HOME_POSITION) % static_cast<int>(MAX_ENCODER_VALUE);
    return encoderPosition;
}

// Core functionality
bool ST3215Controller::setMiddle(uint8_t servoId) {
    if (!isValidServoId(servoId)) return false;
    st.CalibrationOfs(servoId);
    return true;
}

bool ST3215Controller::setMiddleAll() {
    for (int id : jointIds) {
        if (!setMode(id, 0) || !setMiddle(id)) return false;
    }
    return true;
}

bool ST3215Controller::setMode(uint8_t servoId, uint8_t mode) {
    if (!isValidServoId(servoId)) return false;
    
    st.unLockEprom(servoId);
    if (mode == 0) {
        st.writeWord(servoId, 11, 4095);
    } else if (mode == 3) {
        st.writeWord(servoId, 11, 0);
    }
    st.writeByte(servoId, SMS_STS_MODE, mode);
    st.LockEprom(servoId);
    return true;
}

bool ST3215Controller::setId(uint8_t currentId, uint8_t newId) {
    if (!isValidServoId(currentId)) return false;
    
    st.unLockEprom(currentId);
    st.writeByte(currentId, SMS_STS_ID, newId);
    st.LockEprom(newId);
    return true;
}

bool ST3215Controller::setTorque(uint8_t servoId, bool enable) {
    if (!isValidServoId(servoId)) return false;
    st.EnableTorque(servoId, enable ? 1 : 0);
    return true;
}

bool ST3215Controller::stopServo(uint8_t servoId) {
    if (!isValidServoId(servoId)) return false;
    setTorque(servoId, false);
    delay(10);
    setTorque(servoId, true);
    return true;
}

std::vector<ServoData> ST3215Controller::getFeedbackAll() {
    std::vector<ServoData> allData;
    for (int id : jointIds) 
            allData.push_back(getFeedback(id));
    return allData;
}

int ST3215Controller::getAvailableServos() {
    int count = 0;
    for (int id : jointIds) {
        if (st.Ping(id) != -1) count++;
    }
    return count;
}

std::vector<uint8_t> ST3215Controller::getAvailableServoIds() {
    std::vector<uint8_t> availableIds;
    for (int id : jointIds) {
        if (st.Ping(id) != -1) {
            availableIds.push_back(static_cast<uint8_t>(id));
        }
    }
    return availableIds;
}

int ST3215Controller::getMotorId(const std::string& name) const {
    auto it = jointNameToId.find(name);
    return (it != jointNameToId.end()) ? it->second : -1;
}


