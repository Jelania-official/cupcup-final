#include <cstdlib>
#include <fstream>
#include <seumath/math.hpp>
#include "SimRobot.hpp"

using namespace std;
using namespace webots;
using namespace common::msg;

const char *Neck[2] = { "Neck", "Neck2" };
const char *LeftArmJoint[2] = { "LeftShoulder", "LeftElbow" };
const char *RightArmJoint[2] = { "RightShoulder", "RightElbow" };
const char *LeftLegJoint[6] = { "LeftLegX", "LeftLegY", "LeftLegZ", "LeftKnee", "LeftAnkleX", "LeftAnkleY" };
const char *RightLegJoint[6] = { "RightLegX", "RightLegY", "RightLegZ", "RightKnee", "RightAnkleX", "RightAnkleY" };
const double fall_thresh = 0.75;

SimRobot::SimRobot(std::string robot_name) : Robot(), robotName(robot_name)
{
    totalTime = 0;
    mTimeStep = getBasicTimeStep();
    mCamera = getCamera("Camera");
    mCamera->enable(5 * mTimeStep);
    mIMU = getInertialUnit("IMU");
    mIMU->enable(mTimeStep);
    mNeckYawSensor = getPositionSensor("NeckS");
    mNeckPitchSensor = getPositionSensor("Neck2S");
    mNeckYawSensor->enable(mTimeStep);
    mNeckPitchSensor->enable(mTimeStep);
    mLEDs.resize(2);
    mLEDs[0] = getLED("Led0");
    mLEDs[1] = getLED("Led1");

    fallType = ImuData::FALL_NONE;
    mInitYaw = 0.0;
    imuReset = true;
    mHAngles.yaw = 0.0;
    mHAngles.pitch = 0.0;
    ledTask.led1 = 1;
    ledTask.led2 = 1;

    mImagePublisher = std::make_shared<ImagePublisher>(robotName);
    mImuPublisher = std::make_shared<ImuPublisher>(robotName);
    mHeadPublisher = std::make_shared<HeadAnglePublisher>(robotName);
}

int SimRobot::myStep()
{
    mLEDs[0]->set(0xff0000*ledTask.led1);
    mLEDs[1]->set(0xff0000*ledTask.led2);
    setPositions();
    // Read sensors only AFTER the step that refreshes their buffers. Publishing
    // on the pre-step counter used the previous camera frame (80 ms old for a
    // 20 ms world step) while stamping it with the current simulation time.
    const int result = step(mTimeStep);
    if (result < 0) return result;
    checkFall();
    totalTime += mTimeStep;
    if (totalTime % (5 * mTimeStep) == 0) {
        mImagePublisher->Publish(mCamera->getImage(), mCamera->getWidth(),
            mCamera->getHeight(), getTime());
    }
    HeadAngles measuredHead;
    measuredHead.yaw = seumath::rad2deg(mNeckYawSensor->getValue());
    measuredHead.pitch = -seumath::rad2deg(mNeckPitchSensor->getValue());
    measuredHead.time = static_cast<uint32_t>(getTime() * 1000.0);
    mHeadPublisher->Publish(measuredHead);
    rclcpp::spin_some(mImagePublisher);
    rclcpp::spin_some(mImuPublisher);
    rclcpp::spin_some(mHeadPublisher);
    return result;
}

void SimRobot::checkFall()
{
    const double *rpy = mIMU->getRollPitchYaw();
    if (rpy[1] > fall_thresh)
        fallType = ImuData::FALL_BACKWARD;
    else if (rpy[1] < -fall_thresh)
        fallType = ImuData::FALL_FORWARD;
    else if (rpy[0] < -fall_thresh)
        fallType = ImuData::FALL_LEFT;
    else if (rpy[0] > fall_thresh)
        fallType = ImuData::FALL_RIGHT;
    else
        fallType = ImuData::FALL_NONE;
    ImuData imu;
    imu.yaw = seumath::rad2deg(rpy[2]);
    imu.pitch = seumath::rad2deg(rpy[1]);
    imu.roll = seumath::rad2deg(rpy[0]);
    imu.fall = fallType;
    imu.stamp = static_cast<uint32_t>(getTime() * 1000.0);
    mImuPublisher->Publish(imu);
}

void SimRobot::wait(int ms)
{
    double startTime = getTime();
    double s = (double)ms / 1000.0;
    while (s + startTime >= getTime())
        myStep();
}

void SimRobot::setPositions()
{
    Motor *motor;
    motor = getMotor(LeftLegJoint[0]);
    motor->setPosition(seumath::deg2rad(mAngles.left_hip_roll));
    motor = getMotor(LeftLegJoint[1]);
    motor->setPosition(seumath::deg2rad(mAngles.left_hip_pitch));
    motor = getMotor(LeftLegJoint[2]);
    motor->setPosition(seumath::deg2rad(-mAngles.left_hip_yaw));
    motor = getMotor(LeftLegJoint[3]);
    motor->setPosition(seumath::deg2rad(mAngles.left_knee));
    motor = getMotor(LeftLegJoint[4]);
    motor->setPosition(seumath::deg2rad(mAngles.left_ankle_roll));
    motor = getMotor(LeftLegJoint[5]);
    motor->setPosition(seumath::deg2rad(mAngles.left_ankle_pitch));

    motor = getMotor(RightLegJoint[0]);
    motor->setPosition(seumath::deg2rad(mAngles.right_hip_roll));
    motor = getMotor(RightLegJoint[1]);
    motor->setPosition(seumath::deg2rad(mAngles.right_hip_pitch));
    motor = getMotor(RightLegJoint[2]);
    motor->setPosition(seumath::deg2rad(-mAngles.right_hip_yaw));
    motor = getMotor(RightLegJoint[3]);
    motor->setPosition(seumath::deg2rad(mAngles.right_knee));
    motor = getMotor(RightLegJoint[4]);
    motor->setPosition(seumath::deg2rad(mAngles.right_ankle_roll));
    motor = getMotor(RightLegJoint[5]);
    motor->setPosition(seumath::deg2rad(mAngles.right_ankle_pitch));

    motor = getMotor(LeftArmJoint[0]);
    motor->setPosition(seumath::deg2rad(mAngles.left_shoulder));
    motor = getMotor(LeftArmJoint[1]);
    motor->setPosition(seumath::deg2rad(mAngles.left_elbow));
    motor = getMotor(RightArmJoint[0]);
    motor->setPosition(seumath::deg2rad(-mAngles.right_shoulder));
    motor = getMotor(RightArmJoint[1]);
    motor->setPosition(seumath::deg2rad(-mAngles.right_elbow));

    motor = getMotor(Neck[1]);
    motor->setPosition(seumath::deg2rad(-mHAngles.pitch));
    motor = getMotor(Neck[0]);
    motor->setPosition(seumath::deg2rad(mHAngles.yaw));
}
