#pragma once
namespace Nao{
constexpr int n_bodynode = 83;
constexpr int n_dof = 28;
constexpr int n_vdof = 6;
constexpr int n_adof = 22;
}   // namespace Nao

namespace NaoBodyNode {
constexpr int basePosX = 0;
constexpr int basePosY = 1;
constexpr int basePosZ = 2;
constexpr int baseRotZ = 3;
constexpr int baseRotY = 4;
constexpr int torso = 5;
constexpr int ImuTorsoAccelerometer_frame = 6;
constexpr int ChestButton_frame = 7;
constexpr int ImuTorsoGyrometer_frame = 8;
constexpr int Neck = 9;
constexpr int Head = 10;
constexpr int CameraBottom_frame = 11;
constexpr int CameraBottom_optical_frame = 12;
constexpr int CameraTop_frame = 13;
constexpr int CameraTop_optical_frame = 14;
constexpr int HeadTouchFront_frame = 15;
constexpr int HeadTouchMiddle_frame = 16;
constexpr int HeadTouchRear_frame = 17;
constexpr int LInfraRed_frame = 18;
constexpr int RInfraRed_frame = 19;
constexpr int gaze = 20;
constexpr int LPelvis = 21;
constexpr int LHip = 22;
constexpr int LThigh = 23;
constexpr int LTibia = 24;
constexpr int LAnklePitch = 25;
constexpr int l_ankle = 26;
constexpr int LFootBumperLeft_frame = 27;
constexpr int LFootBumperRight_frame = 28;
constexpr int LFsrFL_frame = 29;
constexpr int LFsrFR_frame = 30;
constexpr int LFsrRL_frame = 31;
constexpr int LFsrRR_frame = 32;
constexpr int l_sole = 33;
constexpr int LShoulder = 34;
constexpr int LBicep = 35;
constexpr int LElbow = 36;
constexpr int LForeArm = 37;
constexpr int l_wrist = 38;
constexpr int LFinger11_link = 39;
constexpr int LFinger12_link = 40;
constexpr int LFinger13_link = 41;
constexpr int LFinger21_link = 42;
constexpr int LFinger22_link = 43;
constexpr int LFinger23_link = 44;
constexpr int l_gripper = 45;
constexpr int LHandTouchBack_frame = 46;
constexpr int LHandTouchLeft_frame = 47;
constexpr int LHandTouchRight_frame = 48;
constexpr int LThumb1_link = 49;
constexpr int LThumb2_link = 50;
constexpr int RPelvis = 51;
constexpr int RHip = 52;
constexpr int RThigh = 53;
constexpr int RTibia = 54;
constexpr int RAnklePitch = 55;
constexpr int r_ankle = 56;
constexpr int RFootBumperLeft_frame = 57;
constexpr int RFootBumperRight_frame = 58;
constexpr int RFsrFL_frame = 59;
constexpr int RFsrFR_frame = 60;
constexpr int RFsrRL_frame = 61;
constexpr int RFsrRR_frame = 62;
constexpr int r_sole = 63;
constexpr int RShoulder = 64;
constexpr int RBicep = 65;
constexpr int RElbow = 66;
constexpr int RForeArm = 67;
constexpr int r_wrist = 68;
constexpr int RFinger11_link = 69;
constexpr int RFinger12_link = 70;
constexpr int RFinger13_link = 71;
constexpr int RFinger21_link = 72;
constexpr int RFinger22_link = 73;
constexpr int RFinger23_link = 74;
constexpr int r_gripper = 75;
constexpr int RHandTouchBack_frame = 76;
constexpr int RHandTouchLeft_frame = 77;
constexpr int RHandTouchRight_frame = 78;
constexpr int RThumb1_link = 79;
constexpr int RThumb2_link = 80;
constexpr int LSonar_frame = 81;
constexpr int RSonar_frame = 82;
}  // namespace NaoBodyNode

namespace NaoDoF {
constexpr int basePosX = 0;
constexpr int basePosY = 1;
constexpr int basePosZ = 2;
constexpr int baseRotZ = 3;
constexpr int baseRotY = 4;
constexpr int baseRotX = 5;
constexpr int HeadYaw = 6;
constexpr int HeadPitch = 7;
constexpr int LHipYawPitch = 8;
constexpr int LHipRoll = 9;
constexpr int LHipPitch = 10;
constexpr int LKneePitch = 11;
constexpr int LAnklePitch = 12;
constexpr int LAnkleRoll = 13;
constexpr int LShoulderPitch = 14;
constexpr int LShoulderRoll = 15;
constexpr int LElbowYaw = 16;
constexpr int LElbowRoll = 17;
constexpr int RHipYawPitch = 18;
constexpr int RHipRoll = 19;
constexpr int RHipPitch = 20;
constexpr int RKneePitch = 21;
constexpr int RAnklePitch = 22;
constexpr int RAnkleRoll = 23;
constexpr int RShoulderPitch = 24;
constexpr int RShoulderRoll = 25;
constexpr int RElbowYaw = 26;
constexpr int RElbowRoll = 27;
}  // namespace NaoDoF

namespace NaoAux {
constexpr double servo_rate = 0.001;
}
