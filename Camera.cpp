#include "Camera.hpp"
#include <cmath>
#include <algorithm>
#define PI 3.14159265

Camera::Camera(Eigen::Vector3f eye, Eigen::Vector3f ori, Eigen::Vector3f up = Eigen::Vector3f(0.0f, 1.0f, 0.0f)):
    eye(eye), ori(ori), up(up), right(ori.cross(up).normalized()), speed(10.0){
    pitch = acos(-ori[1]);
    yaw = atan2(ori[0]/sin(pitch), -ori[2]/sin(pitch));
};

Eigen::Vector3f Camera::getEye(){
    return this->eye;
}

Eigen::Vector3f Camera::getOri(){
    return this->ori;
}

void Camera::rotate(float dyaw, float dpitch){
    pitch += dpitch;
    pitch = pitch > PI ? PI : (pitch > 0 ? pitch : 0);
    yaw -= dyaw;
    this->ori[0] = sin(yaw)*sin(pitch);
    this->ori[1] = -cos(pitch);
    this->ori[2] = -cos(yaw)*sin(pitch);

    this->ori.normalize();
    this->right = this->ori.cross(up).normalized();
}

void Camera::setSpeed(double speed){
    this->speed = speed;
}

void Camera::setUp(Eigen::Vector3f up){
    this->up = up;
}

void Camera::moveTo(Eigen::Vector3f pos){
    this->eye = pos;
}

void Camera::move(enum direction dir){
    switch (dir){
        case Forward:
            eye += ori * speed;
            break;

        case Backward:
            eye -= ori * speed;
            break;

        case Right:
            eye += right * speed;
            break;

        case Left:
            eye -= right * speed;
            break;

        default:
            break;
    }
}

void Camera::lookAt(){
	gluLookAt(eye[0], eye[1], eye[2], eye[0]+ori[0], eye[1]+ori[1], eye[2]+ori[2], up[0], up[1], up[2]);
}