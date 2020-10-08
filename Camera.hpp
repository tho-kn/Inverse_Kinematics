#pragma once
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/Geometry>

class Camera{
    public:
        enum direction {
            Forward,
            Backward,
            Left,
            Right
        };

        Camera(Eigen::Vector3f eye, Eigen::Vector3f ori, Eigen::Vector3f up);

        void rotate(float yaw, float pitch);
        void setUp(Eigen::Vector3f up);
        Eigen::Vector3f getEye();
        Eigen::Vector3f getOri();

        void moveTo(Eigen::Vector3f pos);
        void move(enum direction dir);
        void lookAt();

        void setSpeed(double speed);

    private:
        float yaw;
        float pitch;

        Eigen::Vector3f eye;
        Eigen::Vector3f ori;
        Eigen::Vector3f right;
        Eigen::Vector3f up;
        double speed;
};
