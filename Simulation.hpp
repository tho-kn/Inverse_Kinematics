#include "bvh-loader/BVHReader.h"
#include "Camera.hpp"
#include "bvh-loader/GlHelper/DrawHelper.h"

using namespace std;

static std::vector<std::vector<Segment *>> getEndSites(Segment * root);

class Simulation {
    public:
        static vector<string> splitString(string str);
        static Simulation *current();
        static void setCurrent(Simulation *window);
        void activateConsole();
        bool readFile(string filename);
        void initialize();

        void loadGlobalCoord();
        void drawGridPlane();

        static void DisplayEvent();
        static void KeyboardEvent(unsigned char key,int x,int y);
        static void MouseEvent(int button, int state, int x, int y);
        static void MotionEvent(int x, int y);
        static void ReshapeEvent(int w, int h);
        static void TimerEvent(int value);
        static Simulation *curr;

    private:
        std::unique_ptr<BVHReader> bvh = nullptr;

        GLfloat mousePosX, mousePosY;
        Camera *currView;
        Segment *toMove;

        std::vector<std::vector<Segment *>> endSites;
        int currEndSite;
        void prevSite();
        void nextSite();
        void nextFrame();
        void reload();

        unsigned timeStep = 30;
        bool moveObject = false;

        int width, height;

        bool leftButton = false;

        const Eigen::Vector3f initPos = Eigen::Vector3f(-300.0f, 200.0f, -300.0f);
        Camera mainCamera = Camera(initPos, Eigen::Vector3f(1.0f, 0.0f, 1.0f), Eigen::Vector3f(0.0f, 1.0f, 0.0f));
        Camera goalCamera = Camera(Eigen::Vector3f(0.0f, 100.0f, 0.0f), Eigen::Vector3f(-1.0f, 0.0f, -1.0f), Eigen::Vector3f(0.0f, 1.0f, 0.0f));

        void drawGoal();
        Eigen::Vector3f goalPosition();
        Eigen::Vector3f goalOrientation();

        void motion(int x, int y);
        void mouse(int button, int state, int x, int y);
        void display();
        void reshape(int w, int h);
        void timer(int unused);
        void keyboard(unsigned char key, int x, int y);
};
