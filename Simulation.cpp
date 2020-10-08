#include <string>
#include <iostream>
#include "Simulation.hpp"
#include "InverseKinematics.hpp"
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

using namespace std;

Simulation *Simulation::curr = nullptr;

void Simulation::initialize(){
	goalCamera.moveTo(bvh->getRoot()->getOffset().cast<float>() - goalCamera.getOri() * 50);
	endSites[currEndSite].back()->setColor(Eigen::Vector3d(0.0, 1.0, 1.0));
	currView = &mainCamera;
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(800, 600);
	glutInitWindowPosition(50, 0);
	glutCreateWindow("Inverse Kinematics Test");
	glutReshapeFunc(Simulation::ReshapeEvent);
	glutDisplayFunc(Simulation::DisplayEvent);
	glutTimerFunc(timeStep, Simulation::TimerEvent, 0);
	glutKeyboardFunc(Simulation::KeyboardEvent);
	glutMouseFunc(Simulation::MouseEvent);
	glutMotionFunc(Simulation::MotionEvent);
}

vector<string> Simulation::splitString(string str){
    vector<string> result; 
    istringstream iss(str); 
    for(std::string s; iss >> s; ) 
        result.push_back(s);
    return result;
}

static std::vector<std::vector<Segment *>> getEndSites(Segment * root){
	/*
		Get the list of endsites with the list of corresponding joints connected to it.
		BVHReader object will have control of Segment objects, ignore memory usage
	*/
	std::vector<std::vector<Segment *>> endSites;
    std::vector<std::pair<Segment *, int>> segQ;
	segQ.push_back(std::pair<Segment *, int>(root, 0));

    while(segQ.size() > 0){
        auto pr = segQ.back();
		Segment *curr = pr.first;
		int segCount = pr.second;
        segQ.pop_back();

		/* End Site case */
		if (curr->numSub() == 0){
			std::vector<Segment *> endPath;
			for (auto joint : segQ){
				endPath.push_back(joint.first);
			}
			endPath.push_back(curr);
			endSites.push_back(endPath);

			continue;
		}

        int segs = curr->numSub();
        if (segCount < segs){
            segQ.push_back(std::pair<Segment *, int>(curr, segCount + 1));
			segQ.push_back(std::pair<Segment *, int>(curr->getSeg(segCount), 0));
        }
    }

	return endSites;
}

void Simulation::activateConsole(){
    /*
        Get control of main data and manipulate it, and return it
    */
    cout << endl << "// Console Interpreter is enabled //" << endl;

    while (true){
        cout << ">> ";
        string line;
        getline(cin, line);

        int spacepos = line.find(' ');
        string cmd = line.substr(0, spacepos);
        string args = line.substr(spacepos + 1, line.length() - spacepos - 1);

        if (cmd == "read"){
            readFile(args);
        }
		else if (cmd == "reload") reload();
        else if (cmd == "close") break;
        else if (cmd == "quit") break;
        else if (cmd == "exit") exit(0);

	    glutPostRedisplay();
    }

    cout << "// Console Interpreter is disabled //" << endl << endl;

    return;
}

bool Simulation::readFile(string filename){
    unique_ptr<BVHReader> newBvh (new BVHReader(filename));

    if (!newBvh->loadFile()){
        std::cout << "Failed to load .bvh file" << std::endl;
        return false;
    }

    std::cout << "Loaded " << filename << " successfully" << std::endl;

    bvh = move(newBvh);
	Simulation::setCurrent(this);

	endSites = getEndSites(bvh->getRoot());

	return true;
}

void Simulation::loadGlobalCoord()
{
	glLoadIdentity();
	currView->lookAt();
}

Simulation *Simulation::current(){
	return Simulation::curr;
}

void Simulation::setCurrent(Simulation *window){
	Simulation::curr = window;
}

void Simulation::DisplayEvent(){ current()->display(); }
void Simulation::KeyboardEvent(unsigned char key,int x,int y ){ current()->keyboard(key,x,y); }
void Simulation::MouseEvent(int button, int state, int x, int y) { current()->mouse(button,state,x,y); }
void Simulation::MotionEvent(int x, int y) { current()->motion(x,y); }
void Simulation::ReshapeEvent(int w, int h) { current()->reshape(w,h); }
void Simulation::TimerEvent(int value) { current()->timer(value); }

void Simulation::motion(int x, int y)
{
	if (leftButton) {
		float dx = x - mousePosX;
		float dy = y - mousePosY;

		mousePosX = x;
		mousePosY = y;

        currView->rotate(dx / width, dy / height);
		loadGlobalCoord();
	}
	return;
}

void Simulation::nextFrame(){
	Eigen::VectorXd delta(6);
	delta = leastSquareDirection(endSites[currEndSite], this->goalOrientation().cast<double>(), this->goalPosition().cast<double>());
	for (int i = 0; i < endSites[currEndSite].size() - 1; i++){
		cout << i << ": " << delta.block(3* i, 0, 3, 1) << endl;
		endSites[currEndSite][i]->rotate(endSites[currEndSite][i]->getRot() + delta.block(3 * i, 0, 3, 1));
	}
}

void Simulation::keyboard(unsigned char key, int x, int y) {
	switch (key) {
	case 'w':
		currView->move(Camera::Forward);
		break;
	case 's':
		currView->move(Camera::Backward);
		break;
	case 'a':
		currView->move(Camera::Left);
		break;
	case 'd':
		currView->move(Camera::Right);
		break;
	case 'q':
        prevSite();
		break;
	case 'e':
		nextSite();
		break;
	case '1':
		nextFrame();
		break;
	case '`':
		activateConsole();
		break;
	case 'r':
		reload();
		break;
	case 27:
		exit(0);
		break;
	default:
		break;
	}
}

void Simulation::reload(){
    std::vector<Segment *> segQ;
	segQ.push_back(this->bvh->getRoot());

    while(segQ.size() > 0){
        auto pr = segQ.back();
		Segment *curr = pr;
        segQ.pop_back();

		curr->rotate(Eigen::Vector3d(0.0, 0.0, 0.0));
		curr->translate(Eigen::Vector3d(0.0, 0.0, 0.0));
		
        for (int i = 0; i < pr->numSub(); i++){
            segQ.push_back(pr->getSeg(i));
        }
    }

	return;
}

void Simulation::prevSite(){
	endSites[currEndSite].back()->setColor(Eigen::Vector3d(1.0, 0.0, 0.0));
	if (currEndSite > 0){
		currEndSite -= 1;
	}
	else currEndSite = endSites.size() - 1;
	endSites[currEndSite].back()->setColor(Eigen::Vector3d(0.0, 1.0, 1.0));
}

void Simulation::nextSite(){
	endSites[currEndSite].back()->setColor(Eigen::Vector3d(1.0, 0.0, 0.0));
	if (currEndSite >= endSites.size() - 1){
		currEndSite = 0;
	}
	else currEndSite += 1;
	endSites[currEndSite].back()->setColor(Eigen::Vector3d(0.0, 1.0, 1.0));
}

void Simulation::mouse(int button, int state, int x, int y)
{
	switch (button)
	{
	case GLUT_LEFT_BUTTON:
		if (state == GLUT_DOWN)
		{
			mousePosX = x;
			mousePosY = y;
			leftButton = true;
		}
		else if (state == GLUT_UP)
		{
			leftButton = false;
		}
		break;
	case GLUT_RIGHT_BUTTON:
		if (state == GLUT_DOWN){
			moveObject = !moveObject;
		}
		break;
	case 3:break;
	default:break;
	}
	return;
}

void Simulation::drawGridPlane() {
	glBegin(GL_LINES);
	glColor3f(0, 1, 0);
	for (int i = -1000; i < 1001; i+=100){
		glVertex3f(i, 0, -1000);
		glVertex3f(i, 0, 1000);
		glVertex3f(1000, 0, i);
		glVertex3f(-1000, 0, i);
	}
	glEnd();
}

void Simulation::drawGoal() {
	glPushMatrix();
	{
		glEigenTranslatef(this->goalPosition());
		glColor3f(0, 1, 0);
		glutSolidSphere(2, 10, 10);
		glColor3f(0, 1, 1);
		for (int i = 0; i < 5; i++){
			glEigenTranslatef(this->goalOrientation() * 2);
			glutSolidSphere(1, 4, 4);
		}
	}
	glPopMatrix();
	/*
	std::vector<Eigen::Vector3d> jointOriX;
	std::vector<Eigen::Vector3d> jointOriY;
	std::vector<Eigen::Vector3d> jointOriZ;
	std::vector<Eigen::Vector3d> jointPos;

	getGlobalPosAndOri(&endSites[currEndSite], &jointOriX, &jointOriY, &jointOriZ, &jointPos);
	std::reverse(jointPos.begin(), jointPos.end());
	for(int i = 0; i < jointPos.size(); i++){
		glPushMatrix();
		{
			glEigenTranslated(jointPos[i]);
			glColor3f(1, 1, 0);
			glutSolidSphere(2 + i, 4, 4);
		}
		glPopMatrix();
	}
	*/
}

Eigen::Vector3f Simulation::goalPosition(){
	return this->goalCamera.getEye() + this->goalCamera.getOri() * 50;
}

Eigen::Vector3f Simulation::goalOrientation(){
	return -this->goalCamera.getOri();
}

void Simulation::display() {
	if (moveObject) currView = &goalCamera;
	else currView = &mainCamera;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	loadGlobalCoord();

	//glRotatef(45, -1, 0, 0);
	glPushMatrix();
	drawGridPlane();
	drawGoal();

	bvh->draw();
	
	glPopMatrix();

	glutSwapBuffers();
}

void Simulation::reshape(int w, int h) {
	width = w;
	height = h;
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0f, (GLfloat)w / (GLfloat)h, .1f, 2500.0f);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void Simulation::timer(int unused)
{
	/* call the display callback and forces the current window to be displayed */
	glutPostRedisplay();
	glutTimerFunc(timeStep, Simulation::TimerEvent, 0);
}