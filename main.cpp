#include "Simulation.hpp"
#define PI 3.14159265

int main(int argc, char** argv) {
	/* Can be enhanced to have UI to choose file */
	string filename = "";
	Simulation *sim = new Simulation();
	bool fileRead = false;

	if (argc >= 2){
		for (int i = 1; i < argc; i++){
			filename += argv[i];
		}
		fileRead = sim->readFile(filename);
	}

	while (fileRead == false){
		cout << "Enter filename: ";
        getline(cin, filename);
    	fileRead = sim->readFile(filename);
	}

	glutInit(&argc, argv);
	sim->initialize();

	glutMainLoop();

	free(sim);

	return 0;
}