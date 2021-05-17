#include "GLViewer.hpp"



void print(std::string msg_prefix, sl::ERROR_CODE err_code, std::string msg_suffix) {
	cout << "[Sample]";
	if (err_code != sl::ERROR_CODE::SUCCESS)
		cout << "[Error] ";
	else 
		cout << " ";
	cout << msg_prefix << " ";

	if (err_code != sl::ERROR_CODE::SUCCESS) {
		cout << " | " << toString(err_code) << " : ";
		cout << toVerbose(err_code);
	}
	if (!msg_suffix.empty())
		cout << " " << msg_suffix;
	cout << endl;
}

using namespace sl;

GLViewer* currentInstance_ = nullptr;

GLViewer::GLViewer() {
	currentInstance_ = this;

}

void GLViewer::init(int argc, char **argv) {
	glutInit(&argc, argv);

	int window_width= glutGet(GLUT_SCREEN_WIDTH);
	int window_height = glutGet(GLUT_SCREEN_HEIGHT) * 0.9;

	glutInitWindowSize(window_width * 0.9, window_height * 0.9);
	glutInitWindowPosition(window_width * 0.05, window_height * 0.05);

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutCreateWindow("Test");

	GLenum err = glewInit();

	if (GLEW_OK != err) {
		print("ERROR: glewInit failed: " + std::string((char*)glewGetErrorString(err))); 
	} 

}

