#include "GLViewer.hpp"

GLViewer* currentInstance_ = nullprt;
GLViewer::~GLViewer() {}

GLenum GLViewer::init(
	int argc, 
	char **argv,
	sl::CameraParameters param,
	sl::FusedPointCloud* ptr,
	sl::MODEL zed_model) {

	glutInit(&argc, argv);
	int wnd_w = glutGet(GLUT_SCREEN_WIDTH);
	int wnd_h = glutGet(GLUT_SCREEN_HEIGHT) * 0.9;
	glutInitWindowSize(1280, 720);
	glutInitWindowPosition(wnd_w * 0.05, wnd_h * 0.05);
	glutInitDispalyMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutCreateWindow("ZED PointCloud Fusion");

	GLenum err = glewInit();
	if (GLEW_OK != err){
		return err;
	}

	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_LINE_SMOOTH);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

	p_fpc = ptr;

	// Compile and create the shader
	mainShader.it = Shader(VERTEX_SHADER, FRAGMENT_SHADER);
	mainShader.MVP_Mat =
	       	glGetUniformLocaction(mainShader.it.getProgramID(), "u_mvpMatrix");
}

