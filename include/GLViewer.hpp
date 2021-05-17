#pragma once 

#include <GL/glew.h>
#include <GL/freeglut.h>

#include <sl/Camera.hpp>

using namespace std;
void print(std::string msg_prefix, sl::ERROR_CODE err_code = sl::ERROR_CODE::SUCCESS, std::string msg_suffix = "");


class GLViewer {
	public:
	GLViewer();

	void init(int argc, char **argv);
};


