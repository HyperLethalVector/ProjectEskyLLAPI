//In this example, we will use a few IBO/VBO and a few VAO.
//We will use glDrawRangeElements to render some shapes.
//We will use a couple of simple shaders. GL 3.3 shaders.
//We will create a 3.3 forward context with freeGLUT.
//
//We are using GLEW to get function pointers.
//
//As for the VBO, we will use an interleaved vertex format.
//Vertex, texcoords and normals.
//Vertex and color.
//
//As for the IBO, we will use 16 bit unsigned integers.
//
//http://freeglut.sourceforge.net
//http://glew.sourceforge.net


#include <GL/glew.h>
#include <GL/freeglut.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

using namespace std;




#define BUFFER_OFFSET(i) ((void*)(i))





//Vertex, tex0
//
//SIZE : 4+4+4 +4+4 = 4*6 = 20 bytes
//It's better to make it multiple of 32
//32-20 = 12 bytes (of garbage should be added)
//12/4 = 3 floats should be added




	//The location of ProjectionModelviewMatrix in the shaders

// loadFile - loads text file into char* fname
// allocates memory - so need to delete after use
// size of file returned in fSize



void InitGLStates()
{
	glShadeModel(GL_SMOOTH);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glReadBuffer(GL_BACK);
	glDrawBuffer(GL_BACK);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glDepthMask(TRUE);
	glDisable(GL_STENCIL_TEST);
	glStencilMask(0xFFFFFFFF);
	glStencilFunc(GL_EQUAL, 0x00000000, 0x00000001);
	glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
	glFrontFace(GL_CCW);
	glCullFace(GL_BACK);
	glEnable(GL_CULL_FACE);
	glClearColor(1.0, 0.0, 0.0, 0.0);
	glClearDepth(1.0);
	glClearStencil(0);
	glDisable(GL_BLEND);
	glDisable(GL_ALPHA_TEST);
	glDisable(GL_DITHER);
	glActiveTexture(GL_TEXTURE0);
}






/*
int main(int argc, char* argv[])
{


	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_STENCIL);
	//We want to make a GL 3.3 context
	glutInitContextVersion(3, 3);
	glutInitContextFlags(GLUT_CORE_PROFILE);
	glutInitWindowPosition(100, 50);
	glutInitWindowSize(600, 600);
	__glutCreateWindowWithExit("GL 3.3 Test", ExitFunction);

	//Currently, GLEW uses glGetString(GL_EXTENSIONS) which is not legal code
	//in GL 3.3, therefore GLEW would fail if we don't set this to TRUE.
	//GLEW will avoid looking for extensions and will just get function pointers for all GL functions.
	glewExperimental = TRUE;
	GLenum err = glewInit();
	if (err != GLEW_OK)
	{
		//Problem: glewInit failed, something is seriously wrong.
		cout << "glewInit failed, aborting." << endl;
		exit(1);
	}

	//The old way of getting the GL version
	//This will give you something like 3.3.2895 WinXP SSE
	//where the 3.3 would be the version number and the rest is vendor
	//dependent information.
	//In this case, 2895 is a build number.
	//Then the OS : WinXP
	//Then CPU features such as SSE
	cout << "OpenGL version = " << glGetString(GL_VERSION) << endl;

	//This is the new way for getting the GL version.
	//It returns integers. Much better than the old glGetString(GL_VERSION).
	glGetIntegerv(GL_MAJOR_VERSION, &OpenGLVersion[0]);
	glGetIntegerv(GL_MINOR_VERSION, &OpenGLVersion[1]);
	cout << "OpenGL major version = " << OpenGLVersion[0] << endl;
	cout << "OpenGL minor version = " << OpenGLVersion[1] << endl << endl;

	//The old method to get the extension list is obsolete.
	//You must use glGetIntegerv and glGetStringi
	glGetIntegerv(GL_NUM_EXTENSIONS, &NumberOfExtensions);

	//We don't need any extensions. Useless code.
	for (i = 0; i < NumberOfExtensions; i++)
	{
		const GLubyte* ccc = glGetStringi(GL_EXTENSIONS, i);
	}

	InitGLStates();

	if (LoadShader("Shader1.vert", "Shader1.frag", false, false, true, ShaderProgram[0], VertexShader[0], FragmentShader[0]) == -1)
	{
		exit(1);
	}
	else
	{
		ProjectionModelviewMatrix_Loc[0] = glGetUniformLocation(ShaderProgram[0], "ProjectionModelviewMatrix");
	}

	if (LoadShader("Shader2.vert", "Shader2.frag", true, true, false, ShaderProgram[1], VertexShader[1], FragmentShader[1]) == -1)
	{
		exit(1);
	}
	else
	{
		ProjectionModelviewMatrix_Loc[1] = glGetUniformLocation(ShaderProgram[1], "ProjectionModelviewMatrix");
	}

	CreateGeometry();

	glutDisplayFunc(display);
	glutReshapeFunc(reshape);

	glutMainLoop();
	return 0;
}
*/