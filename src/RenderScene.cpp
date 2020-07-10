#include "common_header.h"

#include "win_OpenGLApp.h"

#include "shaders.h"

/*-----------------------------------------------

Name:		initScene

Params:	lpParam - Pointer to anything you want.

Result:	Initializes OpenGL features that will
			be used.

/*---------------------------------------------*/

float fQuad[12]; // Data to render quad using triangle strips (4 vertices, each has 3 floats)
float fQuadTextureCoords[12];
float fQuadColor[12];

UINT uiVBO[4];
UINT uiVAO[2];

CShader shVertex, shFragment;
CShaderProgram spMain;
GLint baseImageLocLeft;
GLint baseImageLocRight;
int TextureLoc;
void InitScene(LPVOID lpParam)
{
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

 
	// Setup quad vertices
 
	fQuad[0] = -1.0f; fQuad[1] = 1.0f; fQuad[2] = 0.0f;
	fQuad[3] = -1.0f; fQuad[4] = -1.0f; fQuad[5] = 0.0f;
	fQuad[6] = 1.0f; fQuad[7] = 1.0f; fQuad[8] = 0.0f;
	fQuad[9] = 1.0f; fQuad[10] = -1.0f; fQuad[11] = 0.0f;

	fQuadTextureCoords[0] = 0.0f; fQuadTextureCoords[1] = 1.0f; fQuadTextureCoords[2] = 0.0f;
	fQuadTextureCoords[3] = 0.0f; fQuadTextureCoords[4] = 0.0f; fQuadTextureCoords[5] = 0.0f;
	fQuadTextureCoords[6] = 1.0f; fQuadTextureCoords[7] = 1.0f; fQuadTextureCoords[8] = 0.0f;
	fQuadTextureCoords[9] = 1.0f; fQuadTextureCoords[10] = 0.0f; fQuadTextureCoords[11] = 0.0f;

	// Setup quad color
	fQuadColor[0] = 1.0f; fQuadColor[1] = 0.0f; fQuadColor[2] = 0.0f;
	fQuadColor[3] = 0.0f; fQuadColor[4] = 1.0f; fQuadColor[8] = 0.0f;
	fQuadColor[6] = 0.0f; fQuadColor[7] = 0.0f; fQuadColor[5] = 1.0f;
	fQuadColor[9] = 1.0f; fQuadColor[10] = 1.0f; fQuadColor[11] = 0.0f;
	glGenVertexArrays(2, uiVAO); // Generate two VAOs, one for triangle and one for quad
	glGenBuffers(3, uiVBO); // And four VBOs
	// Setup whole quad
	glBindVertexArray(uiVAO[1]);
	glBindBuffer(GL_ARRAY_BUFFER, uiVBO[1]);
	glBufferData(GL_ARRAY_BUFFER, 12*sizeof(float), fQuad, GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glBindBuffer(GL_ARRAY_BUFFER, uiVBO[2]);
	glBufferData(GL_ARRAY_BUFFER, 12*sizeof(float), fQuadColor, GL_STATIC_DRAW);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glBindBuffer(GL_ARRAY_BUFFER, uiVBO[0]);
	glBufferData(GL_ARRAY_BUFFER, 12 * sizeof(float), fQuadTextureCoords, GL_STATIC_DRAW);
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, 0);
	// Load shaders and create shader program
	shVertex.LoadShader("shader.vert", GL_VERTEX_SHADER);
	shFragment.LoadShader("shader.frag", GL_FRAGMENT_SHADER);
	spMain.CreateProgram();
	spMain.AddShaderToProgram(&shVertex);
	spMain.AddShaderToProgram(&shFragment);
	baseImageLocLeft = glGetUniformLocation(spMain.uiProgram, "gSamplerLeft");
	baseImageLocRight = glGetUniformLocation(spMain.uiProgram, "gSamplerRight");
	spMain.LinkProgram();
	spMain.UseProgram();
}

/*-----------------------------------------------

Name:	RenderScene

Params:	lpParam - Pointer to anything you want.

Result:	Renders whole scene.

/*---------------------------------------------*/

void RenderScene(LPVOID lpParam)
{
	// Typecast lpParam to COpenGLControl pointer
	COpenGLControl* oglControl = (COpenGLControl*)lpParam;
	// We just clear color
	glClear(GL_COLOR_BUFFER_BIT);

	//glBindVertexArray(uiVAO[0]);
//	glDrawArrays(GL_TRIANGLES, 0, 3);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, oglControl->TextureIDLeft);
	glBindSampler(0, baseImageLocLeft);
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, oglControl->TextureIDRight);
	glBindSampler(1, baseImageLocRight);

//	glBindTexture(GL_TEXTURE_2D, oglControl->TextureID);
	glBindVertexArray(uiVAO[1]);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

	oglControl->SwapBuffersM();
}

/*-----------------------------------------------

Name:	ReleaseScene

Params:	lpParam - Pointer to anything you want.

Result:	Releases OpenGL scene.

/*---------------------------------------------*/

void ReleaseScene(LPVOID lpParam)
{
	spMain.DeleteProgram();

	shVertex.DeleteShader();
	shFragment.DeleteShader();
}