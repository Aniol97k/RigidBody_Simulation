#include <imgui\imgui.h>
#include <imgui\imgui_impl_glfw_gl3.h>
#include <GL\glew.h>
#include <glm\gtc\type_ptr.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include <cstdio>
#include <iostream>
#include <time.h>
#include <math.h>


bool show_test_window = false;
glm::mat4 cubeMatrix;
float timer, reset;

namespace Cube {
	void setupCube();
	void updateCube(const glm::mat4& transform);
	void drawCube();
	void cleanupCube();
};

void GUI() {
	{	
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

		
	}

	if(show_test_window) {
		ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiSetCond_FirstUseEver);
		ImGui::ShowTestWindow(&show_test_window);
	}
}

void scale(float x, float y, float z) { glm::vec3 scaleVector = { x, y, z }; cubeMatrix = glm::scale(cubeMatrix, scaleVector); }

void rotate(float x, float y, float z,float angle) { glm::vec3 rotateVector = { x, y, z }; cubeMatrix = glm::rotate(cubeMatrix, angle, rotateVector); }

void translate(float x, float y, float z) { glm::vec3 translateVector = { x, y, z }; cubeMatrix = glm::translate(cubeMatrix, translateVector); }


void PhysicsInit() {
	//Setup cube
	cubeMatrix =
	{	1,0,0,0,
		0,1,0,0,
		0,0,1,0,
		0,0,0,1 };
	scale(1, 1, 1);
	rotate(1, 1, 1, 30);
	translate(2, 2, 2);

	timer = 0;
	

}
void PhysicsUpdate(float dt) {
	//Update cube
	
	rotate(0, -1, 0, 80.f);
	translate(0, 0.1f, 0);
	
	Cube::updateCube(cubeMatrix);
	
	//Draw cube

}
void PhysicsCleanup() {
	//Cleanup cube
}