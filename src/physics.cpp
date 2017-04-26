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
glm::vec3 linearMomentum, angularMomentum, torque,torqueVector, velocity, cubePosition, randomPoint, randomForce;

glm::vec3 gravity = { 0,-9.81,0 };
glm::vec3 massCenter = { 0,0,0 };


static float elasticity = 0.8f;
static int resetTime = 10;
static float dtCounter = 0;
static float tolerance = 0;
static float mass = 1;

static float lastElasticity, lastTolerance, lastMass;
static int lastResetTime;

namespace Cube {
	void setupCube();
	void updateCube(const glm::mat4& transform);
	void drawCube();
	void cleanupCube();
};

void GUI() {
	{
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
		ImGui::SliderInt("Reset Time", &resetTime, 1, 20);
		ImGui::SliderFloat("Mass", &mass, 1, 20);
		ImGui::SliderFloat("Elasticity", &elasticity, 0.1f, 0.9f);
		ImGui::SliderFloat("Tolerance", &tolerance, 0.1f, 0.9f);
	}

	if(show_test_window) {
		ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiSetCond_FirstUseEver);
		ImGui::ShowTestWindow(&show_test_window);
	}
}

void scaleCube(glm::vec3 scaleVector) { cubeMatrix = glm::scale(cubeMatrix, scaleVector); } //Scale the cube function

void rotateCube(glm::vec3 rotateVector,float angle) { cubeMatrix = glm::rotate(cubeMatrix, angle, rotateVector); } //Rotate the cube function

void translateCube(glm::vec3 translateVector) { cubeMatrix = glm::translate(cubeMatrix, translateVector); } //Translate the cube function

void reset() { //Resets all variables and sets the cube and it's values to random

	lastElasticity = elasticity;
	lastResetTime = resetTime;
	lastTolerance = tolerance;
	lastMass = mass;
	linearMomentum = angularMomentum = torque = velocity = cubePosition = { 0,0,0 };

	randomPoint = { 0.8f,0,0 };
	torqueVector = randomPoint - massCenter;
	randomForce = { 0,7,0 };

	torque = glm::cross(torqueVector, randomForce);
	angularMomentum = angularMomentum  + 0.033f * torque;

	cubeMatrix = {
		1,0,0,0,
		0,1,0,0,
		0,0,1,0,
		0,0,0,1
	};

	translateCube({ 0, 5, 0 });
}

void checkChanges() {if (lastElasticity != elasticity || lastResetTime != resetTime || lastTolerance != tolerance || lastMass != mass) {reset();}} //Check if any value has changed and calls reset function

void eulerStep(float dt) { //Calculation of equations of motion

	linearMomentum = linearMomentum + dt * gravity; //Calculation of the linear momentum (P)
	velocity = linearMomentum / mass; //Calculation of the velocity (v)
	cubePosition = cubePosition + dt * velocity;
	
}

void PhysicsInit() { //Initializes all variables

	lastElasticity = elasticity;
	lastResetTime = resetTime;
	lastTolerance = tolerance;
	lastMass = mass;
	linearMomentum = angularMomentum = torque = velocity = { 0,0,0 };

	randomPoint = { 0.8f,0,0 }; //Random vector for the force position on the cube
	torqueVector = randomPoint - massCenter; //Calculates the torque vector
	randomForce = { 0,7,0 }; //Random vector for the force to apply

	torque = glm::cross(torqueVector, randomForce); //Calculates the torque vector (t)
	angularMomentum = angularMomentum + 0.033f * torque; //Calculates the angular momentum (L)

	cubeMatrix = {
		1,0,0,0,
		0,1,0,0,
		0,0,1,0,
		0,0,0,1
	};

	translateCube({ 0, 5, 0 });
}



void PhysicsUpdate(float dt) {
	
	eulerStep(dt);
	translateCube(cubePosition);
	dtCounter += dt;
	if (dtCounter >= resetTime) { reset(); dtCounter = 0; }
	checkChanges();

	Cube::updateCube(cubeMatrix);
}

void PhysicsCleanup() {
	//Cleanup cube
}