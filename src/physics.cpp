#include <imgui\imgui.h>
#include <imgui\imgui_impl_glfw_gl3.h>
#include <GL\glew.h>
#include <glm\gtc\type_ptr.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include <glm\gtc\quaternion.hpp>
#include <cstdio>
#include <iostream>
#include <time.h>
#include <math.h>

bool show_test_window = false;
bool firstTime = true;
//Matrix for the cube calculus
glm::mat4 cubeMatrix, transformMatrix, rotationMatrix4;
glm::mat3 inertiaMatrix, rotationMatrix3, inertiaMatrixTemp; 
glm::quat rotationMatrixQuat, angularVelocityQuat;

//Vectors for all the equations
glm::vec3 linearMomentum, angularMomentum, torque,torqueVector, velocity, cubePosition,angularVelocity;
glm::vec3 gravity = { 0,-9.81f,0 };
glm::vec3 randomPoint = { 0.8f,0.2f,0 };
glm::vec3 randomForce = { 8.5f, -12.f,6.5f };
glm::vec3 initialForce = { 0, 120.f,0 };
glm::vec3 randomPosition = { 0,5,0 };
glm::vec3 massCenter = { 0,0,0 };

//Window variables
static float elasticity = 0.8f;
static int resetTime = 3;
static float dtCounter = 0;
static float tolerance = 0;
static float mass = 1;

//To define variables for checking changes
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
		//Creating interactive variables for the simulation
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
		ImGui::Spacing(); ImGui::Spacing(); ImGui::Spacing(); ImGui::Spacing();
		ImGui::Text("Simulation variables");
		ImGui::SliderInt("Reset Time", &resetTime, 1, 20); ImGui::Spacing();
		ImGui::SliderFloat("Mass", &mass, 1, 20); ImGui::Spacing();
		ImGui::SliderFloat("Elasticity", &elasticity, 0.1f, 0.9f); ImGui::Spacing();
		ImGui::SliderFloat("Tolerance", &tolerance, 0.1f, 0.9f); ImGui::Spacing();
		ImGui::Spacing(); ImGui::Spacing(); ImGui::Spacing(); ImGui::Spacing();
		ImGui::Text("Cube vectors");
		ImGui::SliderFloat3("Random point", &randomPoint.x, 0.01f, 0.99f); ImGui::Spacing();
		ImGui::SliderFloat3("Random force", &randomForce.x, -30.f, 30.f); ImGui::Spacing();
		ImGui::SliderFloat3("Initial force", &initialForce.x, -300.f, 300.f); ImGui::Spacing();
		ImGui::Spacing(); ImGui::Spacing(); ImGui::Spacing(); ImGui::Spacing();
		ImGui::Text("Starting position vector");
		ImGui::SliderFloat("X", &randomPosition.x, -4.5f, 4.5f);
		ImGui::SliderFloat("Y", &randomPosition.y, 0.5f, 9.5f);
		ImGui::SliderFloat("Z", &randomPosition.z, -4.5f, 4.5f);
		ImGui::Spacing(); ImGui::Spacing(); ImGui::Spacing(); ImGui::Spacing();
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
	firstTime = true;
	linearMomentum = angularMomentum = torque = velocity =  { 0,0,0 };
	cubePosition = randomPosition;

	torqueVector = randomPoint - massCenter;
	torque = glm::cross(torqueVector, randomForce);
	angularMomentum = angularMomentum  + 0.033f * torque;
	linearMomentum = linearMomentum + 0.033f * randomForce;

	cubeMatrix = glm::mat4(1.0f);
	inertiaMatrix = glm::mat3(1 / 12.0f * mass*2);
	inertiaMatrixTemp = glm::mat3(1);
	rotationMatrixQuat = glm::quat(0,0,0,0);
}

void checkChanges() {if (lastElasticity != elasticity || lastResetTime != resetTime || lastTolerance != tolerance || lastMass != mass) {reset();}} //Check if any value has changed and calls reset function

void eulerStep(float dt) { //Calculation of equations of motion

	if (firstTime == true) { linearMomentum = linearMomentum + dt * initialForce; firstTime = false; } //Calculation of linear momentum with the random inital force (P)
	else { linearMomentum = linearMomentum + dt * gravity; } //Calculation of the linear momentum (P)

	velocity = linearMomentum / mass; //Calculation of the velocity (v)
	cubePosition = cubePosition + dt * velocity; //Calculation of the new cube position (x)

		rotationMatrix3 = glm::mat3_cast(rotationMatrixQuat); //Cast to mat3 of the orientation quat
		inertiaMatrixTemp = rotationMatrix3 * glm::inverse(inertiaMatrix) * glm::transpose(rotationMatrix3); //Calculation of the inertia matrix (I)
		angularVelocity = inertiaMatrixTemp*angularMomentum; //Calculation of the angular velocity with angular momentum (L)
		angularVelocityQuat = glm::quat(0,angularVelocity); //Cast to quat the angular velocity

	rotationMatrixQuat = rotationMatrixQuat + dt * 0.5f * (angularVelocityQuat * rotationMatrixQuat); //Final calculation for the cube rotation (R)
	rotationMatrixQuat = glm::normalize(rotationMatrixQuat); //Normalization of the quat matrix
	rotationMatrix4 = glm::mat4_cast(rotationMatrixQuat); //Cast to mat4 of the quat rotation matrix
}

void PhysicsInit() { //Initializes all variables 

	//Declaration of window and cube variables
	lastElasticity = elasticity;
	lastResetTime = resetTime;
	lastTolerance = tolerance;
	lastMass = mass;
	firstTime = true;
	linearMomentum = angularMomentum = torque = velocity = { 0,0,0 };
	cubePosition = randomPosition;

	torqueVector = randomPoint - massCenter; //Calculates the torque vector
	torque = glm::cross(torqueVector, randomForce); //Calculates the torque vector (t)
	angularMomentum = angularMomentum + 0.033f * torque; //Calculates the angular momentum (L). The 0.033f to act as a frame
	linearMomentum = linearMomentum + 0.033f * randomForce; //Applies the random force to the linear momentum

	//Declaration of cube matrix and quats
	cubeMatrix = glm::mat4(1.0f);
	inertiaMatrix = glm::mat3(1 / 12.0f * mass * 2); 
	rotationMatrixQuat = glm::quat(0, 0, 0, 0);
	inertiaMatrixTemp = glm::mat3(1);
}

void PhysicsUpdate(float dt) {
	
	eulerStep(dt);

	transformMatrix = glm::mat4(1.0f);
	transformMatrix = glm::translate(transformMatrix, cubePosition) * rotationMatrix4;
	
	dtCounter += dt;
	if (dtCounter >= resetTime) { reset(); dtCounter = 0; }
	checkChanges();

	Cube::updateCube(transformMatrix);
}

void PhysicsCleanup() {}