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
glm::mat4 cubeMatrix, transformMatrix, rotationMatrix4, lastTransformMatrix;
glm::mat3 inertiaMatrix, rotationMatrix3, inertiaMatrixTemp;
glm::quat rotationMatrixQuat, angularVelocityQuat;

//Vectors for all the equations
glm::vec3 linearMomentum, angularMomentum, torque, torqueVector, velocity, cubePosition, angularVelocity;
glm::vec3 gravity = { 0,-9.81f,0 };
glm::vec3 randomPoint = { 0.8f,0.2f,0 };
glm::vec3 randomForce = { 8.5f, -12.f,6.5f };
glm::vec3 initialForce = { 0, 120.f,0 };
glm::vec3 randomPosition = { 0,5,0 };
glm::vec3 massCenter = { 0,0,0 };

//Window variables
static float impulseElasticity = 0.8f;
static int resetTime = 3;
static float dtCounter = 0;
static float tolerance = 0;
static float mass = 1;
static bool collisions = true;


//To define variables for checking changes
static float lastImpulseElasticity, lastTolerance, lastMass;
static int lastResetTime;
extern const float halfW = 0.5f;

//Scene planes
glm::vec3 groundN = { 0,1,0 };
glm::vec3 roofN = { 0,-1,0 };
glm::vec3 leftN = { 1,0,0 };
glm::vec3 rightN = { -1,0,0 };
glm::vec3 backN = { 0,0,1 };
glm::vec3 frontN = { 0,0,-1 };

glm::vec3 vertexs[8] = {
	glm::vec3(-halfW, -halfW, -halfW),
	glm::vec3(-halfW, -halfW,  halfW),
	glm::vec3( halfW, -halfW,  halfW),
	glm::vec3( halfW, -halfW, -halfW),
	glm::vec3(-halfW,  halfW, -halfW),
	glm::vec3(-halfW,  halfW,  halfW),
	glm::vec3( halfW,  halfW,  halfW),
	glm::vec3( halfW,  halfW, -halfW)
};

namespace Cube {
	void setupCube();
	void updateCube(const glm::mat4& transform);
	void drawCube();
	void cleanupCube();
};

void reset();

void GUI() {
	{
		//Creating interactive variables for the simulation
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
		ImGui::Spacing(); ImGui::Spacing(); ImGui::Spacing(); ImGui::Spacing();
		ImGui::Text("Simulation variables");
		ImGui::SliderInt("Reset Time", &resetTime, 1, 20); ImGui::Spacing();
		ImGui::SliderFloat("Mass", &mass, 1, 20); ImGui::Spacing();
		ImGui::SliderFloat("Impulse Elasticity", &impulseElasticity, 0.1f, 0.9f); ImGui::Spacing();
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
		if (ImGui::Button("Reset") == true) { reset(); }
		ImGui::Checkbox("Collisions", &collisions);
	}

	if (show_test_window) {
		ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiSetCond_FirstUseEver);
		ImGui::ShowTestWindow(&show_test_window);
	}
}

void scaleCube(glm::vec3 scaleVector) { cubeMatrix = glm::scale(cubeMatrix, scaleVector); } //Scale the cube function

void rotateCube(glm::vec3 rotateVector, float angle) { cubeMatrix = glm::rotate(cubeMatrix, angle, rotateVector); } //Rotate the cube function

void translateCube(glm::vec3 translateVector) { cubeMatrix = glm::translate(cubeMatrix, translateVector); } //Translate the cube function

void reset() { //Resets all variables and sets the cube and it's values to random 

	lastImpulseElasticity = impulseElasticity;
	lastResetTime = resetTime;
	lastTolerance = tolerance;
	lastMass = mass;
	firstTime = true;
	linearMomentum = angularMomentum = torque = velocity = { 0,0,0 };
	cubePosition = randomPosition;

	torqueVector = randomPoint - massCenter;
	torque = glm::cross(torqueVector, randomForce);
	angularMomentum = angularMomentum + 0.033f * torque;
	linearMomentum = linearMomentum + 0.033f * randomForce;

	cubeMatrix = glm::mat4(1.0f);
	inertiaMatrix = glm::mat3(1 / 12.0f * mass * 2);
	inertiaMatrixTemp = glm::mat3(1);
	rotationMatrixQuat = glm::quat(0, 0, 0, 0);
}

void checkChanges() { if (lastImpulseElasticity != impulseElasticity || lastResetTime != resetTime || lastTolerance != tolerance || lastMass != mass) { reset(); } } //Check if any value has changed and calls reset function

void eulerStep(float dt) { //Calculation of equations of motion

	if (firstTime == true) { linearMomentum = linearMomentum + dt * initialForce; firstTime = false; } //Calculation of linear momentum with the random inital force (P)
	else { linearMomentum = linearMomentum + dt * gravity; } //Calculation of the linear momentum (P)

	velocity = linearMomentum / mass; //Calculation of the velocity (v)
	cubePosition = cubePosition + dt * velocity; //Calculation of the new cube position (x)

	rotationMatrix3 = glm::mat3_cast(rotationMatrixQuat); //Cast to mat3 of the orientation quat
	inertiaMatrixTemp = rotationMatrix3 * glm::inverse(inertiaMatrix) * glm::transpose(rotationMatrix3); //Calculation of the inertia matrix (I)
	angularVelocity = inertiaMatrixTemp*angularMomentum; //Calculation of the angular velocity (rotation speed) with angular momentum (w)
	angularVelocityQuat = glm::quat(0, angularVelocity); //Cast to quat the angular velocity

	rotationMatrixQuat = rotationMatrixQuat + dt * 0.5f * (angularVelocityQuat * rotationMatrixQuat); //Final calculation for the cube rotation (R)
	rotationMatrixQuat = glm::normalize(rotationMatrixQuat); //Normalization of the quat matrix
	rotationMatrix4 = glm::mat4_cast(rotationMatrixQuat); //Cast to mat4 of the quat rotation matrix
}

float calculateCollision(glm::vec3 vertexVector,glm::mat4 lastTransformMatrix, glm::mat4 transformMatrix, glm::vec3 normal, int d) {

	glm::vec3 lastVector = lastTransformMatrix * glm::vec4(vertexVector, 1); //Generates vector for the last position from the mat4 position matrix
	glm::vec3 newVector = transformMatrix * glm::vec4(vertexVector, 1);  //Generates vector for the current position from the mat4 position matrix
	float calc1 = glm::dot(newVector, normal) + d;
	float calc2 = glm::dot(lastVector, normal) + d;
	return calc1*calc2;
}

void calculateImpulse(glm::vec3 normal, glm::vec3 vertex, glm::mat4 transformMatrix) {

	//Relative speed calculation
	glm::vec3 vertexPosition = transformMatrix  * glm::vec4(vertex, 1); //Generates position vector for the vertex (P_a)
	glm::vec3 collidingPoint = velocity + glm::cross(angularVelocity, vertexPosition - cubePosition); //Calculates the colliding point
	float Vrelative =  glm::dot(normal, collidingPoint); //Final calculation of relative speed (V_rel)

	//Impulse magnitude calculation (j)
	glm::vec3 IMPart1 = inertiaMatrixTemp*glm::cross(vertexPosition, normal);
	glm::vec3 IMPart2 = glm::cross(IMPart1, vertexPosition);
	float IMPart3 = glm::dot(normal, IMPart2) + (1.0f / mass);
	float impulseMagnitude = -(1 + impulseElasticity)*Vrelative / IMPart3;

	//Calculation of the impulse (J)
	glm::vec3 impulse = normal * impulseMagnitude;

	//Calculation of the torque impulse (t_impulse)
	glm::vec3 torqueImpulse = glm::cross(vertexPosition, impulse);

	//Update linear momentum (P) and angular momentum (L)
	linearMomentum = linearMomentum + impulse;
	angularMomentum = angularMomentum + torqueImpulse;
}


void calculateAllCollisions(glm::mat4 lastTransformMatrix, glm::mat4 transformMatrix ) {

	for (int i = 0; i < 8; i++) { //Check collision with each vertex

		if (calculateCollision(vertexs[i], lastTransformMatrix, transformMatrix, groundN, 0) < 0) calculateImpulse(groundN, vertexs[i], transformMatrix); 
		
		if (calculateCollision(vertexs[i], lastTransformMatrix, transformMatrix, roofN, 10) < 0) calculateImpulse(roofN, vertexs[i], transformMatrix);
		
		if (calculateCollision(vertexs[i], lastTransformMatrix, transformMatrix, leftN, 5) < 0) calculateImpulse(leftN, vertexs[i], transformMatrix);
		
		if (calculateCollision(vertexs[i], lastTransformMatrix, transformMatrix, rightN, 5) < 0) calculateImpulse(rightN, vertexs[i], transformMatrix);
		
		if (calculateCollision(vertexs[i], lastTransformMatrix, transformMatrix, frontN, 5) < 0) calculateImpulse(frontN, vertexs[i], transformMatrix);
		
		if (calculateCollision(vertexs[i], lastTransformMatrix, transformMatrix, backN, 5) < 0) calculateImpulse(backN, vertexs[i], transformMatrix);
	}
}


void PhysicsInit() { //Initializes all variables 

	//Declaration of window and cube variables
	lastImpulseElasticity = impulseElasticity;
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

	lastTransformMatrix = transformMatrix; //Save last position as a mat4

	eulerStep(dt); //Calculate the euler step

	transformMatrix = glm::mat4(1.0f); //Generate the transform matrix
	transformMatrix = glm::translate(transformMatrix, cubePosition) * rotationMatrix4; //Calculate translation and rotation for the current cube matrix
	
	if(collisions == true) calculateAllCollisions(lastTransformMatrix, transformMatrix); //Check all collisions for the cube and each one of its vertex

	//Counter update and check variable changes
	dtCounter += dt;
	if (dtCounter >= resetTime) { reset(); dtCounter = 0; }
	checkChanges();

	Cube::updateCube(transformMatrix); //Update the cube to draw
}

void PhysicsCleanup() {}