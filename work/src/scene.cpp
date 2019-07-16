#include <algorithm>
#include <cmath>
#define _USE_MATH_DEFINES
#include <math.h> 
#include <iostream>
#include <stdexcept>
#include <limits>

#include "opengl.hpp"
#include "imgui.h"

#include "cgra/matrix.hpp"
#include "cgra/wavefront.hpp"

#include "scene.hpp"

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtx/euler_angles.hpp"

#include "cloth.hpp"
#include "sphere.hpp"

void Scene::init() {
	m_program = cgra::Program::load_program(
		CGRA_SRCDIR "/res/shaders/simple.vs.glsl",
		CGRA_SRCDIR "/res/shaders/simple.fs.glsl");

	currentTime = std::chrono::steady_clock::now();
	glm::vec3 xDir(15, 0, 0);
	glm::vec3 yDir(0, 10, 0);
	glm::vec3 pos(-2.5f, -2.5f, 0);

	cameraPosition = glm::vec3(0, 0, 10);
	cameraDirection = glm::vec3(0, 0, -1);
	m_viewMatrix = glm::lookAt(cameraPosition, cameraPosition + cameraDirection, glm::vec3(0, 1, 0));
	m_program.setViewMatrix(m_viewMatrix);

	paused = true;
	customization = true;
	entities.push_back(std::make_shared<Cloth>(-(xDir + yDir) / 2.0f, xDir, yDir, 60, 60, 0.1));
	entities.push_back(std::make_shared<Sphere>(glm::vec3(0, 0, -3), 1.3, m_program));
	entities[1]->particles[0].invMass = 0;

	glm::vec3 rotation(1.0f, 1.0f, 0.0f);
	m_rotationMatrix = glm::rotate(glm::mat4(1.0f), glm::radians(0.0f), glm::vec3(rotation[0], rotation[1], rotation[2]));

	gridMakers.push_back(GridMaker(pos, xDir, yDir, 5.0f));
	currentGrid = &gridMakers[0];
}


void Scene::tick() {
	std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
	double delta = timeAccumationModifier * (now - currentTime) / std::chrono::seconds(1);
	currentTime = now;

	if (!paused) {
		timeAccumulation = std::min(maxAccumulation, timeAccumulation + delta);

		while (timeAccumulation >= dt) {
			updatePhysics(dt * timeScale);
			timeAccumulation -= dt;
		}
	}

	drawScene();
}

void Scene::updatePhysics(float delta) {
	interaction();

	pbd.step(entities, delta);

	glm::vec3 v(entities[0]->particles[0].position);
	//std::cout << v.x << ", " << v.y << ", " << v.z << std::endl;
}


void Scene::interaction() {
	if (!customization && !paused) {
		bool a = selected == &entities[0]->particles[0];
		if (m_mouseButtonDown[GLFW_MOUSE_BUTTON_LEFT]) {
			glm::vec3 ray = getRay(m_viewportSize, m_mousePosition, m_projectionMatrix, m_viewMatrix);

			if (selected == NULL) {
				glm::vec3 A = cameraPosition;

				Particle *closest = &entities[0]->particles[5];
				float closestDist = INFINITY;
				for (Particle &c : entities[0]->particles) {
					glm::vec3 P = c.position;
					glm::vec3 X = A + glm::dot(P - A, ray)*ray;
					float dist = glm::distance(P, X);
					if (dist < closestDist) {
						closest = &c;
						closestDist = dist;
					}
				}
				selected = closest;
				initialSelectedDistance = glm::distance(cameraPosition, selected->position);
			}

			//std::cout << selected << " "<<&entities[0]->particles[0]<<"\n";
			// glm::vec3 vel((2.0f * m_mousePosition.x) / m_viewportSize.x - 1.0f, 1.0f - (2.0f * m_mousePosition.y) / m_viewportSize.y, 2);
			glm::vec3 vel((cameraPosition + ray * initialSelectedDistance) - selected->position);
			selected->velocity = vel * forceMultiplier * selected->invMass;
		}
		else {
			selected = NULL;
		}
	}
}


void Scene::drawScene() {
	if (freeLook) {
		glm::vec3 right = glm::normalize(glm::cross(cameraDirection, glm::vec3(0, 1, 0)));

		glm::vec4 tempCamDirection(cameraDirection, 0);
		cameraPitch -= mousePositionSmoothDelta.y * mouseSensitivity;
		cameraYaw -= mousePositionSmoothDelta.x * mouseSensitivity;

		cameraPitch = glm::clamp(cameraPitch, (float)-M_PI / 2.0f + 0.01f, (float)M_PI / 2.0f - 0.01f);

		cameraDirection = glm::vec3(glm::rotate(glm::mat4(1), cameraYaw, glm::vec3(0, 1, 0)) * glm::rotate(glm::mat4(1), cameraPitch, glm::vec3(1, 0, 0)) * glm::vec4(0, 0, -1, 0));

		int netForward = (glfwGetKey(m_window, GLFW_KEY_W) == GLFW_PRESS) - (glfwGetKey(m_window, GLFW_KEY_S) == GLFW_PRESS);
		int netRight = (glfwGetKey(m_window, GLFW_KEY_D) == GLFW_PRESS) - (glfwGetKey(m_window, GLFW_KEY_A) == GLFW_PRESS);
		int netUp = (glfwGetKey(m_window, GLFW_KEY_SPACE) == GLFW_PRESS) - (glfwGetKey(m_window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS);

		glm::vec3 motion = (float)netForward * cameraDirection + (float)netRight * right + (float)netUp * glm::vec3(0, 1, 0);

		if (glm::length(motion) > 0.01) {
			motion = glm::normalize(motion);
		}

		mousePositionSmoothDelta = glm::lerp(mousePositionSmoothDelta, glm::vec2(0, 0), mouseDeltaSmoothingFactor);

		cameraPosition += motionSensitivity * motion;
		m_viewMatrix = glm::lookAt(cameraPosition, cameraPosition + cameraDirection, glm::vec3(0, 1, 0));
		m_program.setViewMatrix(m_viewMatrix);
	}

	// Calculate the aspect ratio of the viewport;
	// width / height
	float aspectRatio = m_viewportSize.x / m_viewportSize.y;
	// Calculate the projection matrix with a field-of-view of 45 degrees
	m_projectionMatrix = glm::perspective(glm::radians(45.0f), aspectRatio, 0.1f, 100.0f);

	// Set the projection matrix
	m_program.setProjectionMatrix(m_projectionMatrix);

	glm::mat4 modelTransform = glm::scale(glm::mat4(1.0f), glm::vec3(m_scale)) * m_rotationMatrix * glm::mat4(1.0f);
	m_program.setModelMatrix(modelTransform);

	if (customization) {
		//Draw the control points and edge curves for customization
		for (GridMaker g : gridMakers) {
			g.draw(m_program, modelTransform);
		}
	}
	else {
		// Draw the mesh
		for (std::shared_ptr<Entity> e : entities) {
			e->draw();
		}
	}
	// mesh.draw();
}

void Scene::doGUI() {
	ImGui::SetNextWindowSize(ImVec2(450, 450), ImGuiSetCond_FirstUseEver);
	ImGui::Begin("Shapes");

	// Example for rotation, use glm to create a a rotation
	// matrix from this vector
	static float stiffness = 1.0f;
	if (ImGui::DragFloat("Stiffness", &stiffness, 0.01, 0, 1)) {
		std::static_pointer_cast<Cloth>(entities[0])->updateStiffness(stiffness);
	}

	if (ImGui::DragFloat("Time Accumulation Modifier", &timeAccumationModifier, 0.01, 0.1, 2)) {
	}

	if (ImGui::DragFloat("Time Scale", &timeScale, 0.01, 0.1, 2)) {
	}

	ImGui::DragFloat("Force multiplier", &forceMultiplier, 0.1, 0, 100);

	static bool useWireFrame;
	if (ImGui::Checkbox("Wire Frame", &useWireFrame)) {
		std::static_pointer_cast<Cloth>(entities[0])->setDrawWireFrame(useWireFrame);
	}


	static glm::vec3 xDir(15, 0, 0);
	static glm::vec3 yDir(0, 10, 0);
	static int particleAmount[2] = { 3, 3 };
	if (ImGui::DragInt2("Cloth x/y particles", particleAmount, 1.0f, 2, 40)) {
		if (!customization) {
			glm::vec3 oldColor = std::static_pointer_cast<Cloth>(entities[0])->color;

			currentGrid->redefineGrid(currentGrid->BILINEAR, particleAmount[0], particleAmount[1]);
			entities[0] = std::make_shared<Cloth>(currentGrid->gridPoints, currentGrid->x_dir,
				currentGrid->y_dir, particleAmount[0], particleAmount[1], stiffness);
			entities[0]->particles[0].velocity = glm::vec3(5, 5, 10);
			std::static_pointer_cast<Cloth>(entities[0])->setDrawWireFrame(useWireFrame);
			std::static_pointer_cast<Cloth>(entities[0])->setColor(oldColor);
		}
	}


	if (!customization) {
		static glm::vec3 color;
		if (ImGui::ColorEdit4("Cloth Color", &color[0])) {
			std::static_pointer_cast<Cloth>(entities[0])->setColor(color);
		}
		ImGui::DragFloat3("Sphere Position", &entities[1]->particles[0].position[0], 0.1);
		ImGui::DragFloat3("Sphere Velocity", &entities[1]->particles[0].velocity[0], 0.1, -3, 3);
	}
	static glm::vec3 position(m_translation);


	ImGui::NewLine();
	ImGui::NewLine();

	if (ImGui::Checkbox("Customize Mode", &customization)) { paused = true; }

	if (ImGui::Checkbox("Pause", &paused) && customization) { paused = true; };



	static bool interpMode = true;
	static bool equalQuadMode = false;
	static int mode = currentGrid->BILINEAR;

	if (ImGui::Button("View Appearence")) {
		currentGrid->redefineGrid(mode,
			particleAmount[0], particleAmount[1]);
	}

	if (ImGui::Checkbox("B-Interpolation", &interpMode)) { equalQuadMode = false; mode = currentGrid->BILINEAR; }
	if (ImGui::Checkbox("Equal Squares", &equalQuadMode)) { interpMode = false; mode = currentGrid->EQUALQUADS; }

	if (ImGui::Button("Generate Cloth")) {
		paused = false; customization = false;
		glm::vec3 oldColor = std::static_pointer_cast<Cloth>(entities[0])->color;
		currentGrid->redefineGrid(mode, particleAmount[0], particleAmount[1]);
		entities[0] = std::make_shared<Cloth>(currentGrid->gridPoints, currentGrid->x_dir,
			currentGrid->y_dir, particleAmount[0], particleAmount[1], stiffness);

		std::static_pointer_cast<Cloth>(entities[0])->setColor(oldColor);
	}

	ImGui::End();
}


// Input Handlers

void Scene::onMouseButton(int button, int action, int) {
	if (button >= 0 && button < 3) {
		// Set the 'down' state for the appropriate mouse button
		m_mouseButtonDown[button] = action == GLFW_PRESS;
		m_mouseButtonRelease[button] = action == GLFW_RELEASE;
	}
}

void Scene::onCursorPos(double xPos, double yPos) {

	// Make a vec2 with the current mouse position
	glm::vec2 currentMousePosition(xPos, yPos);

	// Get the difference from the previous mouse position
	glm::vec2 delta(currentMousePosition - m_mousePosition);
	mousePositionSmoothDelta = glm::lerp(mousePositionSmoothDelta, glm::clamp(delta, -150.0f, 150.0f), mouseDeltaSmoothingFactor);

	if (freeLook) {
		glfwSetCursorPos(m_window, m_viewportSize.x / 2, m_viewportSize.y / 2);

		currentMousePosition = glm::trunc(m_viewportSize / 2.0f);
	}

	if (m_mouseButtonDown[GLFW_MOUSE_BUTTON_LEFT]) {
		if (customization) {
			currentGrid->dragPoint(m_viewportSize, currentMousePosition,
				delta, m_projectionMatrix, m_viewMatrix);
		}
	}
	else if (m_mouseButtonDown[GLFW_MOUSE_BUTTON_MIDDLE]) {

	}
	else if (m_mouseButtonDown[GLFW_MOUSE_BUTTON_RIGHT]) {

	}
	else if (m_mouseButtonRelease[GLFW_MOUSE_BUTTON_LEFT]) {
		if (customization) {
			currentGrid->releasePoint();
		}
	}
	else if (m_mouseButtonRelease[GLFW_MOUSE_BUTTON_RIGHT]) {
		if (customization) {
			currentGrid->alterPoint(m_viewportSize, currentMousePosition,
				delta, m_projectionMatrix, m_viewMatrix);
		}
	}

	// Update the mouse position to the current one
	for (bool &release : m_mouseButtonRelease) {
		release = false;
	}
	m_mousePosition = currentMousePosition;
}

void Scene::onKey(int key, int scancode, int action, int mods) {
	if (key == GLFW_KEY_GRAVE_ACCENT && action != GLFW_RELEASE) {
		freeLook = !freeLook;
		if (freeLook) {
			m_mousePosition = m_viewportSize / 2.0f;
			glfwSetCursorPos(m_window, m_mousePosition.x, m_mousePosition.y);
			mousePositionSmoothDelta = glm::vec2();
		}

		glfwSetInputMode(m_window, GLFW_CURSOR, freeLook ? GLFW_CURSOR_HIDDEN : GLFW_CURSOR_NORMAL);
	}

	if (key == GLFW_KEY_ESCAPE && action != GLFW_RELEASE) {
		exit(0);
	}

	if (!customization) { return; }
	// `(void)foo` suppresses unused variable warnings
	(void)scancode;
	(void)mods;
	float degrees = 5;

	//if (key == GLFW_KEY_A && action!=GLFW_RELEASE) {
	//	currentGrid->rotate(-degrees, glm::vec3(0, 1, 0));
	//}
	//if (key == GLFW_KEY_D && action != GLFW_RELEASE) {
	//	currentGrid->rotate(degrees, glm::vec3(0, 1, 0));
	//}
	//if (key == GLFW_KEY_W && action != GLFW_RELEASE) {
	//	currentGrid->rotate(-degrees, glm::vec3(1, 0, 0));
	//}
	//if (key == GLFW_KEY_S && action != GLFW_RELEASE) {
	//	currentGrid->rotate(degrees, glm::vec3(1, 0, 0));
	//}

	if (key == GLFW_KEY_LEFT_CONTROL &&
		action == GLFW_PRESS) {
		currentGrid->duplicateMode = true;
	}
	else if (key == GLFW_KEY_LEFT_CONTROL &&
		action == GLFW_RELEASE) {
		currentGrid->duplicateMode = false;
	}

	if (key == GLFW_KEY_LEFT_SHIFT &&
		action == GLFW_PRESS) {
		currentGrid->bezierMode = true;
		currentGrid->duplicateMode = true;
	}
	else if (key == GLFW_KEY_LEFT_SHIFT &&
		action == GLFW_RELEASE) {
		currentGrid->bezierMode = false;
		currentGrid->duplicateMode = false;
	}
}

void Scene::onScroll(double xoffset, double yoffset) {
	// `(void)foo` suppresses unused variable warnings
	(void)xoffset;
	(void)yoffset;
}
