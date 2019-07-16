
#pragma once

#include "cgra/mesh.hpp"
#include "cgra/shader.hpp"
#include "glm/glm.hpp"
#include "pbd.hpp"
#include "entity.hpp"
#include "cloth.hpp"
#include "grid_maker.hpp"

#include <memory>
#include <chrono>

class Scene {
public:
	GLFWwindow *m_window;
	glm::vec2 m_viewportSize;

	glm::mat4 m_projectionMatrix;
	glm::mat4 m_viewMatrix;

	cgra::Program m_program;

	bool freeLook = false;
	glm::vec3 cameraPosition;
	glm::vec3 cameraDirection;

	float cameraPitch = 0;
	float cameraYaw = 0;

	glm::vec2 mousePositionSmoothDelta;

	// 0 = never change, 1 = instantly change (no smoothing)
	float mouseDeltaSmoothingFactor = 0.9;

	float mouseSensitivity = 0.002;
	float motionSensitivity = 0.5;

	glm::vec3 m_translation;
	float m_scale;
	glm::mat4 m_rotationMatrix;

	glm::vec2 m_mousePosition;
	bool m_mouseButtonDown[3];
	bool m_mouseButtonRelease[3];

	Particle *selected = nullptr;
	float initialSelectedDistance = 0;


	std::chrono::time_point<std::chrono::steady_clock> currentTime;
	bool paused = false;
	bool customization;

	int FPS = 60;
	double dt = 1.0f / FPS;
	float timeAccumationModifier = 1.0f;
	float timeScale = 1.0f;

	double timeAccumulation = 0.0f;
	double maxAccumulation = 5 * dt;

	float forceMultiplier = 1.0f;

	PBD pbd;
	std::vector<std::shared_ptr<Entity>> entities;

	std::vector<GridMaker> gridMakers;
	GridMaker *currentGrid;

	cgra::Mesh mesh;

	Scene(GLFWwindow *win)
		: m_window(win),
		m_viewportSize(1, 1), m_mousePosition(0, 0),
		m_translation(0), m_scale(1), m_rotationMatrix(1) {
		m_mouseButtonDown[0] = false;
		m_mouseButtonDown[1] = false;
		m_mouseButtonDown[2] = false;
	}

	void setWindowSize(int width, int height) {
		m_viewportSize.x = float(width);
		m_viewportSize.y = float(height);
	}

	void tick();
	void updatePhysics(float delta);
	void interaction();

	virtual void init();
	//virtual void makeCloth();
	virtual void drawScene();
	virtual void doGUI();
	virtual void onKey(int key, int scancode, int action, int mods);
	virtual void onMouseButton(int button, int action, int mods);
	virtual void onCursorPos(double xpos, double ypos);
	virtual void onScroll(double xoffset, double yoffset);

	glm::vec3 getRay(glm::vec2 viewportSize, glm::vec2 mousePosition, glm::mat4 &projectionMatrix, glm::mat4 &viewMatrix) {
		int width = viewportSize.x, height = viewportSize.y;
		glm::mat4 invProjection = glm::inverse(projectionMatrix);
		float x = (2.0 * mousePosition.x) / width - 1.0f,
			y = 1.0f - (2.0 * mousePosition.y) / height, z = -1.0f;
		glm::vec4 ray = invProjection * glm::vec4(x, y, z, 1);
		ray = glm::vec4(ray.x, ray.y, -1, 0);
		return glm::normalize(glm::vec3(glm::inverse(viewMatrix) * ray));
	}

	glm::vec3 rayIntersect(glm::mat4 invTranslation, glm::vec3 ray);
};
