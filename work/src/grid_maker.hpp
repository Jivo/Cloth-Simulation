#include <vector>
#include <unordered_map>

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtx/euler_angles.hpp"
#include "cgra/mesh.hpp"
#include "cgra/shader.hpp"
#include "pbd.hpp"
#include "entity.hpp"
#include "cloth.hpp"

class GridMaker {

	enum corner_point { TOP_LEFT, TOP_RIGHT, BOTTOM_LEFT, BOTTOM_RIGHT };
	std::vector<glm::vec3> points;

	float point_radius = 0.2f;
	float bezier_radius = 0.1f;

	glm::vec3 testPoint;
	cgra::Mesh test_mesh;
	bool test = false;

	cgra::Mesh point_mesh;
	cgra::Mesh bezier_mesh;
	cgra::Mesh edge_meshes[4];
	cgra::Mesh grid_mesh;

	enum curve_names { LEFT, UPPER, RIGHT, LOWER };
	std::vector<glm::vec3> edgeCurves[4];

	std::vector<glm::vec3 *> draggedPoints;

	struct BezierControl {
		glm::vec3 P1, P2;
		int index;
		std::vector<glm::vec3> *edge;
		BezierControl(glm::vec3 p1, glm::vec3 p2, int idx, std::vector<glm::vec3>*edgeptr) :
			P1(p1), P2(p2) {
			index = idx;
			edge = edgeptr;
		}
		BezierControl() {}
	};
	std::vector<BezierControl> beziers;
	BezierControl *draggedBezier = NULL;

	//tables for length parameterization
	std::vector<float> params[4];
	std::vector<float> lengths[4];

	glm::vec3 rayIntersect(glm::mat4 invTranslation, glm::vec3 ray);
	glm::vec3 getRay(glm::vec2 viewportSize, glm::vec2 mousePosition, glm::mat4 &projectionMatrix, glm::mat4 &viewMatrix);
	void loadObj(const char *filename, cgra::Mesh *mesh);

	glm::vec3 evaluateSpline(glm::vec3 P0, glm::vec3 P1, glm::vec3 P2, glm::vec3 P3, float t);
	glm::vec3 evaluateLine(glm::vec3 P0, glm::vec3 P1, float t);
	glm::vec3 evaluateBezier(glm::vec3 P0, glm::vec3 P1, glm::vec3 P2, glm::vec3 P3, float t);
	glm::vec3 fullCurvePosition(float t, std::vector<glm::vec3> &curve);

	void bilinearInterpolation(float uStep, float vStep);
	void arcLengthInterpolation(float uStep, float vStep, float uMax, float vMax);
	void equalQuads2(float uStep, float vStep, int uMax, int vMax);

	void refreshTable(float stepSize, int edge);
	float getParam(float &size, std::vector<float>&params, std::vector<float>&lengths);
	int binSearch(float val, std::vector<float> &arr);
	std::vector<glm::vec3> makeEqualLengths(std::vector<glm::vec3> &row);

	void makeCurveMesh();
	void makeGridMesh();

	BezierControl *getBezier(int index, std::vector<glm::vec3> *edge) {
		for (BezierControl &bc : beziers) {
			if (*bc.edge == *edge && bc.index == index) { return &bc; }
		}
		return NULL;
	}

	BezierControl *getFowardBezier(int index, std::vector<glm::vec3> *edge) {
		for (BezierControl &bc : beziers) {
			for (int i = index; i < edge->size(); i++) {
				if (*bc.edge == *edge && bc.index == i) { return &bc; }
			}
		}
		return NULL;
	}

public:
	glm::vec3 x_dir, y_dir, origin;
	glm::vec3 normal1, normal2;
	unsigned int u_max = 0, v_max = 0;
	std::vector<glm::vec3> gridPoints;
	bool showGrid = false;

	bool duplicateMode = false;
	bool bezierMode = false;

	enum grid_method { BILINEAR, EQUALQUADS };

	GridMaker(glm::vec3 pos, glm::vec3 xDir, glm::vec3 yDir, float size) {
		initialize(pos, xDir, yDir, size);
	}

	void draw(cgra::Program &program, glm::mat4 &modelTransform);
	void initialize(glm::vec3 pos, glm::vec3 xDir, glm::vec3 yDir, float size);
	void dragPoint(glm::vec2 viewPortSize, glm::vec2 mousePosition, glm::vec2 mousePositionDelta, glm::mat4 &projectionMatrix, glm::mat4 &view);
	void releasePoint() { draggedPoints.clear(); draggedBezier = NULL; }
	void alterPoint(glm::vec2 viewPortSize, glm::vec2 mousePosition, glm::vec2 mousePositionDelta, glm::mat4 &projectionMatrix, glm::mat4 &view);
	void duplicateAndDragg(glm::vec3 location);
	void rotate(float angle, glm::vec3 axis);

	void redefineGrid(unsigned int mode, unsigned int uMax, unsigned int vMax);
	std::vector<glm::vec3> *getGrid() { return &gridPoints; }
};