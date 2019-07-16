#include <vector>
#include <iostream>

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtx/euler_angles.hpp"
#include "cgra/mesh.hpp"
#include "cgra/shader.hpp"
#include "cgra/wavefront.hpp"

//#include "pbd.hpp"
//#include "entity.hpp"
//#include "cloth.hpp"
//#include "particle.hpp"

#include "grid_maker.hpp"

void GridMaker::initialize(glm::vec3 position, glm::vec3 xDir, glm::vec3 yDir, float size) {

	point_mesh.setDefaultColor(glm::vec3(0, 1, 1));
	bezier_mesh.setDefaultColor(glm::vec3(1, 1, 0));
	test_mesh.setDefaultColor(glm::vec3(1, 0, 0));

	loadObj(CGRA_SRCDIR"/res/sphere.obj", &test_mesh);
	loadObj(CGRA_SRCDIR"/res/sphere.obj", &point_mesh);
	loadObj(CGRA_SRCDIR"/res/sphere.obj", &bezier_mesh);

	x_dir = xDir; y_dir = yDir; origin = position;
	normal1 = glm::cross(glm::normalize(xDir), glm::normalize(yDir));
	normal2 = -normal1;

	points.push_back(glm::vec3());
	points.push_back(glm::vec3());
	points.push_back(glm::vec3());
	points.push_back(glm::vec3());

	points[BOTTOM_LEFT] = position;
	points[BOTTOM_RIGHT] = position + glm::normalize(xDir)*size;
	points[TOP_LEFT] = position + glm::normalize(yDir)*size;
	points[TOP_RIGHT] = position + glm::normalize(yDir)*size + glm::normalize(xDir)*size;

	edgeCurves[LEFT].push_back(points[BOTTOM_LEFT]);
	edgeCurves[LEFT].push_back(points[TOP_LEFT]);

	edgeCurves[RIGHT].push_back(points[BOTTOM_RIGHT]);
	edgeCurves[RIGHT].push_back(points[TOP_RIGHT]);

	edgeCurves[UPPER].push_back(points[TOP_LEFT]);
	edgeCurves[UPPER].push_back(points[TOP_RIGHT]);

	edgeCurves[LOWER].push_back(points[BOTTOM_LEFT]);
	edgeCurves[LOWER].push_back(points[BOTTOM_RIGHT]);
}

void GridMaker::draw(cgra::Program &program, glm::mat4 &modelTransform) {

	grid_mesh.line_mode = true;
	if (showGrid) grid_mesh.draw();

	makeCurveMesh();
	for (cgra::Mesh &mesh : edge_meshes) {
		mesh.setDefaultColor(glm::vec3(1, 0.3, 1));
		mesh.draw();
	}

	glm::vec3 scalar(point_radius);
	glm::mat4 scale = glm::scale(glm::mat4(1.0f), scalar);
	if (test) {
		glm::mat4 pos = glm::translate(glm::mat4(1.0f), testPoint);
		program.setModelMatrix(modelTransform*pos*scale);
		test_mesh.draw();
	}

	for (std::vector<glm::vec3> &edge : edgeCurves) {
		for (glm::vec3 &point : edge) {

			if (point == edgeCurves[UPPER][0] ||
				point == edgeCurves[UPPER][edgeCurves[UPPER].size() - 1]) {
				point_mesh.setColor(glm::vec3(1, 0.5, 0.5));
			}

			else if (point == edgeCurves[LOWER][0] ||
				point == edgeCurves[LOWER][edgeCurves[LOWER].size() - 1]) {
				point_mesh.setColor(glm::vec3(0.5, 1, 0.5));
			}

			else { point_mesh.setColor(glm::vec3(1, 1, 0)); }
			glm::mat4 pos = glm::translate(glm::mat4(1.0f), point);
			program.setModelMatrix(modelTransform*pos*scale);
			point_mesh.draw();
		}
	}

	scalar = glm::vec3(bezier_radius);
	for (BezierControl &bc : beziers) {
		glm::vec3 &p1 = bc.P1;
		glm::vec3 &p2 = bc.P2;
		glm::mat4 scale = glm::scale(glm::mat4(1.0f), scalar);

		glm::mat4 pos = glm::translate(glm::mat4(1.0f), p1);
		program.setModelMatrix(modelTransform*pos*scale);
		bezier_mesh.draw();
		pos = glm::translate(glm::mat4(1.0f), p2);
		program.setModelMatrix(modelTransform*pos*scale);
		bezier_mesh.draw();
	}

}

void GridMaker::dragPoint(glm::vec2 viewportSize, glm::vec2 mousePosition, glm::vec2 mousePositionDelta,
	glm::mat4 &projectionMatrix, glm::mat4 &viewMatrix) {

	int width = viewportSize.x, height = viewportSize.y;
	//glm::mat4 translation = glm::translate(glm::mat4(1.0f), viewTranslation);
	glm::mat4 invView = glm::inverse(viewMatrix);
	glm::mat4 invProjection = glm::inverse(projectionMatrix);

	glm::vec3 ray = getRay(viewportSize, mousePosition, projectionMatrix, viewMatrix);
	glm::vec3 intersect = rayIntersect(invView, ray);
	if (intersect.x == INFINITY) { return; }

	if (draggedPoints.size() == 0) {
		if (duplicateMode) { duplicateAndDragg(intersect); }
		else {
			//Look for intersecting Bezier control points
			for (BezierControl &bc : beziers) {
				if (glm::distance(bc.P1, intersect) <= point_radius) {
					draggedPoints.push_back(&bc.P1); return;
				}
				else if (glm::distance(bc.P2, intersect) <= point_radius) {
					draggedPoints.push_back(&bc.P2); return;
				}
			}

			//Look for intersecting edge control points
			for (std::vector<glm::vec3> &edge : edgeCurves) {
				for (glm::vec3 &point : edge) {
					if (glm::distance(point, intersect) <= point_radius) {
						draggedPoints.push_back(&point);
					}
				}
			}
		}
	}

	for (glm::vec3 *dragged : draggedPoints) {
		showGrid = false;
		(*dragged) = intersect;
		if (draggedBezier != NULL) {
			glm::vec3 offset = (*dragged) - (*draggedBezier->edge)[draggedBezier->index];
			draggedBezier->P2 = (*draggedBezier->edge)[draggedBezier->index + 1] - offset;
		}
	}
}

void GridMaker::alterPoint(glm::vec2 viewportSize, glm::vec2 mousePosition, glm::vec2 mousePositionDelta,
	glm::mat4 &projectionMatrix, glm::mat4 &viewMatrix) {

	int width = viewportSize.x, height = viewportSize.y;
	//glm::mat4 translation = glm::translate(glm::mat4(1.0f), viewTranslation);
	glm::mat4 invView = glm::inverse(viewMatrix);
	glm::mat4 invProjection = glm::inverse(projectionMatrix);

	glm::vec3 ray = getRay(viewportSize, mousePosition, projectionMatrix, viewMatrix);
	glm::vec3 intersect = rayIntersect(invView, ray);
	if (intersect.x == INFINITY) { return; }

	for (std::vector<glm::vec3> &edge : edgeCurves) {
		for (int i = 1; i < edge.size() - 1; i++) {
			if (glm::distance(edge[i], intersect) <= point_radius) {
				edge.erase(edge.begin() + i);
			}
		}
	}
	for (int i = 0; i < beziers.size(); i++) {
		BezierControl &bc = beziers[i];
		if (glm::distance(bc.P1, intersect) <= point_radius ||
			glm::distance(bc.P2, intersect) <= point_radius) {
			beziers.erase(beziers.begin() + i);
		}
	}
}

void GridMaker::duplicateAndDragg(glm::vec3 location) {
	for (std::vector<glm::vec3> &edge : edgeCurves) {

		for (int i = 0; i < edge.size() - 1; i++) {
			int index, direction;
			if (&edge == &edgeCurves[LOWER] || &edge == &edgeCurves[RIGHT]) { index = edge.size() - i - 1; direction = 0; }
			else { index = i; direction = 1; }

			if (glm::distance(edge[index], location) <= point_radius) {

				if (bezierMode) {
					int stepToNext = &edge == &edgeCurves[LOWER] || &edge == &edgeCurves[RIGHT] ?
						-1 : 0; index += stepToNext;
					if (getBezier(index, &edge) != NULL) { return; }
					beziers.push_back(BezierControl(edge[index], edge[index + 1], index, &edge));
					draggedBezier = &beziers[beziers.size() - 1];
					draggedPoints.push_back(&beziers[beziers.size() - 1].P1);
					return;
				}
				else {
					BezierControl *bc = getFowardBezier(index, &edge);
					if (bc != NULL) { bc->index++; }
					edge.insert(edge.begin() + index + direction, edge[index]);
					draggedPoints.push_back(&edge[index + direction]);
					return;
				}
			}
		}
	}
}

glm::vec3 GridMaker::getRay(glm::vec2 viewportSize, glm::vec2 mousePosition, glm::mat4 &projectionMatrix, glm::mat4 &viewMatrix) {
	int width = viewportSize.x, height = viewportSize.y;
	glm::mat4 invProjection = glm::inverse(projectionMatrix);
	float x = (2.0 * mousePosition.x) / width - 1.0f,
		y = 1.0f - (2.0 * mousePosition.y) / height, z = -1.0f;
	glm::vec4 ray = invProjection * glm::vec4(x, y, z, 1);
	ray = glm::vec4(ray.x, ray.y, -1, 0);
	return glm::normalize(glm::vec3(glm::inverse(viewMatrix) * ray));
}

glm::vec3 GridMaker::rayIntersect(glm::mat4 invTranslation, glm::vec3 ray) {
	glm::vec3 rayOrigin = glm::vec3(invTranslation*glm::vec4(0, 0, 0, 1));
	float t = (glm::dot((origin - rayOrigin), normal1)) / glm::dot(ray, normal1);
	if (!t > 0) { t = (glm::dot((origin - rayOrigin), normal2)) / glm::dot(ray, normal2); }
	if (t < 0) { return glm::vec3(INFINITY, 0, 0); }
	return rayOrigin + ray*t;
}

void GridMaker::makeCurveMesh() {

	float bezStep = 0.01;

	for (int edgeIdx = 0; edgeIdx < 4; edgeIdx++) {
		std::vector<glm::vec3> &edge = edgeCurves[edgeIdx];
		std::vector<glm::vec3> vertices;
		std::vector<glm::vec2> indices;
		vertices.push_back(edge[0]);
		for (int i = 1; i < edge.size(); i++) {

			BezierControl *bc = getBezier(i - 1, &edgeCurves[edgeIdx]);
			//if (edgeIdx == 3) { std::cout << &edge << "\n"; }
			if (bc == NULL) {
				vertices.push_back(edge[i]);
				int lastIdx = vertices.size() - 1;
				indices.push_back(glm::vec2(lastIdx - 1, lastIdx));
			}
			else {
				float t = 0.0f;
				while (t < 1) {
					vertices.push_back(evaluateBezier(edge[i - 1], bc->P1, bc->P2, edge[i], t));
					int lastIdx = vertices.size() - 1;
					indices.push_back(glm::vec2(lastIdx - 1, lastIdx));
					t += bezStep;
					if (t > 1) t = 1.0f;
				}
			}
		}
		cgra::Matrix<double> m_vertices(vertices.size(), 3);
		cgra::Matrix<unsigned int> m_indices(indices.size(), 2);
		m_vertices.setRow(0, { vertices[0].x, vertices[0].y, vertices[0].z });
		for (int i = 1; i < vertices.size(); i++) {
			m_vertices.setRow(i, { vertices[i].x, vertices[i].y, vertices[i].z });
			m_indices.setRow(i - 1, { (unsigned int)indices[i - 1].x, (unsigned int)indices[i - 1].y });
		}
		edge_meshes[edgeIdx].setData_lines(m_vertices, m_indices);
	}
}


void GridMaker::rotate(float angle, glm::vec3 axis) {
	showGrid = false;
	glm::mat4 rotation = glm::rotate(glm::mat4(1.0f), glm::radians(angle), axis);
	origin = glm::vec3(rotation*glm::vec4(origin, 0.0f));
	x_dir = glm::vec3(rotation*glm::vec4(x_dir, 0.0f));
	y_dir = glm::vec3(rotation*glm::vec4(y_dir, 0.0f));

	//Rotate edge points
	for (std::vector<glm::vec3> &edge : edgeCurves) {
		for (glm::vec3 &point : edge) {
			point = glm::vec3(rotation*glm::vec4(point, 0.0f));
		}
	}

	//Rotate Bezier points
	for (BezierControl &bc : beziers) {
		bc.P1 = glm::vec3(rotation*glm::vec4(bc.P1, 0.0f));
		bc.P2 = glm::vec3(rotation*glm::vec4(bc.P2, 0.0f));
	}

	for (glm::vec3 &point : gridPoints) {
		point = glm::vec3(rotation*glm::vec4(point, 0.0f));
		//makeGridMesh();
	}

	normal1 = glm::cross(glm::normalize(x_dir), glm::normalize(y_dir));
	normal2 = -normal1;
}

void GridMaker::loadObj(const char *filename, cgra::Mesh *mesh) {
	cgra::Wavefront obj;
	try {
		obj = cgra::Wavefront::load(filename);
	}
	catch (std::exception e) {
		std::cerr << "Couldn't load file: '" << e.what() << "'" << std::endl; return;
	}

	int numVertices = obj.m_positions.size();
	int numTriangles = obj.m_faces.size();

	cgra::Matrix<double> mesh_vertices(numVertices, 3);
	cgra::Matrix<unsigned int> mesh_triangles(numTriangles, 3);

	for (size_t i = 0; i < obj.m_positions.size(); i++) {
		mesh_vertices.setRow(i, { obj.m_positions[i][0], obj.m_positions[i][1], obj.m_positions[i][2] });
	}

	for (size_t i = 0; i < obj.m_faces.size(); i++) {
		mesh_triangles.setRow(i, { obj.m_faces[i].m_vertices[0].m_p - 1,
			obj.m_faces[i].m_vertices[1].m_p - 1,
			obj.m_faces[i].m_vertices[2].m_p - 1 });
	}

	mesh->setData(mesh_vertices, mesh_triangles);
}