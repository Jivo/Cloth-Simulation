#include <vector>
#include <iostream>

#include "glm/glm.hpp"
#include "glm/ext.hpp"
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

float areaApprox(std::vector<glm::vec3> &points, int uMax, int vMax, int startU, int startV);
float getArea(glm::vec3 A, glm::vec3 B, glm::vec3 C, glm::vec3 D);
float distLineToPoint(glm::vec3 p1, glm::vec3 p2, glm::vec3 p);
float getIntersecting(glm::vec3, glm::vec3, glm::vec3, glm::vec3);
float cross2D(glm::vec3 v, glm::vec3 w) {return v.x*w.y - v.y*w.x;}
glm::vec3 calcBisec(glm::vec3 e1, glm::vec3 e2, glm::vec3 middle, glm::vec3 projPoint);
glm::vec3 moveLineParallel(glm::vec3, glm::vec3, glm::vec3, float);

std::vector<glm::vec3> getRow(int v, int uMax, std::vector<glm::vec3> grid);


//TODO==========================================REMOPVE
glm::vec3 intersection(-100, -100, -100);
bool intersecting = false;

glm::vec3 GridMaker::evaluateSpline(glm::vec3 P0, glm::vec3 P1, glm::vec3 P2, glm::vec3 P3, float t) {
	return 0.5f * ((2.0f*P1) +
		(-P0 + P2) * t +
		(2.0f * P0 - 5.0f*P1 + 4.0f*P2 - P3) * (t*t) +
		(-P0 + 3.0f*P1 - 3.0f*P2 + P3) * (t*t*t));
}

glm::vec3 GridMaker::evaluateBezier(glm::vec3 P0, glm::vec3 P1, glm::vec3 P2, glm::vec3 P3, float t) {
	return (float)glm::pow(1 - t, 3)*P0 +
		3 * (float)glm::pow(1 - t, 2)*t*P1 +
		3 * (1 - t)*(float)glm::pow(t, 2)*P2 +
		(float)glm::pow(t, 3)*P3;
}

glm::vec3 GridMaker::evaluateLine(glm::vec3 P0, glm::vec3 P1, float t) {
	return (1 - t)*P0 + t*P1;
}

glm::vec3 GridMaker::fullCurvePosition(float t, std::vector<glm::vec3> &curve) {
	if (t >= 1)return curve[curve.size()-1];
	t *= curve.size() - 1;

	int pos = (int)t;
	glm::vec3 P0 = curve[pos], P1 = curve[pos + 1];
	BezierControl *bc = getBezier(pos, &curve);
	if (bc == NULL) { return evaluateLine(P0, P1, t - pos); }
	else { return evaluateBezier(P0, bc->P1, bc->P2, P1, t - pos); }
}

void GridMaker::redefineGrid(unsigned int mode, unsigned int uMax, unsigned int vMax) {
	showGrid = true;
	u_max = uMax; v_max = vMax;
	float uStep = 1.0f / (uMax - 1), vStep = 1.0f / (vMax - 1);
	gridPoints.clear();

	switch (mode) {
	case BILINEAR:
		bilinearInterpolation(uStep, vStep); break;
	case EQUALQUADS:
		equalQuads2(uStep, vStep, uMax, vMax); break;
	}
	makeGridMesh();
}

void GridMaker::bilinearInterpolation(float uStep, float vStep) {
	float v = 0.0f, u = 0.0f;
	while (v <= 1) {
		while (u <= 1) {
			glm::vec3 fu0 = fullCurvePosition(u, edgeCurves[LOWER]);
			glm::vec3 fu1 = fullCurvePosition(u, edgeCurves[UPPER]);
			glm::vec3 f0v = fullCurvePosition(v, edgeCurves[LEFT]);
			glm::vec3 f1v = fullCurvePosition(v, edgeCurves[RIGHT]);

			glm::vec3 f00 = edgeCurves[LEFT][0], f01 = edgeCurves[UPPER][0],
				f10 = edgeCurves[RIGHT][0], f11 = edgeCurves[UPPER][edgeCurves[UPPER].size() - 1];

			glm::vec3 interpLR = (1 - u)*f0v + u*f1v;
			glm::vec3 interpUD = (1 - v)*fu0 + v*fu1;
			glm::vec3 unitBilin = f00*(1 - u)*(1 - v) + f10*u*(1 - v) + f01*(1 - u)*v + f11*u*v;

			gridPoints.push_back(interpLR + interpUD - unitBilin);
			u += uStep;
			if (u > 1 && u - 1<0.9*uStep)u = 1;
		}
		v += vStep;
		u = 0.0f;
		if (v > 1 && v - 1<0.9*vStep)v = 1;
	}
}

void GridMaker::arcLengthInterpolation(float uStep, float vStep, float uMax, float vMax) {
	std::vector<glm::vec3> leftEdge, rightEdge, topEdge, bottomEdge;
	float u = 0;
	while (u <= 1) {
		topEdge.push_back(fullCurvePosition(u, edgeCurves[UPPER]));
		bottomEdge.push_back(fullCurvePosition(u, edgeCurves[LOWER]));
		u += uStep;
		if (u > 1 && u - 1<0.9*uStep)u = 1;
	}
	float v = 0;
	while (v <= 1) {
		leftEdge .push_back(fullCurvePosition(v, edgeCurves[LEFT]));
		rightEdge.push_back(fullCurvePosition(v, edgeCurves[RIGHT]));
		v += vStep;
		if (v > 1 && v - 1<0.9*vStep)v = 1;
	}

	leftEdge = makeEqualLengths(leftEdge);
	rightEdge = makeEqualLengths(rightEdge);
	topEdge = makeEqualLengths(topEdge);
	bottomEdge = makeEqualLengths(bottomEdge);

	for (int i = 0; i < vMax; i++) {
		for (int j = 0; j < uMax; j++) {
			glm::vec3 fu0 = bottomEdge[j];
			glm::vec3 fu1 = topEdge[j];
			glm::vec3 f0v = leftEdge[i];
			glm::vec3 f1v = rightEdge[i];

			float u = j*uStep, v = i*vStep;
			if (u > 1) u = 1; if (v > 1)v = 1;

			glm::vec3 f00 = leftEdge[0], f01 = topEdge[0],
				f10 = rightEdge[0], f11 = topEdge[topEdge.size()-1];

			glm::vec3 interpLR = (1 - u)*f0v + u*f1v;
			glm::vec3 interpUD = (1 - v)*fu0 + v*fu1;
			glm::vec3 unitBilin = f00*(1 - u)*(1 - v) + f10*u*(1 - v) + f01*(1 - u)*v + f11*u*v;

			gridPoints.push_back(interpLR + interpUD - unitBilin);
		}
	}

}
void GridMaker::equalQuads2(float uStep, float vStep, int uMax, int vMax) {
	arcLengthInterpolation(uStep, vStep, uMax, vMax);
	std::vector<glm::vec3> bilinearPoints = gridPoints;
	float S = areaApprox(gridPoints, uMax, vMax, 0, 0);
	float Sq = glm::sqrt(S / ((uMax - 1)*(vMax - 1)));
	gridPoints.clear();

	refreshTable(uStep, LOWER);
	refreshTable(uStep, UPPER);
	refreshTable(vStep, LEFT);
	refreshTable(vStep, RIGHT);

	float u = 0;
	std::vector<glm::vec3> topPoints;
	while (u <= 1) {
		gridPoints.push_back(fullCurvePosition(u, edgeCurves[LOWER]));
		topPoints.push_back(fullCurvePosition(u, edgeCurves[UPPER]));
		u += uStep; if (u > 1 && u - 1<0.9*uStep)u = 1;
	}
	float v = 0;
	std::vector<glm::vec3> leftPoints;
	std::vector<glm::vec3> rightPoints;
	while (v <= 1) {
		leftPoints.push_back(fullCurvePosition(v, edgeCurves[LEFT]));
		rightPoints.push_back(fullCurvePosition(v, edgeCurves[RIGHT]));
		v += vStep; if (v > 1 && v - 1<0.9*vStep)v = 1;
	}
	gridPoints = makeEqualLengths(gridPoints);
	topPoints = makeEqualLengths(topPoints);
	leftPoints = makeEqualLengths(leftPoints);
	rightPoints = makeEqualLengths(rightPoints);

	for (int v = 1; v < vMax - 1; v++) {
		glm::vec3 leftEdge = leftPoints[v];
		glm::vec3 rightEdge = rightPoints[v];

		std::vector<float> distances;
		for (u = 0; u < uMax; u++) {
			distances.push_back((1.0f / (vMax - 1))*
				glm::distance(gridPoints[u],
					topPoints[u]));
		}

		std::vector<glm::vec3> rowPoints;
		float Sqs = 0;
		float Sseg = S / (vMax - 1);
		float threshold = Sseg*0.3;
		float multiplier = 1.0f;

		do {
			rowPoints.clear();
			rowPoints.insert(rowPoints.begin(), gridPoints.end() - uMax, gridPoints.end());
			rowPoints.push_back(leftEdge);

			for (u = 1; u < uMax - 1; u++) {

				//distances[u] *= multiplier;
				distances[u] *= multiplier;
				float Sheight = distances[u];// *multiplier;

				glm::vec3 left = rowPoints[rowPoints.size() - 1];
				glm::vec3 down = rowPoints[rowPoints.size() - uMax];
				glm::vec3 downLeft = rowPoints[rowPoints.size() - uMax - 1];
				glm::vec3 downRight = rowPoints[rowPoints.size() - uMax + 1];

				glm::vec3 e1 = (downLeft - down);
				glm::vec3 e2 = (downRight - down);
				glm::vec3 bisecVec = glm::length(e2)*e1 + glm::length(e1)*e2;

				glm::vec3 P = left, A = down;
				glm::vec3 D = glm::normalize(downLeft - down);
				float interceptLength = glm::dot(P - A, D);
				glm::vec3 X = A + interceptLength*D;
				glm::vec3 vecProj2 = P - X;

				float h = Sheight;
				glm::vec3 point2 = X + h * glm::normalize(vecProj2);


				if (glm::length(bisecVec) < glm::length(e1) &&
					glm::length(bisecVec) < glm::length(e2)) {
					glm::vec3 P = down, A = point2;
					glm::vec3 D = glm::normalize(downLeft - down);
					float interceptLength = glm::dot(P - A, D);
					glm::vec3 X = A + interceptLength*D;
					bisecVec = X - P;
				}

				glm::vec3 Cp = down, Dp = point2, g = Dp - Cp;
				glm::vec3 e = glm::normalize(bisecVec), f = glm::normalize(downLeft - down);
				glm::vec3 FcrossG = glm::cross(f, g), FcrossE = glm::cross(f, e);
				float j = glm::length(FcrossG), k = glm::length(FcrossE);
				glm::vec3 l = (j / k)*e, M;

				if (glm::normalize(FcrossG) != glm::normalize(FcrossE) &&
					glm::length(glm::normalize(FcrossG) - glm::normalize(FcrossE)) > 0.0005) {
					M = Cp - l;
				}
				else {
					M = Cp + l;
				}
				rowPoints.push_back(M);

			}
			rowPoints.push_back(rightEdge);
			Sqs = areaApprox(rowPoints, uMax, 2, 0, 0);
			if (Sqs == -1) {
				gridPoints = bilinearPoints;
				return;
			}
			multiplier = (Sseg) / Sqs;
		} while (glm::abs(Sseg - Sqs) > threshold);

		std::vector<glm::vec3> topOfRow(rowPoints.begin() + uMax, rowPoints.end());
		topOfRow = makeEqualLengths(topOfRow);
		std::copy(topOfRow.begin(), topOfRow.end(), rowPoints.begin() + uMax);
		gridPoints.insert(gridPoints.end(), rowPoints.end() - uMax, rowPoints.end());
	}
	gridPoints.insert(gridPoints.end(), topPoints.begin(), topPoints.end());
}

glm::vec3 moveLineParallel(glm::vec3 parallel, glm::vec3 P, glm::vec3 A, float height) {
	glm::vec3 D = glm::normalize(parallel);
	float interceptLength = glm::dot(P - A, D);
	glm::vec3 X = A + interceptLength*D;
	glm::vec3 vecProj = P - X;
	return X + height*glm::normalize(vecProj);
}

glm::vec3 calcBisec(glm::vec3 e1, glm::vec3 e2, glm::vec3 middle, glm::vec3 projPoint) {
	glm::vec3 bisec = glm::length(e2)*e1 + glm::length(e1)*e2;
	
	if (glm::length(bisec) < glm::length(e1) &&
		glm::length(bisec) < glm::length(e2)) {
		glm::vec3 P = middle, A = projPoint;
		glm::vec3 D = glm::normalize(e1);
		float interceptLength = glm::dot(P - A, D);
		glm::vec3 X = A + interceptLength*D;
		bisec = X - P;
	}
	return bisec;
}

std::vector<glm::vec3> getRow(int v, int uMax, std::vector<glm::vec3> grid) {
	std::vector<glm::vec3> row;
	for (int u = 0; u < uMax; u++) {
		row.push_back(grid[v*uMax + u]);}
	return row;
}

void GridMaker::makeGridMesh() {
	cgra::Matrix<double> m_vertices(gridPoints.size(), 3);
	cgra::Matrix<unsigned int> m_indices(2 * u_max*v_max - v_max, 2);
	std::vector < std::pair<unsigned int, unsigned int>> indices;
	int count = 0;
	m_vertices.setRow(0, { gridPoints[0].x, gridPoints[0].y, gridPoints[0].z });
	for (unsigned int i = 1; i < gridPoints.size(); i++) {
		m_vertices.setRow(i, { gridPoints[i].x, gridPoints[i].y, gridPoints[i].z });
		if (i%u_max != 0)m_indices.setRow(count++, { i, i - 1 });
		if (i >= u_max)m_indices.setRow(count++, { i, i - u_max });
	}
	grid_mesh.setData_lines(m_vertices, m_indices);

	testPoint = intersection;
}


float areaApprox(std::vector<glm::vec3> &points, int uMax, int vMax, int startU, int startV) {
	float area = 0.0f;
	for (int v = startV; v<vMax-1; v++) {
		for (int u = startU; u<uMax-1; u++) {
			glm::vec3 A = points[uMax*v + u];
			glm::vec3 B = points[uMax*v + u + 1];
			glm::vec3 C = points[uMax*(v + 1) + u + 1];
			glm::vec3 D = points[uMax*(v + 1) + u];

			float a = getArea(A, B, C, D);
			if (a == -1) { return -1; }
			area += a;
		}
	}
	return area;
}

float getArea(glm::vec3 A, glm::vec3 B, glm::vec3 C, glm::vec3 D) {
	float AB = glm::distance(A, B);
	float BC = glm::distance(B, C);
	float CD = glm::distance(C, D);
	float DA = glm::distance(D, A);
	float AC = glm::distance(A, C);

	float t = getIntersecting(A, D, B, C);
	
	if (glm::length(glm::normalize(C-B) - glm::normalize(intersection - B)) < 0.001) { 
		if (glm::length(intersection - B) < glm::length(C - B)) { return -1; }
	}

	float S1 = (AB + BC + AC) / 2.0f;
	float area = glm::sqrt(S1*(S1 - AB)*(S1 - BC)*(S1 - AC));
	S1 = (DA + CD + AC) / 2.0f;
	return area + glm::sqrt(S1*(S1 - DA)*(S1 - CD)*(S1 - AC));
}

float distLineToPoint(glm::vec3 p1, glm::vec3 p2, glm::vec3 p) {
	glm::vec3 line = p2 - p1;
	glm::vec3 p_to_p1 = p - p1;

	return glm::length(p_to_p1 - ((glm::dot(p_to_p1, line) /
		glm::pow(glm::length(line), 2))*line));
}

float getIntersecting(glm::vec3 x1, glm::vec3 x2, glm::vec3 y1, glm::vec3 y2) {

	glm::vec3 P = x1, Q = y1;
	glm::vec3 r = x2 - x1, s = y2 - y1;

	float t = cross2D(Q - P, s) / cross2D(r, s);
	intersecting = true;
	intersection = P + t*r;
	return t;

}


void GridMaker::refreshTable(float stepSize, int edge) {
	params[edge] = std::vector<float>();
	lengths[edge] = std::vector<float>();

	float param = 0.0;
	params[edge].push_back(0.0f);
	lengths[edge].push_back(0.0f);

	glm::vec3 prevPoint = fullCurvePosition(param, edgeCurves[edge]);

	param = stepSize;
	while (param <= 1) {
		params[edge].push_back(param);

		glm::vec3 newPoint = fullCurvePosition(param, edgeCurves[edge]);
		float prevSize = lengths[edge][lengths[edge].size() - 1];
		lengths[edge].push_back(prevSize + glm::distance(prevPoint, newPoint));
		prevPoint = newPoint;
		param += stepSize;
		if (param > 1 && param - 1<0.9*stepSize)param = 1;
	}
}

float GridMaker::getParam(float &size, std::vector<float> &params, std::vector<float> &lengths) {
	if (size > lengths[lengths.size() - 1]) { size = 0.0f; }

	//Get lower bound index using a binary search on the lengths vector
	int i0 = binSearch(size, lengths), i1 = i0 + 1;

	float u0 = params[i0], l0 = lengths[i0];
	float u1 = params[i1], l1 = lengths[i1];

	//Interpolate to get the parameter
	return u0 + ((size - l0) / (l1 - l0))*(u1 - u0);
}

int GridMaker::binSearch(float val, std::vector<float> &arr) {
	int first = 0, last = arr.size() - 1;
	int mid = -1;
	while (first <= last) {
		mid = (first + last) / 2;
		if (val > arr[mid]) {
			first = mid + 1;
		}
		else if (val < arr[mid]) {
			last = mid - 1;
		}
		else {
			return mid;
		}
	}
	return mid;
}

std::vector<glm::vec3> GridMaker::makeEqualLengths(std::vector<glm::vec3> &row) {
	std::vector<glm::vec3> result;
	std::vector<float> params, lengths;
	float stepSize = 0.015;

	float param = 0.0;
	params.push_back(0.0f);
	lengths.push_back(0.0f);

	glm::vec3 prevPoint = fullCurvePosition(param, row);

	param = stepSize;
	while (param <= 1) {
		params.push_back(param);

		glm::vec3 newPoint = fullCurvePosition(param, row);
		float prevSize = lengths[lengths.size() - 1];
		lengths.push_back(prevSize + glm::distance(prevPoint, newPoint));
		prevPoint = newPoint;
		param += stepSize;
		if (param > 1 && param - 1<0.9*stepSize)param = 1;
	}

	result.push_back(row[0]);
	float lenStepSize = lengths[lengths.size() - 1] / (row.size() - 1);
	for (int i = 1; i < row.size() - 1; i++) {
		float length = i*lenStepSize;
		float param = getParam(length, params, lengths);
		result.push_back(fullCurvePosition(param, row));
	}
	result.push_back(row[row.size() - 1]);
	return result;
}
