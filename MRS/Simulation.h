#pragma once

#include <math.h> 

float pi = 3.14159265358979323846;

#include "Matrix.h"

struct Point
{
	Point()
	{
		x = 0;
		y = 0;
	}

	Point(float px, float py)
	{
		x = px;
		y = py;
	}

	Point(const Point& p1, const Point& p2)
	{
		x = p2.x - p1.x;
		y = p2.y - p1.y;
	}

	float x;
	float y;

	Point operator+(const Point& other) { return Point(x + other.x, y + other.y); }
	Point operator-(const Point& other) { return Point(x - other.x, y - other.y); }
	Point operator*(const float other) { return Point(x * other, y * other); }
	Point operator/(const float other) { return Point(x / other, y / other); }

	float getLenght() const
	{
		return sqrt(x * x + y * y);
	}

	Point getPerpendicular() const
	{
		return Point(y, -x);
	}

	float dot(const Point& other) const
	{
		return x * other.x + y * other.y;
	}

	float getAngle(const Point& other) const
	{
		return acos(this->dot(other) / (this->getLenght() * other.getLenght()));
	}
};

struct Wall
{
	Wall(float px1, float py1, float px2, float py2)
	{
		x1 = px1;
		y1 = py1;
		x2 = px2;
		y2 = py2;
	}

	float x1;
	float y1;
	float x2;
	float y2;
};

// source : https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
bool getLineIntersection(float p0_x, float p0_y, float p1_x, float p1_y,
	float p2_x, float p2_y, float p3_x, float p3_y, float& i_x, float& i_y)
{
	float s1_x, s1_y, s2_x, s2_y;
	s1_x = p1_x - p0_x;     s1_y = p1_y - p0_y;
	s2_x = p3_x - p2_x;     s2_y = p3_y - p2_y;

	float s, t;
	s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y);
	t = (s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y);

	if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
	{
		// Collision detected
		i_x = p0_x + (t * s1_x);
		i_y = p0_y + (t * s1_y);
		return true;
	}

	return false; // No collision
}

bool getLineIntersection(Point p1, Point p2, Point p3, Point p4, Point& i)
{
	return getLineIntersection(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y, p4.x, p4.y, i.x, i.y);
}

float getLength(float x1, float y1, float x2, float y2)
{
	return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

float botWallHit(Point circle, float size, Point circleVel, Point p1, Point p2)
{
	Point lineDir = p2 - p1;
	Point perpendicular = lineDir.getPerpendicular();
	float angle = circleVel.getAngle(perpendicular);

	float l = perpendicular.getLenght();
	perpendicular = perpendicular * size / l;

	if (pi / 2 < angle && angle < 3 * pi / 2)
	{
		perpendicular = perpendicular * -1;
	}

	Point velBig = circleVel * 100 / circleVel.getLenght();
	Point firstHitOnEdge = circle + perpendicular;
	Point moveTill = firstHitOnEdge + velBig;
	Point intersection;

	// hitting middle of the line
	if (getLineIntersection(p1, p2, firstHitOnEdge, moveTill, intersection))
	{
		return (firstHitOnEdge - intersection).getLenght();
	}

	return 1000;
}

float botPointHit(Point circle, float size, Point circleVel, Point p1, Point& hitDir)
{
	Point velUnit = circleVel / circleVel.getLenght();
	Point velPerpendicular = velUnit.getPerpendicular();

	Point intersection;
	if (getLineIntersection(circle + (velPerpendicular * size), circle - (velPerpendicular * size), p1, p1 - (velUnit * 100), intersection))
	{
		float x = Point(circle, intersection).getLenght() / size;
		float y = std::sqrt(1 - x * x) * size;

		Point firstHit = intersection + velUnit * y;
		hitDir = Point(circle, firstHit);

		return Point(firstHit, p1).getLenght();
	}

	return 1000;
}

class NeuralNet
{
public:
	NeuralNet(std::vector<float> parameters, int inputSize, std::vector<int> layerSize)
	{
		int parametersNeeded = 0;
		int lastLayer = inputSize;
		for (int i = 0; i < layerSize.size(); i++)
		{
			parametersNeeded += lastLayer * layerSize[i] + layerSize[i];
			lastLayer = layerSize[i];
		}

		if (parametersNeeded != parameters.size())
		{
			throw std::runtime_error("Wrong amount of paramters");
		}

		auto loc = parameters.begin();
		lastLayer = inputSize;
		for (int i = 0; i < layerSize.size(); i++)
		{
			layers.emplace_back(lastLayer, layerSize[i]);
			layers.back().set(loc, loc + (lastLayer * layerSize[i]));
			loc += (lastLayer * layerSize[i]);

			biases.emplace_back(1, layerSize[i]);
			biases.back().set(loc, loc + layerSize[i]);
			loc += layerSize[i];

			lastLayer = layerSize[i];
		}

		if (loc != parameters.end())
		{
			throw std::runtime_error("Unkown error");
		}

		state.set(0);
	}

	Matrix forward(const Matrix& input)
	{
		Matrix scaled = input * -0.05f;
		scaled.putExp();

		Matrix layer = scaled.getHorizontalStitch(state);

		for (int i = 0; i < layers.size(); i++)
		{
			layer = layer * layers[i] + biases[i];
			layer.putSigmoid();

			if (i == layers.size() - 2)
			{
				state = layer;
			}
		}

		return layer;
	} 

private:
	std::vector<Matrix> layers;
	std::vector<Matrix> biases;
	Matrix state = Matrix(1, 4);
};

struct Bot
{
public:
	float size;

	Point pos;
	float dir;

	Point newPos;
	float newDir;

	bool printTurnPoint = false;
	Point turnPoint;

	//float sensors[12];
	Matrix sensors = Matrix(1, 12);

	int vl = 0;
	int vr = 0;

	void calcSensor(const std::vector<Wall>& walls)
	{
		for (int i = 0; i < 12; i++)
		{
			float xDir = cos(dir + (i * 2 * pi / 12.0));
			float yDir = sin(dir + (i * 2 * pi / 12.0));

			float length = 200;

			for (const auto& wall : walls)
			{
				float px;
				float py;
				if (getLineIntersection(pos.x, pos.y, pos.x + xDir * length, pos.y + yDir * length, wall.x1, wall.y1, wall.x2, wall.y2, px, py))
				{
					float newLength = getLength(pos.x, pos.y, px, py);
					if (newLength < length)
					{
						length = newLength;
					}
				}
			}

			sensors.setElement(0, i, length < 120 ? length : 120);
		}
	}
	void calcnewPosition()
	{
		float time = 0.1;

		if (vl == vr)
		{
			newPos.x = pos.x + vl * cos(dir) * time;
			newPos.y = pos.y + vl * sin(dir) * time;
			newDir = dir;

			printTurnPoint = false;
		}
		else if (vl == -vr || -vl == vr)
		{
			newPos.x = pos.x;
			newPos.y = pos.y;
			newDir = dir + (2 * vl * time) / size;

			printTurnPoint = false;
		}
		else
		{
			float r, w;
			if (vl == 0)
			{
				r = size / 2;
				w = (vr - vl) / size;
			}
			else
			{
				r = (size / 2) * ((vl + vr) / (vr - vl));
				w = (vr - vl) / size;
			}

			float iCCx = pos.x - (r * sin(dir));
			float iCCy = pos.y + (r * cos(dir));

			turnPoint.x = iCCx;
			turnPoint.y = iCCy;

			newPos.x = ((cos(w * time) * (pos.x - iCCx)) + (-sin(w * time) * (pos.y - iCCy))) + iCCx;
			newPos.y = ((sin(w * time) * (pos.x - iCCx)) + (cos(w * time) * (pos.y - iCCy))) + iCCy;
			newDir = dir + w * time;

			printTurnPoint = true;
		}
	}
	void move(const std::vector<Wall>& walls)
	{
		Point vel = newPos - pos;
		float speed = vel.getLenght();
		int maxTries = 2;

		while (maxTries > 0)
		{
			maxTries--;

			bool hitType = 0;
			float maxMovement = 2000;
			Point hitDir;
			int wallIndex;
			for (int i = 0; i < walls.size(); i++)
			{
				float distance = botWallHit(pos, size, vel, Point(walls[i].x1, walls[i].y1), Point(walls[i].x2, walls[i].y2));
				if (distance < maxMovement)
				{
					hitType = 0;
					maxMovement = distance;
					wallIndex = i;
				}

				Point localhitDir;
				distance = botPointHit(pos, size, vel, Point(walls[i].x1, walls[i].y1), localhitDir);
				if (distance < maxMovement)
				{
					hitDir = localhitDir;
					hitType = 1;
					maxMovement = distance;
					wallIndex = i;
				}

				distance = botPointHit(pos, size, vel, Point(walls[i].x2, walls[i].y2), localhitDir);
				if (distance < maxMovement)
				{
					hitDir = localhitDir;
					hitType = 2;
					maxMovement = distance;
					wallIndex = i;
				}
			}

			maxMovement -= 1;
			if (maxMovement <= 0)
			{
				maxMovement = 0;
			}

			if (maxMovement < speed)
			{
				pos = pos + (vel * maxMovement / speed);
				vel = vel - (vel * maxMovement / speed);

				if (hitType == 0)
				{
					// project speed
					Point line(Point(walls[wallIndex].x1, walls[wallIndex].y1), Point(walls[wallIndex].x2, walls[wallIndex].y2));
					float lineLenght = line.getLenght();
					float comp = vel.dot(line) / (lineLenght * lineLenght);
					vel = line * comp;

					speed = vel.getLenght();
					maxMovement = speed;
				}
				else if (hitType == 1 || hitType == 2)
				{
					Point hitPerpendicular = hitDir.getPerpendicular() / hitDir.getLenght();

					// project speed
					float comp = vel.dot(hitPerpendicular);
					vel = hitPerpendicular * comp;

					speed = vel.getLenght();
					maxMovement = speed;
				}
				else
				{
					break;
				}

				if (speed <= 0)
				{
					break;
				}
			}
			else
			{
				pos = pos + vel;
				break;
			}
		}
		dir = newDir;
	}
};

class Simulation
{
public:
	Simulation(float pxSize, float pySize, int pxSteps, int pySteps)
		:floor(pxSteps, pySteps)
	{
		floor.set(0);

		xSize = pxSize;
		ySize = pySize;
		xSteps = pxSteps;
		ySteps = pySteps;
	}

	void step(int vl, int vr, int steps = 1)
	{
		for (int i = 0; i < steps; i++)
		{
			bot.calcSensor(walls);

			bot.vl = vl;
			bot.vr = vr;

			bot.calcnewPosition();

			bot.move(walls);

			updateFloor(bot.pos.x, bot.pos.y);
		}

		stepsDone += steps;
	}

	void autoPilot(std::shared_ptr<NeuralNet> nn, int steps = 1)
	{
		for (int i = 0; i < steps; i++)
		{
			bot.calcSensor(walls);

			Matrix motorSpeed = nn->forward(bot.sensors);
			bot.vl = motorSpeed.getElement(0, 0) * 20 - 10;
			bot.vr = motorSpeed.getElement(0, 1) * 20 - 10;

			bot.calcnewPosition();

			bot.move(walls);

			updateFloor(bot.pos.x, bot.pos.y);
		}

		stepsDone += steps;
	}

	void updateFloor(float x, float y)
	{
		int i = x * xSteps / xSize;
		int j = y * ySteps / ySize;

		floor.setElement(i, j, 1);
	}

	int getAreaSweeped()
	{
		return floor.sumElements();
	}
	
	Matrix floor;
	float xSize;
	float ySize;
	int xSteps;
	int ySteps;


	Bot bot;
	std::vector<Wall> walls;

	int stepsDone = 0;
};