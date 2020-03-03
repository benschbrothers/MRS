#include <math.h>       
#include <iostream>
#include <chrono>
#include <thread>
#include <SFML/Graphics.hpp>
#include "GeneticSearch.h"

float * forwardKinematics(int vl, int vr, float xPos, float yPos, float t, float l);
float * checkBorder(float vector[3]);
float pi = 3.14159265358979323846;

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

	Point operator+(const Point& other){return Point(x + other.x, y + other.y);}
	Point operator-(const Point& other){return Point(x - other.x, y - other.y);}
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

	float sensors[12];

	void calcnewPosition(int vl, int vr)
	{
		float time = 0.01;

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
		float y = std::sqrt(1 - x*x) * size;

		Point firstHit = intersection + velUnit * y;
		hitDir = Point(circle, firstHit);

		return Point(firstHit, p1).getLenght();
	}

	return 1000;
}

float rosenbrock(float xi, float yi)
{
	float x = xi;
	float y = yi;
	float b = 100;
	float a = 1;

	float left = a - x;
	float right = y - x * x;
	float result = left * left + 100 * right * right;

	return result;
}

float rastrigin(float xi, float yi) 
{
	float x = xi;
	float y = yi;
	float n = 2;

	float left = 10 * n;

	float xsum = x * x - 10 * cos(2 * pi * x);
	float ysum = y * y - 10 * cos(2 * pi * y);

	return (left + xsum + ysum);
}

void Plot(double min, double max, std::function<float(float,float)> func)
{
	sf::RenderWindow window(sf::VideoMode(1000, 1000), "Mobile Robot Simulator : Group 3 - Plotter");
	sf::Font fontMedium;
	sf::Texture textureHeatmap;
	if (!fontMedium.loadFromFile("res/fonts/Montserrat-Medium.ttf")) {}


	// create an empty 800x800 texture
	if (!textureHeatmap.create(800, 800)) {}

	sf::Sprite spriteHeatmap;
	spriteHeatmap.setPosition(sf::Vector2f(20.f, 20.f));

	// 800x800 Heatmap
	sf::Uint8* pixels = new sf::Uint8[800 * 800 * 4];

	//Get min / max values for rosenbrock / rastrigin to create color range for the heatmap
	float curmax = -999999999;
	float curmin = 999999999;
	float stepSize = (max - min) / 800.0;
	for (int x = 0; x < 800; x++) {
		for (int y = 0; y < 800; y++)
		{
			float value = func(min + stepSize * x , min + stepSize * y);

			if (value < curmin)
				curmin = value;
			else if (value > curmax)
				curmax = value;
		}
	}
	std::cout << "Min: " << curmin << " MAX: " << curmax << "\n";

	//create heatmap
	int i = 0;
	for (int x = 0; x < 800; x++) {
		for (int y = 0; y < 800; y++)
		{
			float value = func(min + stepSize * x, min + stepSize * y);

			/*int HeatmapRange = 800;
			int valuesRange = max - min;
			int rangevalue = (((OldValue - OldMin) * NewRange) / OldRange) + NewMin;*/


			// convert value range to RGB
			float r, g, b = 0;
			float ratio = 2 * (value - curmin) / (curmax - curmin);
			if (int(255 * (1 - ratio) > 0))
				b = int(255 * (1 - ratio));
			else
				b = 0;
			if (int(255 * (ratio - 1)) > 0)
				r = int(255 * (ratio - 1));
			else
				r = 0;
			g = 255 - b - r;
			
			pixels[i] = (int)r;
			pixels[i + 1] = (int)g;
			pixels[i + 2] = (int)b;
			pixels[i + 3] = 255;
			i += 4;

		}
	}

	textureHeatmap.update(pixels);
	spriteHeatmap.setTexture(textureHeatmap);
	window.draw(spriteHeatmap);

	sf::RectangleShape rectangle{ { 10.f, 10.f } };

	auto drawDot = [&](Individual i, sf::Color color = sf::Color::White)
	{
		
		rectangle.setFillColor(color);

		float x = (i[0] - min) / stepSize;
		float y = (i[1] - min) / stepSize;

		rectangle.setPosition({ x, y });

		window.draw(rectangle);
	};

	// NOTE:
	// typedef std::vector<double> Individual;
	// typedef std::vector<Individual> Population;
	
	GeneticSearch ga(2, 50, 100); // Number of variables, number of generations, population size
	ga.mutationRate = 0.2;
	ga.lowerBound = min; // Lower and upper bound of variables in initial population generation and mutations
	ga.upperBound = max;
	ga.elitism = 0.02;
	ga.top = 0.25; // Only top 25% are used as parents, future version could be upgraded to other selection method

	ga.setFitnessFunction([&](const Individual& i)
		{
			// The GA maximizes, add a negative here if we want to minimize
			return -func(i[0], i[1]);
		});

	// This function is not necessary, should do nothing, just usefull for intermediary updates, it gets called every time a new population is made.
	ga.setUpdateCallback([&](const Population& pop)
		{
			sf::Event event;
			while (window.pollEvent(event))
			{
				// "close requested" event: we close the window
				if (event.type == sf::Event::Closed)
					window.close();
			}

			window.draw(spriteHeatmap);

			double lowest = 999;
			Individual best;
			for (const auto& i : pop)
			{
				drawDot(i);
				if (func(i[0], i[1]) < lowest)
				{
					lowest = func(i[0], i[1]);
					best = i;
				}
			}
			drawDot(best, sf::Color::Red);
			window.display();
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			std::cout << func(best[0], best[1]) << " at " << best[0] << "," << best[1] << "\n";
		});

	ga.run();

	Individual best = ga.getBest();
	std::cout << " FINAL:" << func(best[0], best[1]) << " at " << best[0] << "," << best[1] << "\n";

	// run the program as long as the window is open
	while (window.isOpen())
	{
		// check all the window's events that were triggered since the last iteration of the loop
		sf::Event event;
		while (window.pollEvent(event))
		{
			// "close requested" event: we close the window
			if (event.type == sf::Event::Closed)
				window.close();
		}

		window.draw(spriteHeatmap);

		drawDot(best, sf::Color::Red);

		window.display();
	}
}

int main()
{
	//Plot(-2, 2, rosenbrock);
	Plot(-2, 2, rastrigin);

	return 0;

	// INIT Fonts
	sf::RenderWindow window(sf::VideoMode(1280, 720), "Mobile Robot Simulator: Group 3");
	sf::Font fontThin, fontMedium, fontBold;
	
	if (!fontThin.loadFromFile("res/fonts/Montserrat-Thin.ttf")) {}
	if (!fontMedium.loadFromFile("res/fonts/Montserrat-Medium.ttf")) {}
	if (!fontBold.loadFromFile("res/fonts/Montserrat-Bold.ttf")){}

	//INIT ----------- Interface ---------------
	sf::Text textPosXY;
	textPosXY.setFont(fontMedium);
	textPosXY.setCharacterSize(12);
	textPosXY.setFillColor(sf::Color::White);
	textPosXY.setPosition(780, 700);

	// ----- MENU Lines -----
	sf::RectangleShape menuLine1(sf::Vector2f(720, 2));
	menuLine1.rotate(90);
	menuLine1.setFillColor(sf::Color::Blue);
	menuLine1.setPosition(1000, 0);

	sf::RectangleShape menuLine2(sf::Vector2f(280, 2));
	menuLine2.setFillColor(sf::Color::Blue);
	menuLine2.setPosition(1000, 500);

	sf::RectangleShape menuLine3(sf::Vector2f(280, 2));
	menuLine3.setFillColor(sf::Color::Blue);
	menuLine3.setPosition(1000, 250);

	// ----- MENU Circle -----
	sf::Text menuTextRotation;
	menuTextRotation.setFont(fontMedium);
	menuTextRotation.setCharacterSize(20);
	menuTextRotation.setFillColor(sf::Color::Blue);
	menuTextRotation.setPosition(1070, 475);

	sf::CircleShape menuRobotCircle(70);
	menuRobotCircle.setFillColor(sf::Color::Black);
	menuRobotCircle.setPosition(1070, 300);
	menuRobotCircle.setOutlineThickness(3);
	menuRobotCircle.setOutlineColor(sf::Color::Blue);

	sf::RectangleShape menuRobotCircleLine(sf::Vector2f(70, 4));
	menuRobotCircleLine.setFillColor(sf::Color::Green);
	menuRobotCircleLine.setPosition(1140, 370);

	// ----- MENU Velocity -----

	sf::Text menuTextVelocity;
	menuTextVelocity.setFont(fontBold);
	menuTextVelocity.setCharacterSize(25);
	menuTextVelocity.setFillColor(sf::Color::Red);
	menuTextVelocity.setPosition(1070, 520);

	sf::RectangleShape menuVlBorder(sf::Vector2f(100, 50));
	menuVlBorder.rotate(-90);
	menuVlBorder.setFillColor(sf::Color::Black);
	menuVlBorder.setPosition(1160, 680);
	menuVlBorder.setOutlineThickness(3);
	menuVlBorder.setOutlineColor(sf::Color::Blue);

	sf::RectangleShape menuVl(sf::Vector2f(50, 50));
	menuVl.rotate(-90);
	menuVl.setFillColor(sf::Color::Green);
	menuVl.setPosition(1160, 680);

	sf::RectangleShape menuVrBorder(sf::Vector2f(100, 50));
	menuVrBorder.rotate(-90);
	menuVrBorder.setFillColor(sf::Color::Black);
	menuVrBorder.setPosition(1060, 680);
	menuVrBorder.setOutlineThickness(3);
	menuVrBorder.setOutlineColor(sf::Color::Blue);

	sf::RectangleShape menuVr(sf::Vector2f(50, 50));
	menuVr.rotate(-90);
	menuVr.setFillColor(sf::Color::Green);
	menuVr.setPosition(1060, 680);

	sf::Text menuVlText;
	menuVlText.setFont(fontBold);
	menuVlText.setCharacterSize(18);
	menuVlText.setFillColor(sf::Color::Blue);
	menuVlText.setPosition(1160, 555);

	sf::Text menuVlTextSpeed;
	menuVlTextSpeed.setFont(fontBold);
	menuVlTextSpeed.setCharacterSize(20);
	menuVlTextSpeed.setFillColor(sf::Color::Blue);
	menuVlTextSpeed.setPosition(1160, 630);

	sf::Text menuVrText;
	menuVrText.setFont(fontBold);
	menuVrText.setCharacterSize(18);
	menuVrText.setFillColor(sf::Color::Blue);
	menuVrText.setPosition(1060, 555);

	sf::Text menuVrTextSpeed;
	menuVrTextSpeed.setFont(fontBold);
	menuVrTextSpeed.setCharacterSize(20);
	menuVrTextSpeed.setFillColor(sf::Color::Blue);
	menuVrTextSpeed.setPosition(1060, 630);

	std::vector<Wall> walls;
	walls.emplace_back(50,50,800,50);
	walls.emplace_back(50, 50, 50, 400);
	walls.emplace_back(50, 400, 800, 400);
	walls.emplace_back(800, 50, 800, 400);
	walls.emplace_back(600, 300, 800, 400);

	walls.emplace_back(100, 100, 200, 300);
	walls.emplace_back(200, 300, 300, 350);

	Bot bot;
	bot.pos.x = 500;
	bot.pos.y = 300;
	bot.dir = 0;
	bot.size = 20;

	//INIT Robot
	int vl, vr;		
	vl = vr = 0;

	sf::CircleShape robot(bot.size);
	robot.setFillColor(sf::Color::Green);

	sf::RectangleShape robotLine(sf::Vector2f(bot.size, 2));
	robotLine.setFillColor(sf::Color::Red);
	robotLine.setPosition(bot.size, bot.size);

	sf::RectangleShape icc(sf::Vector2f(3, 3));
	icc.setFillColor(sf::Color::Red);
	icc.setPosition(0, 0);

	while (window.isOpen())
	{
		sf::Event event;
		while (window.pollEvent(event))
		{
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::W))
			{
				if (vl < 10)
					vl++;
			}

			if (sf::Keyboard::isKeyPressed(sf::Keyboard::S))
			{
				if (vl > -10)
					vl--;
			}

			if (sf::Keyboard::isKeyPressed(sf::Keyboard::O))
			{
				if (vr < 10)
					vr++;
			}

			if (sf::Keyboard::isKeyPressed(sf::Keyboard::L))
			{
				if (vr > -10)
					vr--;
			}

			if (sf::Keyboard::isKeyPressed(sf::Keyboard::T))
			{
				if (vl < 10)
					vl++;

				if (vr < 10)
					vr++;
			}

			if (sf::Keyboard::isKeyPressed(sf::Keyboard::G))
			{
				if (vl > -10)
					vl--;

				if (vr > -10)
					vr--;
			}
			
			if (event.type == sf::Event::Closed)
				window.close();
		}

		window.clear();

		menuVl.setSize(sf::Vector2f(50 + vl * 5, 50));
		menuVr.setSize(sf::Vector2f(50 + vr * 5, 50));

		bot.calcnewPosition(vl, vr);

		Point vel = bot.newPos - bot.pos;
		float speed = vel.getLenght();
		int maxTries = 2;

		while(maxTries > 0)
		{
			maxTries--; 

			bool hitType = 0;
			float maxMovement = 2000;
			Point hitDir;
			int wallIndex;
			for (int i = 0; i < walls.size(); i++)
			{
				float distance = botWallHit(bot.pos, bot.size, vel, Point(walls[i].x1, walls[i].y1), Point(walls[i].x2, walls[i].y2));
				if (distance < maxMovement)
				{
					hitType = 0;
					maxMovement = distance;
					wallIndex = i;
				}

				Point localhitDir;
				distance = botPointHit(bot.pos, bot.size, vel, Point(walls[i].x1, walls[i].y1), localhitDir);
				if (distance < maxMovement)
				{
					hitDir = localhitDir;
					hitType = 1;
					maxMovement = distance;
					wallIndex = i;
				}

				distance = botPointHit(bot.pos, bot.size, vel, Point(walls[i].x2, walls[i].y2), localhitDir);
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
				bot.pos = bot.pos + (vel * maxMovement / speed);
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
				bot.pos = bot.pos + vel;
				break;
			}
		}
		bot.dir = bot.newDir;

		robot.setPosition(bot.pos.x - bot.size, bot.pos.y - bot.size);
		robotLine.setRotation(bot.dir * (180 / pi));
		robotLine.setPosition(bot.pos.x, bot.pos.y);
		menuRobotCircleLine.setRotation(bot.dir * (180 / pi));
		icc.setPosition(bot.turnPoint.x, bot.turnPoint.y);
	
		//textPosXY.setString("Position X:" + std::to_string((int)xPos) + " Y:" + std::to_string((int)yPos));
		//menuTextRotation.setString(" Theta:" + std::to_string((int)rotation) + "°");

		menuTextVelocity.setString("Velocity");
		menuVrText.setString("Vl");
		menuVlText.setString("Vr");
		menuVrTextSpeed.setString(std::to_string(vr*10) + "%");
		menuVlTextSpeed.setString(std::to_string(vl*10) + "%");

		window.draw(robot);
		window.draw(robotLine);
		if(bot.printTurnPoint) window.draw(icc);
		window.draw(textPosXY);
		window.draw(menuRobotCircle);
		window.draw(menuRobotCircleLine);
		window.draw(menuTextRotation);
		window.draw(menuTextVelocity);
		window.draw(menuLine1);
		window.draw(menuLine2);
		window.draw(menuLine3);
		window.draw(menuVlBorder);
		window.draw(menuVrBorder);
		window.draw(menuVr);
		window.draw(menuVl);
		window.draw(menuVrText);
		window.draw(menuVlText);
		window.draw(menuVrTextSpeed);
		window.draw(menuVlTextSpeed);

		for (const auto& wall : walls)
		{
			sf::VertexArray line(sf::Lines, 2);
			line[0].position = sf::Vector2f(wall.x1, wall.y1);
			line[0].color = sf::Color(100, 0, 200);
			line[1].position = sf::Vector2f(wall.x2, wall.y2);
			line[1].color = sf::Color(100, 0, 200);
			window.draw(line);
		}

		for (int i = 0; i < 12; i++)
		{
			float xDir = cos(bot.dir + (i * 2 * pi / 12.0));
			float yDir = sin(bot.dir + (i * 2 * pi / 12.0));

			float length = 200;

			for (const auto& wall : walls)
			{
				float px;
				float py;
				if (getLineIntersection(bot.pos.x, bot.pos.y, bot.pos.x + xDir * length, bot.pos.y + yDir * length, wall.x1, wall.y1, wall.x2, wall.y2, px, py))
				{
					float newLength = getLength(bot.pos.x, bot.pos.y, px, py);
					if (newLength < length)
					{
						length = newLength;
					}
				}
			}
			
			bot.sensors[i] = length;

			/*sf::Text sensorData;
			sensorData.setFont(fontBold);
			sensorData.setCharacterSize(10);
			sensorData.setFillColor(sf::Color::Blue);
			sensorData.setPosition((bot.pos.x - 10) + xDir * 30, (bot.pos.y -10) + yDir * 30);
			sensorData.setString(std::to_string((int)bot.sensors[i]));
			window.draw(sensorData);


			sf::VertexArray line(sf::Lines, 2);
			line[0].position = sf::Vector2f(bot.pos.x, bot.pos.y);
			line[0].color = sf::Color(0, 0, 255);
			line[1].position = sf::Vector2f(bot.pos.x + xDir * bot.sensors[i], bot.pos.y + yDir * bot.sensors[i]);
			line[1].color = sf::Color(0, 0, 255);
			window.draw(line);*/
		}

		window.display();
	}
	return 0;
}
