#include <math.h>       
#include <iostream>
#include <SFML/Graphics.hpp>

float * forwardKinematics(int vl, int vr, float xPos, float yPos, float t, float l);
float * checkBorder(float vector[3]);
float pi = 3.14159265358979323846;

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
	float x;
	float y;
	float dir;

	float sensors[12];
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
		if (i_x != NULL)
			i_x = p0_x + (t * s1_x);
		if (i_y != NULL)
			i_y = p0_y + (t * s1_y);
		return true;
	}

	return false; // No collision
}

float getLength(float x1, float y1, float x2, float y2)
{
	return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

int main()
{
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
	walls.emplace_back(200,50,200,200);

	Bot bot;

	//INIT Robot
	int vl, vr;	
	float xPos, yPos, l, t, r, rotation;		
	l = 20;
	vl = vr = 0;

	t = rotation = 0; // THETA = 0;

	sf::CircleShape robot(l);
	robot.setFillColor(sf::Color::Green);

	sf::RectangleShape robotLine(sf::Vector2f(l, 2));
	robotLine.setFillColor(sf::Color::Red);
	robotLine.setPosition(l, l);

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

		menuVl.setSize(sf::Vector2f(50 + vl * 5, 50));
		menuVr.setSize(sf::Vector2f(50 + vr * 5, 50));
		
		xPos = robot.getPosition().x;
		yPos = robot.getPosition().y;

		float* buffer = forwardKinematics(vl, vr, xPos, yPos, t, l);

		xPos = buffer[0];
		yPos = buffer[1];

		if (t != buffer[2])
		{
			rotation = buffer[2] * (180 / pi);

			robotLine.setRotation(rotation);
			menuRobotCircleLine.setRotation(rotation);

			while(rotation > 360)
				rotation = rotation - 360;

			while (rotation < 0)
				rotation = rotation + 360;

			t = buffer[2];
		}

		bot.x = buffer[0] + l;
		bot.y = buffer[1] + l;
		bot.dir = buffer[2];

		icc.setPosition(buffer[3]+l, buffer[4]+l);

		robot.setPosition(buffer[0], buffer[1]);
		robotLine.setPosition(buffer[0]+l, buffer[1]+l);

		textPosXY.setString("Position X:" + std::to_string((int)xPos) + " Y:" + std::to_string((int)yPos));
		menuTextRotation.setString(" Theta:" + std::to_string((int)rotation) + "°");

		menuTextVelocity.setString("Velocity");
		menuVrText.setString("Vl");
		menuVlText.setString("Vr");
		menuVrTextSpeed.setString(std::to_string(vr*10) + "%");
		menuVlTextSpeed.setString(std::to_string(vl*10) + "%");

		window.clear();
		window.draw(robot);
		window.draw(robotLine);
		window.draw(icc);
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
				if (getLineIntersection(bot.x, bot.y, bot.x + xDir * length, bot.y + yDir * length, wall.x1, wall.y1, wall.x2, wall.y2, px, py))
				{
					float newLength = getLength(bot.x, bot.y, px, py);
					if (newLength < length)
					{
						length = newLength;
					}
				}
			}

			bot.sensors[i] = length;

			sf::VertexArray line(sf::Lines, 2);
			line[0].position = sf::Vector2f(bot.x, bot.y);
			line[0].color = sf::Color(0, 0, 255);
			line[1].position = sf::Vector2f(bot.x + xDir * bot.sensors[i], bot.y + yDir * bot.sensors[i]);
			line[1].color = sf::Color(0, 0, 255);
			window.draw(line);
		}

		window.display();
	}
	return 0;
}

float * forwardKinematics(int vl, int vr, float xPos, float yPos, float t, float l)
{
	float vector[6], r, w, time;

	time = 0.01;

	if (vl == vr)
	{
		r = 999999;
		w = 0;

		vector[0] = xPos + vl * cos(t) * time;
		vector[1] = yPos + vl * sin(t) * time;
		vector[2] = t;
		float* buffer = checkBorder(vector);
		vector[0] = buffer[0];
		vector[1] = buffer[1];

		vector[3] = 0;
		vector[4] = 0;

		return vector;
	}
	else if (vl == -vr || -vl == vr)
	{
		r = 0;
		w = t + (2 * vl * time) / l;

		vector[0] = xPos;
		vector[1] = yPos;
		vector[2] = t + (2*vl*time)/l;
		float* buffer = checkBorder(vector);
		vector[0] = buffer[0];
		vector[1] = buffer[1];

		vector[3] = 0;
		vector[4] = 0;

		return vector;
	}
	else if (vr == 0 || vl == 0) 
	{
		r = l / 2;
		w = (vr - vl) / l;
	}
	else
	{
		r = (l / 2) * ((vl + vr) / (vr - vl));
		w = (vr - vl) / l;
	}

	float iCCx = xPos - (r * sin(t));
	float iCCy = yPos + (r * cos(t));

	vector[3] = iCCx;
	vector[4] = iCCy;

	vector[0] = ((cos(w * time) * (xPos - iCCx)) + (-sin(w * time) * (yPos - iCCy))) + iCCx;
	vector[1] = ((sin(w * time) * (xPos - iCCx)) + (cos(w * time) * (yPos - iCCy))) + iCCy;
	vector[2] = t + w * time;

	float* buffer = checkBorder(vector);
	vector[0] = buffer[0];
	vector[1] = buffer[1];

	return vector;
}

float* checkBorder(float vector[3]) 
{
	//Check Screen Borders
	if (vector[0] <= 0)
		vector[0] = 0;

	if (vector[0] >= 1000)
		vector[0] = 1000;

	if (vector[1] <= 0)
		vector[1] = 0;

	if (vector[1] >= 700)
		vector[1] = 700;

	float* result = vector;

	return result;
}
