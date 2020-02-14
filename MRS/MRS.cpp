#include <math.h>       
#include <iostream>
#include <SFML/Graphics.hpp>

float * forwardKinematics(int vl, int vr, float xPos, float yPos, float t, float l);
float * checkBorder(float vector[3]);

int main()
{
	// INIT Fonts
	sf::RenderWindow window(sf::VideoMode(1280, 720), "Mobile Robot Simulator: Group 3");
	sf::Font fontThin, fontMedium, fontBold;
	
	if (!fontThin.loadFromFile("res/fonts/Montserrat-Thin.ttf")) {}
	if (!fontMedium.loadFromFile("res/fonts/Montserrat-Medium.ttf")) {}
	if (!fontBold.loadFromFile("res/fonts/Montserrat-Bold.ttf")){}

	//INIT Interface
	sf::Text textPosXY;
	textPosXY.setFont(fontMedium);
	textPosXY.setCharacterSize(12);
	textPosXY.setFillColor(sf::Color::White);
	textPosXY.setPosition(780, 700);

	sf::RectangleShape menuLine1(sf::Vector2f(720, 2));
	menuLine1.rotate(90);
	menuLine1.setFillColor(sf::Color::Blue);
	menuLine1.setPosition(1000, 0);

	sf::RectangleShape menuLine2(sf::Vector2f(280, 2));
	menuLine2.setFillColor(sf::Color::Blue);
	menuLine2.setPosition(1000, 480);

	sf::RectangleShape menuLine3(sf::Vector2f(280, 2));
	menuLine3.setFillColor(sf::Color::Blue);
	menuLine3.setPosition(1000, 240);

	sf::RectangleShape menuVlBorder(sf::Vector2f(100, 50));
	menuVlBorder.rotate(-90);
	menuVlBorder.setFillColor(sf::Color::Black);
	menuVlBorder.setPosition(1060, 680);
	menuVlBorder.setOutlineThickness(3);
	menuVlBorder.setOutlineColor(sf::Color::Blue);

	sf::RectangleShape menuVl(sf::Vector2f(50, 50));
	menuVl.rotate(-90);
	menuVl.setFillColor(sf::Color::Green);
	menuVl.setPosition(1060, 680);

	sf::RectangleShape menuVrBorder(sf::Vector2f(100, 50));
	menuVrBorder.rotate(-90);
	menuVrBorder.setFillColor(sf::Color::Black);
	menuVrBorder.setPosition(1160, 680);
	menuVrBorder.setOutlineThickness(3);
	menuVrBorder.setOutlineColor(sf::Color::Blue);

	sf::RectangleShape menuVr(sf::Vector2f(50, 50));
	menuVr.rotate(-90);
	menuVr.setFillColor(sf::Color::Green);
	menuVr.setPosition(1160, 680);

	sf::Text menuVlText;
	menuVlText.setFont(fontBold);
	menuVlText.setCharacterSize(18);
	menuVlText.setFillColor(sf::Color::Blue);
	menuVlText.setPosition(1060, 555);

	sf::Text menuVrText;
	menuVrText.setFont(fontBold);
	menuVrText.setCharacterSize(18);
	menuVrText.setFillColor(sf::Color::Blue);
	menuVrText.setPosition(1160, 555);

	sf::Text menuVlTextSpeed;
	menuVlTextSpeed.setFont(fontBold);
	menuVlTextSpeed.setCharacterSize(20);
	menuVlTextSpeed.setFillColor(sf::Color::Blue);
	menuVlTextSpeed.setPosition(1060, 630);

	sf::Text menuVrTextSpeed;
	menuVrTextSpeed.setFont(fontBold);
	menuVrTextSpeed.setCharacterSize(20);
	menuVrTextSpeed.setFillColor(sf::Color::Blue);
	menuVrTextSpeed.setPosition(1160, 630);

	//INIT Robot
	int vl, vr;	
	float xPos, yPos, l, t, r;		
	l = 20;
	vl = vr = 0;

	t = 0; // THETA = 0;

	sf::CircleShape robot(l);
	robot.setFillColor(sf::Color::Green);

	sf::RectangleShape robotLine(sf::Vector2f(l, 2));
	robotLine.setFillColor(sf::Color::Red);
	robotLine.setPosition(l, l);

	sf::RectangleShape icc(sf::Vector2f(2, 2));
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
		t = buffer[2];

		float w = buffer[5];

		icc.setPosition(buffer[3]+l, buffer[4]+l);

		robot.setPosition(buffer[0], buffer[1]);
		robotLine.setPosition(buffer[0]+l, buffer[1]+l);

		//textPosXY.setString("Position X:" + std::to_string(xPos) + " Y:" + std::to_string(yPos) + " T:" + std::to_string(t));
		textPosXY.setString(" Theta:" + std::to_string(t));
		menuVrText.setString("Vl");
		menuVlText.setString("Vr");
		menuVrTextSpeed.setString(std::to_string(vr) + "0%");
		menuVlTextSpeed.setString(std::to_string(vl) + "0%");

		window.clear();
		window.draw(robot);
		window.draw(robotLine);
		window.draw(icc);
		window.draw(textPosXY);
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
		vector[5] = w;

		return vector;
	}
	else if (vl == -vr)
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
		vector[5] = w;

		return vector;
	}
	else if (vl == 0 || vr == 0) 
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
	vector[5] = w;

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
