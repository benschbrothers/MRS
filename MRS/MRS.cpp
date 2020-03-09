#include <math.h>       
#include <iostream>
#include <chrono>
#include <thread>
#include <SFML/Graphics.hpp>

//Uncomment me to run single threaded (below cpp 17)
#define PARALLEL

#include "GeneticSearch.h"
#include "Simulation.h"

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
	//Plot(-2, 2, rastrigin);

	//GeneticSearch ga(120 + 10 + 40 + 4 + 8 + 2, 500, 50); // Number of variables, number of generations, population size
	GeneticSearch ga(64 + 4 + 8 + 2, 500, 50); // Number of variables, number of generations, population size
	//GeneticSearch ga(24 + 2, 500, 50); // Number of variables, number of generations, population size
	ga.mutationRate = 0.1;
	ga.lowerBound = -2.5; // Lower and upper bound of variables in initial population generation and mutations
	ga.upperBound = 2.5;
	ga.elitism = 0.05;
	ga.top = 0.10; // Only top 25% are used as parents, future version could be upgraded to other selection method

	ga.setFitnessFunction([](const Individual& i)
	{
		Simulation sim(800, 400, 100, 100);

		sim.bot.pos.x = 500;
		sim.bot.pos.y = 300;
		sim.bot.dir = 0;
		sim.bot.size = 20;

		sim.walls.emplace_back(50, 50, 800, 50);
		sim.walls.emplace_back(50, 50, 50, 400);
		sim.walls.emplace_back(50, 400, 800, 400);
		sim.walls.emplace_back(800, 50, 800, 400);
		sim.walls.emplace_back(600, 300, 800, 400);
		sim.walls.emplace_back(100, 100, 200, 300);
		sim.walls.emplace_back(200, 300, 300, 350);

		//std::vector<int> layers = { 10, 4, 2 };
		std::vector<int> layers = { 4, 2 };
		//std::vector<int> layers = { 2 };
		auto nn = std::make_shared<NeuralNet>(i, 16, layers);

		sim.autoPilot(nn, 10000);

		return sim.getAreaSweeped();
	});

	// This function is not necessary, should do nothing, just usefull for intermediary updates, it gets called every time a new population is made.
	/*ga.setUpdateCallback([&](const Population& pop)
	{

	});*/

	ga.run();

	std::cout << "Starting visualization of best individual, wait on user input.\n";
	std::cin.get();

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

	sf::Text textStatus;
	textStatus.setFont(fontBold);
	textStatus.setCharacterSize(18);
	textStatus.setFillColor(sf::Color::Blue);
	textStatus.setPosition(20, 20);

	Simulation sim(800, 400, 100, 100);

	sim.bot.pos.x = 500;
	sim.bot.pos.y = 300;
	sim.bot.dir = 0;
	sim.bot.size = 20;

	sim.walls.emplace_back(50, 50, 800, 50);
	sim.walls.emplace_back(50, 50, 50, 400);
	sim.walls.emplace_back(50, 400, 800, 400);
	sim.walls.emplace_back(800, 50, 800, 400);
	sim.walls.emplace_back(600, 300, 800, 400);
	sim.walls.emplace_back(100, 100, 200, 300);
	sim.walls.emplace_back(200, 300, 300, 350);

	int vl, vr;		
	vl = vr = 0;

	sf::CircleShape robot(sim.bot.size);
	robot.setFillColor(sf::Color::Green);

	sf::RectangleShape robotLine(sf::Vector2f(sim.bot.size, 2));
	robotLine.setFillColor(sf::Color::Red);
	robotLine.setPosition(sim.bot.size, sim.bot.size);

	sf::RectangleShape icc(sf::Vector2f(3, 3));
	icc.setFillColor(sf::Color::Red);
	icc.setPosition(0, 0);

	std::vector<int> layers = { 4, 2 };
	//std::vector<int> layers = { 2 };
	auto nn = std::make_shared<NeuralNet>(ga.getBest(), 16, layers);

	while (window.isOpen())
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(5));

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

		//sim.step(vl, vr, 1);

		sim.autoPilot(nn, 1);

		robot.setPosition(sim.bot.pos.x - sim.bot.size, sim.bot.pos.y - sim.bot.size);
		robotLine.setRotation(sim.bot.dir * (180 / pi));
		robotLine.setPosition(sim.bot.pos.x, sim.bot.pos.y);
		menuRobotCircleLine.setRotation(sim.bot.dir * (180 / pi));
		icc.setPosition(sim.bot.turnPoint.x, sim.bot.turnPoint.y);
	
		//textPosXY.setString("Position X:" + std::to_string((int)xPos) + " Y:" + std::to_string((int)yPos));
		//menuTextRotation.setString(" Theta:" + std::to_string((int)rotation) + "°");

		menuTextVelocity.setString("Velocity");
		menuVrText.setString("Vl");
		menuVlText.setString("Vr");
		menuVrTextSpeed.setString(std::to_string(vr*10) + "%");
		menuVlTextSpeed.setString(std::to_string(vl*10) + "%");
		textStatus.setString("Steps: " + std::to_string(sim.stepsDone) + ", sweeped: " + std::to_string(sim.getAreaSweeped()) + ", vl=" + std::to_string(sim.bot.vl) + ", vr=" + std::to_string(sim.bot.vr));

		window.draw(robot);
		window.draw(robotLine);
		if(sim.bot.printTurnPoint) window.draw(icc);
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
		window.draw(textStatus);

		for (const auto& wall : sim.walls)
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
			float xDir = cos(sim.bot.dir + (i * 2 * pi / 12.0));
			float yDir = sin(sim.bot.dir + (i * 2 * pi / 12.0));

			float length = 200;

			for (const auto& wall : sim.walls)
			{
				float px;
				float py;
				if (getLineIntersection(sim.bot.pos.x, sim.bot.pos.y, sim.bot.pos.x + xDir * length, sim.bot.pos.y + yDir * length, wall.x1, wall.y1, wall.x2, wall.y2, px, py))
				{
					float newLength = getLength(sim.bot.pos.x, sim.bot.pos.y, px, py);
					if (newLength < length)
					{
						length = newLength;
					}
				}
			}
			
			//bot.sensors.setElement(i, 0, length);

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
