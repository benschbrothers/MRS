#pragma once

#include <vector>
#include <numeric>
#include <functional>
#include <random>

typedef std::vector<double> Individual;
typedef std::vector<Individual> Population;

std::vector<int> argSortDesc(const std::vector<double>& list)
{
	std::vector<int> idx(list.size());
	std::iota(idx.begin(), idx.end(), 0);

	sort(idx.begin(), idx.end(), [&list](int i1, int i2) {return list[i1] > list[i2]; });

	return idx;
}

class GeneticSearch
{
public:
	GeneticSearch(int pParameters, int pGenerations, int pPopulationSize)
	{
		parameters = pParameters;
		generations = pGenerations;
		populationSize = pPopulationSize;
	}

	void setFitnessFunction(std::function<double(const Individual&)> pFitnessFunction)
	{
		fitnessFunction = pFitnessFunction;
	}

	void run()
	{
		static std::random_device rd;
		static std::mt19937 gen(rd());
		static std::uniform_real_distribution<double> uniform(0, populationSize * top);

		Population population;

		for (int i = 0; i < populationSize; i++)
		{
			population.emplace_back(newIndividual());
		}

		// Run generations
		for (int i = 0; i < generations; i++)
		{
			std::vector<double> fitness = getFitness(population);
			std::vector<int> fitnessOrder = argSortDesc(fitness);

			best = population[fitnessOrder[0]];

			Population newPopulation;		
			for (int j = 0; j < elitism * populationSize; j++)
			{
				newPopulation.emplace_back(population[fitnessOrder[j]]);
			}

			while (newPopulation.size() < populationSize)
			{
				Individual child = crossover(population[uniform(gen)], population[uniform(gen)]);
				child = mutate(child);
				newPopulation.emplace_back(child);
			}

			population = newPopulation;
		}
	}

private:
	std::vector<double> getFitness(const Population& population)
	{
		std::vector<double> fitness;

		for (const auto& individual : population)
		{
			fitness.push_back(fitnessFunction(individual));
		}

		return fitness;
	}

	Individual crossover(const Individual& p1, const Individual& p2)
	{
		static std::random_device rd;
		static std::mt19937 gen(rd());
		static std::bernoulli_distribution bernoulli(0.5);

		Individual child;
		for (int i = 0; i < parameters; i++)
		{
			if (bernoulli(gen))
			{
				child.push_back(p1[i]);
			}
			else
			{
				child.push_back(p2[i]);
			}
		}

		return child;
	}

	Individual mutate(Individual individual)
	{
		static std::random_device rd;
		static std::mt19937 gen(rd());

		static std::bernoulli_distribution bernoulli(mutationRate);
		static std::uniform_real_distribution<double> uniform(lowerBound, upperBound);

		for (auto v : individual)
		{
			if (bernoulli(gen))
			{
				v = uniform(gen);
			}
		}

		return individual;
	}

	Individual newIndividual()
	{
		static std::random_device rd;
		static std::mt19937 gen(rd());
		static std::uniform_real_distribution<double> uniform(lowerBound, upperBound);

		Individual individual;

		for (int i = 0; i < parameters; i++)
		{
			individual.push_back(uniform(gen));
		}
	}

private:

	int parameters;
	int generations;
	int populationSize;

	double mutationRate = 0.1;
	double lowerBound = -1;
	double upperBound = 1;
	double elitism = 0.05;
	double top = 0.25;

	std::function<double(const Individual&)> fitnessFunction;

	Individual best;
};
