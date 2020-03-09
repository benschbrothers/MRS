#include "Matrix.h"

#include <random>
#include <cmath>

Matrix::Matrix(int row, int col)
{
	rows = row;
	cols = col;
	if (rows <= 0 || cols <= 0)
	{
		p = new float[1];
		throw "Invalid size";
	}
	p = new float[rows * cols];
}

Matrix::Matrix(const Matrix& other)
{
	rows = other.rows;
	cols = other.cols;

	p = new float[rows * cols];
	for (int i = 0; i < (rows * cols); i++)
	{
		p[i] = other.p[i];
	}
}

Matrix::~Matrix()
{
	delete[] p;
}

int Matrix::getAmountOfRows()const
{
	return rows;
}

int Matrix::getAmountOfCollumns()const
{
	return cols;
}

float Matrix::getElement(int i, int j)const
{
	if (i < 0)i = rows + i;
	if (j < 0)j = cols + j;

	return p[i * cols + j];
}

float Matrix::getElementScaled(int i, int j, float min, float max)const
{
	if (i < 0)i = rows + i;
	if (j < 0)j = cols + j;

	return p[i * cols + j] * (max - min) + min;
}

std::vector<float> Matrix::getAllElements()const
{
	std::vector<float> data;

	for (int i = 0; i < (rows * cols); i++)
	{
		data.push_back(p[i]);
	}

	return data;
}

std::vector<float> Matrix::getAllElementsScaled(float min, float max) const
{
	std::vector<float> data;

	for (int i = 0; i < (rows * cols); i++)
	{
		data.push_back(p[i] * (max - min) + min);
	}

	return data;
}

Matrix Matrix::getRow(int i) const
{
	if (i < 0)i = rows + i;

	Matrix T(1, cols);
	for (int k = 0; k < cols; k++)
	{
		T.p[k] = p[i * cols + k];
	}
	return T;
}

Matrix Matrix::getCollum(int i) const
{
	if (i < 0)i = cols + i;

	Matrix T(rows, 1);
	for (int k = 0; k < rows; k++)
	{
		T.p[k] = p[k * cols + i];
	}
	return T;
}

void Matrix::setElement(int i, int j, float e)
{
	if (i < 0)i = rows + i;
	if (j < 0)j = cols + j;

	p[i * cols + j] = e;
}

void Matrix::set(float e)
{
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			p[i * cols + j] = e;
		}
	}
}

void Matrix::set(std::vector<float>::iterator begin, std::vector<float>::iterator end)
{
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			p[i * cols + j] = *begin;
			begin++;

			if (begin == end)
			{
				return;
			}
		}
	}
}

Matrix Matrix::operator=(const Matrix& eq)
{
	delete[] p;

	rows = eq.rows;
	cols = eq.cols;

	p = new float[rows * cols];
	for (int i = 0; i < (rows * cols); i++)
	{
		p[i] = eq.p[i];
	}

	return *this;
}

Matrix Matrix::operator+(const Matrix& m2)const
{
	Matrix T(rows, cols);
	if (rows == m2.rows && cols == m2.cols)
	{
		for (int i = 0; i < rows; i++)
		{
			for (int j = 0; j < cols; j++)
			{
				T.p[i * cols + j] = p[i * cols + j] + m2.p[i * cols + j];
			}
		}
	}
	else
	{
		throw "error: +";
	}
	return T;
}

Matrix Matrix::operator-(const Matrix& m2)const
{
	Matrix T(rows, cols);
	if (rows == m2.rows && cols == m2.cols)
	{
		for (int i = 0; i < rows; i++)
		{
			for (int j = 0; j < cols; j++)
			{
				T.p[i * cols + j] = p[i * cols + j] - m2.p[i * cols + j];
			}
		}
	}
	else
	{
		throw "error:-";
	}
	return T;
}

Matrix Matrix::operator-()
{
	Matrix T(rows, cols);
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			T.p[i * cols + j] = -p[i * cols + j];
		}
	}
	return T;
}

void Matrix::putSum(const Matrix& m2)
{
	if (rows == m2.rows && cols == m2.cols)
	{
		for (int i = 0; i < rows; i++)
		{
			for (int j = 0; j < cols; j++)
			{
				p[i * cols + j] += m2.p[i * cols + j];
			}
		}
	}
	else
	{
		throw "error: +";
	}
}

void Matrix::putMinus(const Matrix& m2)
{
	if (rows == m2.rows && cols == m2.cols)
	{
		for (int i = 0; i < rows; i++)
		{
			for (int j = 0; j < cols; j++)
			{
				p[i * cols + j] -= m2.p[i * cols + j];
			}
		}
	}
	else
	{
		throw "error:-";
	}
}

void Matrix::putNegate()
{
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			p[i * cols + j] = -p[i * cols + j];
		}
	}
}

Matrix Matrix::operator*(const Matrix& b)const
{
	Matrix T(rows, b.cols);
	if (cols == b.rows)
	{
		for (int i = 0; i < rows; i++)
		{
			for (int k = 0; k < b.cols; k++)
			{
				T.p[i * T.cols + k] = 0;
				for (int j = 0; j < cols; j++)
				{
					T.p[i * T.cols + k] += p[i * cols + j] * b.p[j * b.cols + k];
				}
			}
		}
	}
	else
	{
		throw "error: *";
	}
	return T;
}

Matrix Matrix::operator*(float e)const
{
	Matrix T(rows, cols);
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			T.p[i * cols + j] = p[i * cols + j] * e;
		}
	}

	return T;
}

void Matrix::putMultiplication(float e)
{
	Matrix T(rows, cols);
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			p[i * cols + j] *= e;
		}
	}
}

void Matrix::putElementWiseMultiplication(const Matrix& other)
{
	if (rows == other.rows && cols == other.cols)
	{
		for (int i = 0; i < rows; i++)
		{
			for (int j = 0; j < cols; j++)
			{
				p[i * cols + j] *= other.p[i * cols + j];
			}
		}
	}
	else
	{
		throw "error: *";
	}
}

Matrix Matrix::getElementWiseMultiplication(const Matrix& other)const
{
	Matrix T(rows, cols);
	if (rows == other.rows && cols == other.cols)
	{
		for (int i = 0; i < rows; i++)
		{
			for (int j = 0; j < cols; j++)
			{
				T.p[i * cols + j] = p[i * cols + j] * other.p[i * cols + j];
			}
		}
	}
	else
	{
		throw "error: *";
	}
	return T;
}

Matrix Matrix::getSigmoid()const
{
	Matrix T(rows, cols);
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			T.p[i * cols + j] = 1.0f / (1.0f + std::exp(-p[i * cols + j]));
		}
	}
	return T;
}

Matrix Matrix::getSigmoidPrime()const
{
	Matrix T(rows, cols);
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			T.p[i * cols + j] = std::exp(-p[i * cols + j]) / ((1.0f + std::exp(-p[i * cols + j])) * (1.0 + std::exp(-p[i * cols + j])));
		}
	}
	return T;
}

void Matrix::putSigmoid()
{
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			p[i * cols + j] = 1.0f / (1.0f + std::exp(-p[i * cols + j]));
		}
	}
}

void Matrix::putSigmoidPrime()
{
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			p[i * cols + j] = std::exp(-p[i * cols + j]) / ((1.0f + std::exp(-p[i * cols + j])) * (1.0 + std::exp(-p[i * cols + j])));
		}
	}
}

Matrix Matrix::getTanh()const
{
	Matrix T(rows, cols);
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			T.p[i * cols + j] = std::tanh(p[i * cols + j]);
		}
	}
	return T;
}

void Matrix::putTanh()
{
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			p[i * cols + j] = std::tanh(p[i * cols + j]);
		}
	}
}

void Matrix::putExp()
{
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			p[i * cols + j] = std::exp(p[i * cols + j]);
		}
	}
}

float Matrix::sumElements() const
{
	float total = 0;
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			total += p[i * cols + j];
		}
	}
	return total;
}

void Matrix::putSquare()
{
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			p[i * cols + j] *= p[i * cols + j];
		}
	}
}

Matrix Matrix::getSquare()const
{
	Matrix T(rows, cols);
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			T.p[i * cols + j] = p[i * cols + j] * p[i * cols + j];
		}
	}
	return T;
}

float Matrix::getSquareSum() const
{
	float sum = 0;
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			sum += p[i * cols + j] * p[i * cols + j];
		}
	}
	return sum;
}

Matrix Matrix::getTranspose()const
{
	Matrix T(cols, rows);
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			T.p[i * T.rows + j] = p[i * cols + j];
		}
	}
	return T;
}

Matrix Matrix::getHorizontalStitch(const Matrix& other)const
{
	if (rows != other.rows)
	{
		throw "error: H-Stitch";
	}
	Matrix T(rows, cols + other.cols);
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			T.p[i * T.cols + j] = p[i * cols + j];
		}
		for (int j = 0; j < other.cols; j++)
		{
			T.p[i * T.cols + j + cols] = other.p[i * other.cols + j];
		}
	}
	return T;
}

Matrix Matrix::getVerticalStitch(const Matrix& other)const
{
	if (cols != other.cols)
	{
		throw "error: V-Stitch";
	}
	Matrix T(rows + other.rows, cols);
	for (int i = 0; i < cols; i++)
	{
		for (int j = 0; j < rows; j++)
		{
			T.p[j * T.cols + i] = p[j * cols + i];
		}
		for (int j = 0; j < other.rows; j++)
		{
			T.p[(j + rows) * T.cols + i] = other.p[j * other.cols + i];
		}
	}
	return T;
}