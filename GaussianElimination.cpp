#include "GaussianElimination.h"
#include <cmath>
#include <stdexcept>

std::vector<double> gaussianElimination(
	std::vector<std::vector<double>>& A,
	std::vector<double>& b)
{
	int n = A.size();

	// Forward elimination
	for (int i = 0; i < n; i++) {
		// ----- Pivot selection (選最大 pivot，避免除以 0) -----
		double maxVal = std::abs(A[i][i]);
		int pivot = i;

		for (int r = i + 1; r < n; r++) {
			if (std::abs(A[r][i]) > maxVal) {
				maxVal = std::abs(A[r][i]);
				pivot = r;
			}
		}

		// Swap rows if needed
		if (pivot != i) {
			std::swap(A[i], A[pivot]);
			std::swap(b[i], b[pivot]);
		}

		// Check pivot != 0
		if (std::abs(A[i][i]) < 1e-12) {
			throw std::runtime_error("Matrix is singular or nearly singular.");
		}

		// Eliminate rows below
		for (int r = i + 1; r < n; r++) {
			double factor = A[r][i] / A[i][i];

			for (int c = i; c < n; c++) {
				A[r][c] -= factor * A[i][c];
			}

			b[r] -= factor * b[i];
		}
	}

	// Back substitution (回代求解)
	std::vector<double> x(n);

	for (int i = n - 1; i >= 0; i--) {
		double sum = b[i];

		for (int c = i + 1; c < n; c++) {
			sum -= A[i][c] * x[c];
		}

		x[i] = sum / A[i][i];
	}

	return x;
}
