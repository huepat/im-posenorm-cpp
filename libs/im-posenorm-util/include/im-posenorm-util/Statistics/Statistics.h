#pragma once

#include <glm/glm.hpp>

#include <cstddef>
#include <functional>

namespace IMPoseNorm::Util::Statistics {

	template <typename T>
	class Statistics {

	private:
		std::size_t counter;
		T mean;
		T variance;

	public:

		std::size_t GetCounter() const {

			return this->counter;
		}

		const T& GetMean() const {

			return this->mean;
		}

		const T& GetVariance() const {

			return this->variance;
		}

		T GetStandardDeviation() const {

			return this->SquareRoot(
				this->variance);
		}

		void Update(
				const T& value) {

			T oldMean = this->mean;

			this->mean = oldMean 
				+ (1.0 / (this->counter + 1)) 
				* this->Substract(value, oldMean);

			if (this->counter > 0) {

				this->variance = (1.0 - 1.0 / this->counter) * this->variance 
					+ static_cast<double>(this->counter + 1) 
						* this->Square(this->Substract(this->mean, oldMean));
			}

			this->counter++;
		}

	protected:

		Statistics(
				T initializationValue) :
					counter(0),
					mean(initializationValue),
					variance(initializationValue) {
		}

		virtual T Square(
			T value) const = 0;

		virtual T SquareRoot(
			T value) const = 0;

		T Substract(
				const T& value1,
				const T& value2) const {

			return value1 - value2;
		}
	};

	class DoubleStatistics : public Statistics<double> {

	public:

		DoubleStatistics();

		virtual double Square(
			double value) const override;

		virtual double SquareRoot(
			double value) const override;
	};

	class VectorStatistics : public Statistics<glm::dvec3> {

	public:

		VectorStatistics();

		virtual glm::dvec3 Square(
			glm::dvec3 value) const override;

		virtual glm::dvec3 SquareRoot(
			glm::dvec3 value) const override;
	};

	class AngleStatistics : public DoubleStatistics {

	protected:
		double Substract(
			double angle1,
			double angle2) const;
	};
}