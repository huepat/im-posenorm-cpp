#pragma once

#include <cstddef>
#include <stdexcept>
#include <string>
#include <variant>

namespace IMPoseNorm::IO::In {

	using PropertyValue = std::variant<
		std::uint8_t,
		std::int32_t,
		std::uint32_t,
		std::int64_t,
		float,
		double>;

	enum PropertyType {
		UCHAR,
		INT,
		UINT,
		LONG,
		FLOAT,
		DOUBLE
	};

	std::string ToString(
		PropertyType propertyType);

	struct Property {

	public:
		bool IsListAttribute;
		std::int16_t Index;
		std::string Label;
		PropertyType Type;
		PropertyType ListSizeType;

		Property(
			std::int16_t index,
			std::string label,
			PropertyType type);

		Property(
			std::int16_t index,
			std::string label,
			PropertyType type,
			PropertyType listSizeType);
	};
}