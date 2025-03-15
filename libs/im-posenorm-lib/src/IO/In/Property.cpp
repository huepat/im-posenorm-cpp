#include "Property.h"

#include <cstddef>
#include <stdexcept>
#include <string>

namespace IMPoseNorm::IO::In {

	std::string ToString(
			PropertyType propertyType) {

		switch (propertyType) {
		case PropertyType::UCHAR:
			return "UCHAR";
		case PropertyType::INT:
			return "INT";
		case PropertyType::UINT:
			return "UINT";
		case PropertyType::LONG:
			return "LONG";
		case PropertyType::FLOAT:
			return "FLOAT";
		case PropertyType::DOUBLE:
			return "DOUBLE";
		}

		throw std::runtime_error{ "" };
	}

	Property::Property(
			std::int16_t index,
			std::string label,
			PropertyType type) :
				IsListAttribute(false),
				Index(index),
				Label(label),
				Type(type),
				ListSizeType(PropertyType::UCHAR) {
	}

	Property::Property(
			std::int16_t index,
			std::string label,
			PropertyType type,
			PropertyType listSizeType) :
				IsListAttribute(true),
				Index(index),
				Label(label),
				Type(type),
				ListSizeType(listSizeType) {
	}
}