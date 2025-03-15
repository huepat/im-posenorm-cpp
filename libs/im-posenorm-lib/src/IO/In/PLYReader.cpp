#include <im-posenorm-lib/IO/In/PLYReader.h>

#include <im-posenorm-lib/Geometry/IShape.h>
#include <im-posenorm-lib/Geometry/Mesh.h>
#include <im-posenorm-lib/Geometry/PointCloud.h>
#include <im-posenorm-lib/IO/PLYReaderWriterBase.h>
#if IMPOSENORM_REPORT_TIMING
	#include <im-posenorm-util/Time/TimeReporter.h>
#endif

#include <glm/glm.hpp>

#include <array>
#include <cstddef>
#include <filesystem>
#include <format>
#include <fstream>
#include <functional>
#include <inttypes.h>
#include <ios>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "HeaderContent.h"
#include "IDecoder.h"
#include "InUtil.h"
#include "Property.h"
#include "ShapeData.h"

#include <iostream> // debug, remove

namespace IMPoseNorm::IO::In {

	const std::uint8_t FACE_VERTICES_SIZE = static_cast<std::uint8_t>(3);

	template <typename TVector>
	using ReadVectorCallback = std::function<TVector(
		const std::array<std::string, 3>& propertyLabels,
		std::map<std::string, PropertyValue>& propertyValues)>;

	std::size_t ParseSize(
			const std::string& string) {

		return static_cast<std::size_t>(
			strtoimax(
				string.c_str(),
				nullptr,
				10));
	}

	PropertyType ParseDataType(
			const std::string& label) {

		if (label == "uchar") {

			return PropertyType::UCHAR;
		}

		if (label == "int") {

			return PropertyType::INT;
		}

		if (label == "uint") {

			return PropertyType::UINT;
		}

		if (label == "long") {

			return PropertyType::LONG;
		}

		if (label == "float") {

			return PropertyType::FLOAT;
		}

		if (label == "double") {

			return PropertyType::DOUBLE;
		}

		throw std::runtime_error(
			std::format(
				"Unexpected property type '{}'.",
				label));
	}

	Property ParseProperty(
			std::int16_t index,
			const std::string& line) {

		std::vector<std::string> values = Split(
			line,
			" ");

		if (values[1] == "list") {

			return Property{
				index,
				values[4],
				ParseDataType(values[3]),
				ParseDataType(values[2])
			};
		}

		return Property{
			index,
			values[2],
			ParseDataType(values[1])
		};
	}

	HeaderContent ReadHeader(
			const std::filesystem::path& filePath) {

		bool isVertexSection = false;
		bool isFaceSection = false;
		std::int16_t propertyIndex = 0;
		std::int32_t lineCount = 0;
		std::size_t vertexCount = 0;
		std::size_t faceCount = 0;
		std::size_t readerPosition = 0;
		std::size_t lineBreakByteSize = 0;
		std::streampos initialReaderPosition = 0;
		std::string line;
		PLYEncoding encoding = PLYEncoding::ASCII;
		std::vector<Property> vertexProperties;
		std::vector<Property> faceProperties;

		std::ifstream file{};

		file.open(filePath);

		if (!file.is_open()) {

			throw std::runtime_error(
				std::format(
					"File '{}' cannot be opened.",
					filePath.string()));
		}

		while (file) {

			std::getline(
				file,
				line);

			if (lineCount == 1) {

				lineBreakByteSize = (file.tellg() - initialReaderPosition) - line.size();

				readerPosition += lineBreakByteSize;
			}

			if (lineCount == 0) {

				initialReaderPosition = file.tellg();

				readerPosition = line.size();
			}
			else {
				readerPosition += line.size() + lineBreakByteSize;
			}

			++lineCount;

			if (line.starts_with("format ")) {

				if (line.starts_with("format ascii")) {

					encoding = PLYEncoding::ASCII;
				}
				else if (line.starts_with("format binary_little_endian")) {

					encoding = PLYEncoding::BINARY_LITTLE_ENDIAN;
				}
				else if (line.starts_with("format binary_big_endian")) {

					encoding = PLYEncoding::BINARY_BIG_ENDIAN;
				}
				else {
					throw std::runtime_error(
						std::format(
							"Unknown format '{}'.",
							Split(
								line,
								std::string{
									"format_"
								}
							)[0]
						));
				}
			}
			else if (line.starts_with("element vertex ")) {

				isVertexSection = true;
				isFaceSection = false;

				vertexCount = ParseSize(
					line.substr(
						std::string{
							"element vertex "
						}.size()));
			}
			else if (line.starts_with("element face ")) {

				isVertexSection = false;
				isFaceSection = true;

				faceCount = ParseSize(
					line.substr(
						std::string{
							"element face "
						}.size()));
			}
			else if (line.starts_with("property ")
					&& (isVertexSection
						|| isFaceSection)) {

				(isVertexSection ?
						vertexProperties :
						faceProperties)
					.push_back(
						ParseProperty(
							propertyIndex,
							line));

				++propertyIndex;
			}
			else if (line == "end_header") {
				break;
			}
		}

		file.close();

		return HeaderContent(
			readerPosition,
			vertexCount,
			faceCount,
			encoding,
			vertexProperties,
			faceProperties);
	}

	void CheckPropertyTripletTypeConsistency(
			const HeaderContent& headerContent,
			const std::array<std::string, 3>& propertyLabels) {

		PropertyType propertyType0 = headerContent.GetVertexProperty(propertyLabels[0]).Type;
		PropertyType propertyType1 = headerContent.GetVertexProperty(propertyLabels[1]).Type;
		PropertyType propertyType2 = headerContent.GetVertexProperty(propertyLabels[2]).Type;

		if (propertyType0 != propertyType1
			|| propertyType0 != propertyType2) {

			throw std::runtime_error(
				std::format(
					"Properties '{}', '{}' and '{}' need to have the same property type.",
					propertyLabels[0],
					propertyLabels[1],
					propertyLabels[2]));
		}
	}

	template <
		typename TSource,
		typename TDestination,
		typename TVector>
	TVector ReadVector(
			const std::array<std::string, 3>& propertyLabels,
			std::map<std::string, PropertyValue>& propertyValues) {

		TSource sourceValue0 = std::get<TSource>(propertyValues[propertyLabels[0]]);
		TSource sourceValue1 = std::get<TSource>(propertyValues[propertyLabels[1]]);
		TSource sourceValue2 = std::get<TSource>(propertyValues[propertyLabels[2]]);

		if (std::is_same<TSource, TDestination>::value) {

			return TVector(
				sourceValue0,
				sourceValue1,
				sourceValue2);
		}

		TDestination destinationValue0 = static_cast<TDestination>(sourceValue0);
		TDestination destinationValue1 = static_cast<TDestination>(sourceValue1);
		TDestination destinationValue2 = static_cast<TDestination>(sourceValue2);

		return TVector{
			destinationValue0,
			destinationValue1,
			destinationValue2
		};
	}

	template <
		typename T,
		typename TAlternative,
		typename TVector>
	ReadVectorCallback<TVector> GetReadVectorCallback(
			PropertyType actualPropertyType,
			PropertyType destinationPropertyType,
			PropertyType alternativeSourcePropertyType,
			const std::array<std::string, 3>& propertyLabels) {

		if (actualPropertyType == destinationPropertyType) {

			return ReadVector<T, T, TVector>;
		}

		if (actualPropertyType == alternativeSourcePropertyType) {

			return ReadVector<TAlternative, T, TVector>;
		}

		throw std::runtime_error(
			std::format(
				"We currently only support type {} or {} for property ({}, {}, {}).",
				ToString(destinationPropertyType),
				ToString(alternativeSourcePropertyType),
				propertyLabels[0],
				propertyLabels[1],
				propertyLabels[2]));
	}

	ReadVectorCallback<glm::dvec3> GetReadDoubleVectorCallback(
			const HeaderContent& headerContent,
			const std::array<std::string, 3>& propertyLabels) {

		return GetReadVectorCallback<
				double,
				float,
				glm::dvec3>(
			headerContent
				.GetVertexProperty(propertyLabels[0])
				.Type,
			PropertyType::DOUBLE,
			PropertyType::FLOAT,
			propertyLabels);
	}

	template <typename T>
	void ReadListPropertyValues(
			bool isRelevant,
			std::unique_ptr<IDecoder>& decoder,
			const Property& listProperty,
			std::map<std::string, PropertyValue>& propertyValues) {

		PropertyValue listSizeValue = decoder->Read(listProperty.ListSizeType);

		propertyValues.insert(
			std::pair<std::string, PropertyValue> {
				std::format(
					"{}_size",
					listProperty.Label),
				listSizeValue
			});

		T listSize = std::get<T>(listSizeValue);

		for (T i = static_cast<T>(0); i < listSize; i++) {

			PropertyValue value = decoder->Read(listProperty.Type);

			if (isRelevant) {

				propertyValues.insert(
					std::pair<std::string, PropertyValue> {
						std::format(
							"{}_{}",
							listProperty.Label,
							i),
						value
					});
			}
		}
	}

	std::map<std::string, PropertyValue> ReadPropertyValues(
			std::unique_ptr<IDecoder>& decoder,
			const std::set<std::string>& relevantPropertyLabels,
			const std::vector<Property>& properties) {

		std::map<std::string, PropertyValue> propertyValues{};

		for (size_t i = 0; i < properties.size(); i++) {

			bool isRelevant = relevantPropertyLabels.contains(properties[i].Label);

			if (properties[i].IsListAttribute) {

				switch (properties[i].ListSizeType) {

				case PropertyType::UCHAR:
					ReadListPropertyValues<std::uint8_t>(
						isRelevant,
						decoder,
						properties[i],
						propertyValues);
					break;

				case PropertyType::INT:
					ReadListPropertyValues<std::int32_t>(
						isRelevant,
						decoder,
						properties[i],
						propertyValues);
					break;

				case PropertyType::UINT:
					ReadListPropertyValues<std::uint32_t>(
						isRelevant,
						decoder,
						properties[i],
						propertyValues);
					break;

				case PropertyType::LONG:
					ReadListPropertyValues<std::int64_t>(
						isRelevant,
						decoder,
						properties[i],
						propertyValues);
					break;

				default:
					throw std::runtime_error(
						"Currently, only types UCHAR, INT, UINT and LONG are supported for list size.");
				}
			}
			else {

				PropertyValue value = decoder->Read(properties[i].Type);

				if (isRelevant) {

					propertyValues.insert(
						std::pair<std::string, PropertyValue> {
							properties[i].Label,
							value
						});
				}
			}
		}

		return propertyValues;
	}

	PLYReader::PLYReader() :
				PLYReaderWriterBase(),
				useColor(false) {
	}

	void PLYReader::ConfigureColorUsage(
			bool useColor) {

		this->useColor = useColor;
	}

	void PLYReader::SetAdditionalPropertyLabels(
			const std::vector<std::string>& additionalPropertyLabels) {
		
		this->additionalPropertyLabels = additionalPropertyLabels;
	}

	void PLYReader::SetAdditionalPropertyLabels(
			std::vector<std::string>&& additionalPropertyLabels) {

		this->additionalPropertyLabels = std::move(additionalPropertyLabels);
	}

	std::unique_ptr<Geometry::IShape> PLYReader::Read(
			const std::filesystem::path& filePath) const {

#if IMPOSENORM_REPORT_TIMING
		Util::Time::TimeReporter timeReporter{};
#endif

		HeaderContent headerContent = ReadHeader(filePath);

		this->CheckVertexPropertyUniqueness(
			headerContent.FaceCount == 0,
			this->useColor,
			this->additionalPropertyLabels);

		this->CheckProperties(headerContent);

		std::unique_ptr<IDecoder> decoder = CreateDecoder(
			headerContent.ReaderPosition,
			headerContent.Encoding,
			filePath);

		ShapeData shapeData {
			this->useColor,
			this->additionalPropertyLabels.size(),
			headerContent
		};

		this->ReadVertexProperties(
			headerContent,
			shapeData,
			decoder);

		if (headerContent.FaceCount > 0) {

			this->ReadFaceProperties(
				headerContent,
				shapeData,
				decoder);
		}

		std::unique_ptr<Geometry::IShape> shape = this->CreateShape(
			headerContent,
			shapeData);

#if IMPOSENORM_REPORT_TIMING
		timeReporter.Report(
			std::format(
				"Loading '{}'",
				filePath.string()));
#endif

		return shape;
	}

	void PLYReader::CheckProperties(
			const HeaderContent& headerContent) const {

		headerContent.CheckVertexProperty(this->coordinatePropertyLabels[0]);
		headerContent.CheckVertexProperty(this->coordinatePropertyLabels[1]);
		headerContent.CheckVertexProperty(this->coordinatePropertyLabels[2]);

		CheckPropertyTripletTypeConsistency(
			headerContent,
			this->coordinatePropertyLabels);

		if (headerContent.FaceCount == 0) {

			headerContent.CheckVertexProperty(this->normalPropertyLabels[0]);
			headerContent.CheckVertexProperty(this->normalPropertyLabels[1]);
			headerContent.CheckVertexProperty(this->normalPropertyLabels[2]);

			CheckPropertyTripletTypeConsistency(
				headerContent,
				this->normalPropertyLabels);
		}
		else {
			
			headerContent.CheckFaceProperty(this->faceVerticesPropertyLabel);
		}

		if (this->useColor) {

			headerContent.CheckVertexProperty(this->colorPropertyLabels[0]);
			headerContent.CheckVertexProperty(this->colorPropertyLabels[1]);
			headerContent.CheckVertexProperty(this->colorPropertyLabels[2]);
		}
		
		for (const std::string& propertyLabel : this->additionalPropertyLabels) {

			headerContent.CheckVertexProperty(propertyLabel);
		}
	}

	void PLYReader::ReadVertexProperties(
			const HeaderContent& headerContent,
			ShapeData& shapeData,
			std::unique_ptr<IDecoder>& decoder) const {

		std::map<std::string, PropertyValue> propertyValues;

		bool hasNormals = headerContent.FaceCount == 0;

		std::set<std::string> relevantPropertyLabels = this->GetVertexPropertyLabels(
			hasNormals,
			this->useColor,
			this->additionalPropertyLabels);

		ReadVectorCallback<glm::dvec3> readCoordinateVectorCallback = GetReadDoubleVectorCallback(
			headerContent,
			this->coordinatePropertyLabels);

		ReadVectorCallback<glm::dvec3> readNormalVectorCallback = hasNormals ?
			GetReadDoubleVectorCallback(
				headerContent,
				this->normalPropertyLabels) :
			nullptr;

		for (size_t vertexIndex = 0; 
				vertexIndex < headerContent.VertexCount;
				++vertexIndex) {

			propertyValues = ReadPropertyValues(
				decoder,
				relevantPropertyLabels,
				headerContent.VertexProperties);

			shapeData.Vertices[vertexIndex] = readCoordinateVectorCallback(
				this->coordinatePropertyLabels,
				propertyValues);

			if (hasNormals) {

				shapeData.Normals[vertexIndex] = readNormalVectorCallback(
					this->normalPropertyLabels,
					propertyValues);
			}

			if (this->useColor) {

				shapeData.Colors[vertexIndex] = glm::u8vec3{
					std::get<std::uint8_t>(propertyValues[this->colorPropertyLabels[0]]),
					std::get<std::uint8_t>(propertyValues[this->colorPropertyLabels[1]]),
					std::get<std::uint8_t>(propertyValues[this->colorPropertyLabels[2]])
				};
			}

			if (this->additionalPropertyLabels.size() > 0) {

				shapeData.AdditionalProperties[vertexIndex] = std::vector<float>(
					this->additionalPropertyLabels.size());

				for (size_t propertyIndex = 0; 
						propertyIndex < this->additionalPropertyLabels.size(); 
						++propertyIndex) {

					shapeData.AdditionalProperties[vertexIndex][propertyIndex] = std::get<float>(
						propertyValues[this->additionalPropertyLabels[propertyIndex]]);
				}
			}

			decoder->SwitchLine();
		}
	}

	void PLYReader::ReadFaceProperties(
			const HeaderContent& headerContent,
			ShapeData& shapeData,
			std::unique_ptr<IDecoder>& decoder) const {

		std::map<std::string, PropertyValue> propertyValues;

		std::set<std::string> relevantPropertyLabels{
			this->faceVerticesPropertyLabel
		};

		std::string faceVerticesSizePropertyLabel = std::format(
			"{}_size",
			this->faceVerticesPropertyLabel);

		std::array<std::string, 3> faceVerticesPropertyLabels{
			std::format(
				"{}_0",
				this->faceVerticesPropertyLabel),
			std::format(
				"{}_1",
				this->faceVerticesPropertyLabel),
			std::format(
				"{}_2",
				this->faceVerticesPropertyLabel)
		};

		ReadVectorCallback<glm::u32vec3> readVectorCallback = GetReadVectorCallback<
				std::uint32_t,
				std::int32_t,
				glm::u32vec3>(
			headerContent
				.GetFaceProperty(this->faceVerticesPropertyLabel)
				.Type,
			PropertyType::UINT,
			PropertyType::INT,
			faceVerticesPropertyLabels);

		for (size_t faceIndex = 0;
				faceIndex < headerContent.FaceCount;
				++faceIndex) {

			propertyValues = ReadPropertyValues(
				decoder,
				relevantPropertyLabels,
				headerContent.FaceProperties);

			if (std::get<std::uint8_t>(propertyValues[faceVerticesSizePropertyLabel]) != FACE_VERTICES_SIZE) {

				throw std::runtime_error(
					"We currently only support faces with 3 vertices.");

			}

			shapeData.Faces[faceIndex] = readVectorCallback(
				faceVerticesPropertyLabels,
				propertyValues);

			decoder->SwitchLine();
		}
	}

	std::unique_ptr<Geometry::IShape> PLYReader::CreateShape(
			const HeaderContent& headerContent,
			ShapeData& shapeData) const {

		std::unique_ptr<Geometry::IShape> shape = nullptr;

		if (headerContent.FaceCount > 0) {

			shape = std::make_unique<Geometry::Mesh>(
				std::move(shapeData.Vertices),
				std::move(shapeData.Faces));
		}
		else {

			shape = std::make_unique<Geometry::PointCloud>(
				std::move(shapeData.Vertices),
				std::move(shapeData.Normals));
		}

		if (this->useColor) {

			shape->SetColors(
				std::move(shapeData.Colors));
		}

		if (this->additionalPropertyLabels.size() > 0) {

			shape->SetAdditionalProperties(
				this->additionalPropertyLabels,
				std::move(shapeData.AdditionalProperties));
		}

		return shape;
	}
}