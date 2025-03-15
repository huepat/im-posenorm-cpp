project(${PROJECT_NAME})

add_library(im-posenorm-lib STATIC

	# IMPoseNorm
	"include/im-posenorm-lib/IMPoseNorm.h"
	"src/IMPoseNorm.cpp"	
	"src/Config.h"
	"src/Grid2D.h"
	"src/HorizontalPoseNormalization.h"
	"src/HorizontalPoseNormalization.cpp"
	"src/HorizontalPoseUnambiguation.h"
	"src/HorizontalPoseUnambiguation.cpp"
	"src/Util.h"
	"src/Util.cpp"
	"src/VerticalPoseNormalization.h"
	"src/VerticalPoseNormalization.cpp"
	"src/WeightedValue.h"

	# IMPoseNorm::Geometry
	"include/im-posenorm-lib/Geometry/AABox.h"
	"include/im-posenorm-lib/Geometry/Mesh.h"
	"include/im-posenorm-lib/Geometry/PointCloud.h"
	"src/Geometry/AABox.cpp"
	"src/Geometry/Mesh.cpp"
	"src/Geometry/PointCloud.cpp"
	"src/Geometry/GeometryUtil.h"
	"src/Geometry/GeometryUtil.cpp"

	# IMPoseNorm::IO
	"include/im-posenorm-lib/IO/PLY.h"
	"include/im-posenorm-lib/IO/PLYReaderWriterBase.h"
	"src/IO/PLY.cpp"
	"src/IO/PLYReaderWriterBase.cpp"
	"src/IO/IOUtil.h"
	"src/IO/IOUtil.cpp"

	# IMPoseNorm::IO::In
	"include/im-posenorm-lib/IO/In/PLYReader.h"	
	"src/IO/In/PLYReader.cpp"
	"src/IO/In/ASCIIDecoder.h"
	"src/IO/In/ASCIIDecoder.cpp"
	"src/IO/In/BinaryDecoder.h"
	"src/IO/In/BinaryDecoder.cpp"
	"src/IO/In/HeaderContent.h"
	"src/IO/In/HeaderContent.cpp"
	"src/IO/In/IDecoder.h"
	"src/IO/In/IDecoder.cpp"
	"src/IO/In/InUtil.h"
	"src/IO/In/InUtil.cpp"
	"src/IO/In/Property.h"
	"src/IO/In/Property.cpp"
	"src/IO/In/ShapeData.h"
	"src/IO/In/ShapeData.cpp"

	# IMPoseNorm::IO::Out
	"include/im-posenorm-lib/IO/Out/PLYWriter.h"
	"src/IO/Out/PLYWriter.cpp"
	"src/IO/Out/ASCIIEncoder.h"
	"src/IO/Out/ASCIIEncoder.cpp"
	"src/IO/Out/BinaryEncoder.h"
	"src/IO/Out/BinaryEncoder.cpp"
	"src/IO/Out/IEncoder.h"
	"src/IO/Out/IEncoder.cpp"
)

target_link_libraries(im-posenorm-lib PRIVATE
	im-posenorm-util
)

target_include_directories(im-posenorm-lib PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/im-posenorm-lib>
    $<INSTALL_INTERFACE:include/im-posenorm-lib>
	${CMAKE_SOURCE_DIR}/dependencies/glm
)

target_include_directories(im-posenorm-lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)