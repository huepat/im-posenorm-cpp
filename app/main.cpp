#include <glm/glm.hpp>
#include <im-posenorm-eval/Config.h>
#include <im-posenorm-eval/Eval.h>
#include <im-posenorm-lib/Geometry/AABox.h>
#include <im-posenorm-lib/Geometry/Mesh.h>
#include <im-posenorm-lib/IMPoseNorm.h>
#include <im-posenorm-lib/IO/Out/PLYWriter.h>

#include <iostream>
#include <string>
#include <vector>

#include <im-posenorm-lib/IO/In/PLYReader.h>

//#include <im-posenorm-util/Parallel/Parallel.h>

using namespace IMPoseNorm;

int main(
        int argc, 
        char* argv[]) {

    //glm::dvec3 verticalAxis{ 0.0, 0.0, 1.0 };
    //glm::dvec3 horizontalAxis{ 1.0, 0.0, 0.0 };

    //std::vector<std::string> filePaths{
    //    //R"(E:\phuebner\data\Nextcloud\IndoorReconstruction_Data\E6_mesh_rectified_AlignmentGT.ply)",
    //    /*R"(E:\phuebner\data\Nextcloud\IndoorReconstruction_Data\E6_mesh_rectified_AlignmentGT.ply)",
    //    R"(E:\phuebner\data\Nextcloud\IndoorReconstruction_Data\E6_mesh_rectified_AlignmentGT.ply)",
    //    R"(E:\phuebner\data\Nextcloud\IndoorReconstruction_Data\E6_mesh_rectified_AlignmentGT.ply)"*/

    //    /*R"(E:\phuebner\data\Data\IndoorReconstruction\ISPRS_Benchmark\CaseStudy1\subsampled_2cm_normals.ply)",
    //    R"(E:\phuebner\data\Data\IndoorReconstruction\ISPRS_Benchmark\CaseStudy2\subsampled_2cm_normals.ply)",
    //    R"(E:\phuebner\data\Data\IndoorReconstruction\ISPRS_Benchmark\CaseStudy3\subsampled_2cm_normals.ply)",
    //    R"(E:\phuebner\data\Data\IndoorReconstruction\ISPRS_Benchmark\CaseStudy4\subsampled_2cm_normals.ply)",*/
    //    /*R"(E:\phuebner\data\Data\IndoorReconstruction\ISPRS_Benchmark\CaseStudy5\subsampled_2cm_normals.ply)",*/ // test this!
    //    //R"(E:\phuebner\data\Data\IndoorReconstruction\ISPRS_Benchmark\CaseStudy6\subsampled_2cm_normals.ply)"

    //    R"(C:\Users\phuebner\data\tmp\tmp\demo_mesh_normalized.ply)"
    //};

    //for (const std::string& filePath : filePaths) {

    //    Eval::Config config{
    //        DegreeToRadian(25.0),
    //        verticalAxis,
    //        horizontalAxis,
    //        filePath
    //    };

    //    config.SetOutputPLY(true);

    //    config.SetSampleCount(1);

    //    config.SetOutputDirectoryPath(R"(C:\Users\phuebner\data\tmp\tmp)");

    //    Eval::Evaluate(config);

    //    std::cout << std::endl;
    //    std::cout << "--------------------------------------------------------------------------------------" << std::endl;
    //    std::cout << std::endl;
    //}

    Geometry::Mesh mesh {
        std::vector<Geometry::Mesh> {

            Geometry::AABox {
                glm::dvec3 { 0.0, 0.0, 0.0 },
                glm::dvec3 { 5.0, 10.0, 2.5 }
            }.ConvertToMesh(),

            Geometry::AABox {
                glm::dvec3 { 5.0, 0.0, 0.0 },
                glm::dvec3 { 15.0, 5.0, 2.5 }
            }.ConvertToMesh()
        }
    };

    glm::dvec3 verticalAxis{ 0.0, 0.0, 1.0 };
    glm::dvec3 horizontalAxis{ 1.0, 0.0, 0.0 };

    glm::dmat3 rotation = GetRotationAroundAxis(
            DegreeToRadian(13.0),
            horizontalAxis)
        * GetRotationAroundAxis(
            DegreeToRadian(19.0),
            verticalAxis);

    mesh.Rotate(
        mesh.GetCentroid(),
        rotation);

    IO::Out::PLYWriter writer{};

    writer.Write(
        R"(C:\Users\phuebner\data\tmp\tmp\demo_mesh.ply)",
        mesh);

    NormalizePose(
        mesh,
        verticalAxis,
        horizontalAxis,
        true);

    writer.Write(
        R"(C:\Users\phuebner\data\tmp\tmp\demo_mesh_normalized.ply)",
        mesh);

    Eval::Config config{
        DegreeToRadian(25.0),
        verticalAxis,
        horizontalAxis,
        R"(C:\Users\phuebner\data\tmp\tmp\demo_mesh_normalized.ply)"
    };

    config.SetSampleCount(20);

    Eval::Evaluate(config);

    return 0;
}