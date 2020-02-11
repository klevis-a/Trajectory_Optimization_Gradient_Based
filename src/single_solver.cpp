//
// Created by klevis on 10/31/17.
//
#include <csv.h>
#include <ConfigParser.h>
#include <InputParser.h>
#include <TrajOpt_Solver.h>
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>
#include <regex>

using namespace pagmo;
using namespace std;
using namespace Eigen;
using namespace boost::filesystem;


int main(int argc, char** argv)
{
    ConfigParser configParser(argv[1]);
    InputParser inputParser(argv[2]);

    path framesPath(inputParser.trajectoryFile());
    auto framesDir = framesPath.parent_path();
    path outputDir(inputParser.outputDir());
    auto framesFilePath = framesPath.filename().string();

    //read in the trajectory
    io::CSVReader<16> input(inputParser.toolframesFile());
    double t00,t01,t02,t03,t10,t11,t12,t13,t20,t21,t22,t23,t30,t31,t32,t33;

    MatrixXd firstFrameInv;
    int rowNum=1;

    while(input.read_row(t00,t01,t02,t03,t10,t11,t12,t13,t20,t21,t22,t23,t30,t31,t32,t33)) {

        auto newJointsFileName = regex_replace(framesFilePath, regex("smoothFrames"), "jointsTF" + to_string(rowNum));
        auto newOffsetFileName = regex_replace(framesFilePath, regex("smoothFrames"), "offsetTF" + to_string(rowNum));
        auto jointsFilePath = outputDir / path(newJointsFileName);
        auto offsetFilePath = outputDir / path(newOffsetFileName);

        MatrixXd currentFrame(4, 4);
        currentFrame << t00, t01, t02, t03, t10, t11, t12, t13, t20, t21, t22, t23, t30, t31, t32, t33;
        TrajOpt_Solver solver(configParser, inputParser.trajectoryFile(), inputParser.seedsFile(), currentFrame);
        solver.solve();
        solver.printResults(jointsFilePath.string(), offsetFilePath.string());
    }
}
