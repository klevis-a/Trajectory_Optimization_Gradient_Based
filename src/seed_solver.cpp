//
// Created by klevis on 2/14/18.
//

#include <ConfigParser.h>
#include <TrajOpt_Solver.h>
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <regex>
#include <iostream>
#include <csv.h>

using namespace std;
using namespace boost::filesystem;
using namespace Eigen;

int main(int argc, char** argv)
{
    ConfigParser configParser(argv[1]);
    InputParser inputParser(argv[2]);

    io::CSVReader<16> input(inputParser.toolframesFile());
    double t00,t01,t02,t03,t10,t11,t12,t13,t20,t21,t22,t23,t30,t31,t32,t33;
    input.read_row(t00,t01,t02,t03,t10,t11,t12,t13,t20,t21,t22,t23,t30,t31,t32,t33);
    MatrixXd currentFrame(4, 4);
    currentFrame << t00, t01, t02, t03, t10, t11, t12, t13, t20, t21, t22, t23, t30, t31, t32, t33;

    const boost::regex my_filter(".*jointsS\\d\\.txt");
    for(auto i=directory_iterator(inputParser.trajectoryFolder()); i!=directory_iterator(); i++)
    {
        if(is_directory(i->path()))
        {
            for(auto j=directory_iterator(i->path()); j!=directory_iterator(); j++)
            {
                // Skip if not a file
                if(!is_regular_file( j->status() ) ) continue;

                boost::smatch what;
                //skip if the file does not match the filter
                if(!boost::regex_match( j->path().string(), what, my_filter ) ) continue;

                auto filePath = j->path();
                auto dir = filePath.parent_path();
                auto fileName = filePath.filename().string();
                auto smoothFramesFile = regex_replace(fileName, regex("jointsS\\d\\.txt"), "smoothFrames.txt");
                auto jointsFile = regex_replace(fileName, regex("joints"), "jointsTrajOpt");
                auto jointsFile2 = regex_replace(fileName, regex("joints"), "jointsTrajOpt2");
                auto offsetFile = regex_replace(fileName, regex("joints"), "offset");
                auto offsetFile2 = regex_replace(fileName, regex("joints"), "offset2");
                auto smoothFramesFilePath = dir / path(smoothFramesFile);
                auto jointsFilePath = dir / path(jointsFile);
                auto jointsFilePath2 = dir / path(jointsFile2);
                auto offsetFilePath = dir / path(offsetFile);
                auto offsetFilePath2 = dir / path(offsetFile2);

                cout << "Processing file: " << fileName << endl;
                cout << "Smooth Frames file: " << smoothFramesFilePath.string() << endl;
                cout << "Joints file: " << jointsFilePath.string() << endl;
                cout << "Offset file: " << offsetFilePath.string() << endl;
                cout << " .... " << endl;

                TrajOpt_Solver solverInitSeed(configParser, smoothFramesFilePath.string(), filePath.string(),currentFrame,false);
                solverInitSeed.solve();
                solverInitSeed.printResults(jointsFilePath.string(), offsetFilePath.string());

                TrajOpt_Solver solverCalcSeed(configParser, smoothFramesFilePath.string(), filePath.string(),currentFrame,true);
                solverCalcSeed.solve();
                solverCalcSeed.printResults(jointsFilePath2.string(), offsetFilePath2.string());
            }
        }
    }
}