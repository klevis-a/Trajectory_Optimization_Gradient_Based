//
// Created by klevis on 4/21/18.
//

#ifndef M20IA_TRAJ_OPT_INPUTPARSER_H
#define M20IA_TRAJ_OPT_INPUTPARSER_H
#include <string>
#include "XMLParser.h"

class InputParser : XMLParser {
public:
    InputParser(const char* fileName);
    const std::string &trajectoryFile() const;
    const std::string &trajectoryFolder() const;
    const std::string &seedsFile() const;
    const std::string &toolframesFile() const;
    const std::string &outputDir() const;

private:
    //extracted parameters
    std::string _trajectoryFile;
    std::string _trajectoryFolder;
    std::string _seedsFile;
    std::string _toolframesFile;
    std::string _outputDir;

private:
    void readParameters();
};
#endif //M20IA_TRAJ_OPT_INPUTPARSER_H
