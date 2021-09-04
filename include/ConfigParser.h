//
// Created by klevis on 12/7/17.
//

#ifndef M20IA_TRAJ_OPT_CONFIGPARSER_H
#define M20IA_TRAJ_OPT_CONFIGPARSER_H

#include <string>
#include <vector>

#include <tinyxml2.h>

#include "XMLParser.h"

class ConfigParser : XMLParser {
public:
    ConfigParser(const char* fileName);
    bool checkGradient() const;
    double gradientTolerance() const;
    const std::string &urdfFile() const;
    const std::string &baseName() const;
    const std::string &eeName() const;
    double tolerance() const;
    const std::vector<double> &velLimits() const;
    double majorIterLimit() const;
    double iterLimit() const;
    bool calcInitPosition() const;
    const std::vector<double> &initPosition() const;
    const std::vector<double> &posLB() const;
    const std::vector<double> &posUB() const;
    double rotLB() const;
    double rotUB() const;
    const std::string &snoptcLib() const;

private:
    //extracted parameters
    bool _checkGradient;
    double _gradientTolerance;
    std::string _urdfFile;
    std::string _baseName;
    std::string _eeName;
    double _tolerance;
    std::vector<double> _velLimits;
    double _majorIterLimit;
    double _iterLimit;
    bool _calcInitPosition;
    std::vector<double> _initPosition;
    std::vector<double> _posLB;
    std::vector<double> _posUB;
    double _rotLB;
    double _rotUB;
    std::string _snoptclib;
};


#endif //M20IA_TRAJ_OPT_CONFIGPARSER_H
