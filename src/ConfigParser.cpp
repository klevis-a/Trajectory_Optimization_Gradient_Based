//
// Created by klevis on 12/7/17.
//

#include <iostream>
#include <iterator>
#include <sstream>

#include "ConfigParser.h"
#include "csv.h"

using std::vector;
using std::string;


ConfigParser::ConfigParser(const char *fileName) : XMLParser(fileName) {
    //check gradient
    _checkGradient = parseBool("CheckGradient");
    //gradient tolerance
    _gradientTolerance = parseDouble("GradientTolerance");
    //URDF file
    _urdfFile = string(parseText("URDF_File"));
    //base name
    _baseName = parseText("BaseName");
    //end effector name
    _eeName = parseText("EndEffectorName");
    //tolerance
    _tolerance = parseDouble("Tolerance");
    //velocity limits
    _velLimits = stringToVector(parseText("VelocityLimits"));
    //major iteration limit
    _majorIterLimit = parseDouble("MajorIterationLimit");
    //iteration limit
    _iterLimit = parseDouble("IterationLimit");
    //whether to calculate the initial position or use the seed
    _calcInitPosition = parseBool("CalcInitPosition");
    //read initial position
    _initPosition = stringToVector(parseText("InitPosition"));
    //position lower bound
    _posLB = stringToVector(parseText("TransformPosBoundLower"));
    //position upper bound
    _posUB = stringToVector(parseText("TransformPosBoundUpper"));
    //z rotation lower bound
    _rotLB = parseDouble("TransformRotBoundLower");
    //z rotation upper bound
    _rotUB = parseDouble("TransformRotBoundUpper");
    //location of SNOPT_C lib
    _snoptclib = parseText("Snopt_C_Lib");
}

bool ConfigParser::checkGradient() const {
    return _checkGradient;
}

double ConfigParser::gradientTolerance() const {
    return _gradientTolerance;
}

const string &ConfigParser::urdfFile() const {
    return _urdfFile;
}

const string &ConfigParser::baseName() const {
    return _baseName;
}

const string &ConfigParser::eeName() const {
    return _eeName;
}

double ConfigParser::tolerance() const {
    return _tolerance;
}

const vector<double> &ConfigParser::velLimits() const {
    return _velLimits;
}

double ConfigParser::majorIterLimit() const {
    return _majorIterLimit;
}

double ConfigParser::iterLimit() const {
    return _iterLimit;
}

bool ConfigParser::calcInitPosition() const {
    return _calcInitPosition;
}

const vector<double> &ConfigParser::initPosition() const {
    return _initPosition;
}

const vector<double> &ConfigParser::posLB() const {
    return _posLB;
}

const vector<double> &ConfigParser::posUB() const {
    return _posUB;
}

double ConfigParser::rotLB() const {
    return _rotLB;
}

double ConfigParser::rotUB() const {
    return _rotUB;
}

const string &ConfigParser::snoptcLib() const {
    return _snoptclib;
}
