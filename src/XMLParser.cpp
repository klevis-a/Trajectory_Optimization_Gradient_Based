//
// Created by klevis on 4/21/18.
//

#include <XMLParser.h>
#include <stdexcept>
#include <iostream>
#include <iterator>
#include <sstream>

using namespace std;
using namespace tinyxml2;

XMLParser::XMLParser(const char *fileName) {
    XMLError xmlResult = _doc.LoadFile(fileName);
    XMLCheckResult(xmlResult);

    //read the root node
    rootNode = _doc.FirstChildElement("Parameters");
    if(rootNode == nullptr) throw invalid_argument("Invalid Root Node");
}

void XMLParser::XMLCheckResult(XMLError result)
{
    if (result != XML_SUCCESS) {
        cout <<  "XML Parsing Error: " << result;
        throw invalid_argument("XML Parsing Error");
    }
}

unsigned int XMLParser::parseUnsignedInt(const char *paramName) const
{
    unsigned int ival;
    XMLElement *element = rootNode->FirstChildElement(paramName);
    if(element == nullptr) throw invalid_argument("Could not read: " + string(paramName));
    XMLError xmlResult = element->QueryUnsignedText(&ival);
    XMLCheckResult(xmlResult);
    return ival;
}

double XMLParser::parseDouble(const char *paramName) const
{
    double ival;
    XMLElement *element = rootNode->FirstChildElement(paramName);
    if(element == nullptr) throw invalid_argument("Could not read: " + string(paramName));
    XMLError xmlResult = element->QueryDoubleText(&ival);
    XMLCheckResult(xmlResult);
    return ival;
}

bool XMLParser::parseBool(const char *paramName) const
{
    bool ival;
    XMLElement *element = rootNode->FirstChildElement(paramName);
    if(element == nullptr) throw invalid_argument("Could not read: " + string(paramName));
    XMLError xmlResult = element->QueryBoolText(&ival);
    XMLCheckResult(xmlResult);
    return ival;
}

const char* XMLParser::parseText(const char *paramName) const {
    XMLElement *element = rootNode->FirstChildElement(paramName);
    if(element == nullptr) throw invalid_argument("Could not read: " + string(paramName));
    return element->GetText();
}

vector<double> XMLParser::stringToVector(const string &line) {
    istringstream iss(line);

    return vector<double>{
            istream_iterator<double>(iss),
            istream_iterator<double>()
    };
}
