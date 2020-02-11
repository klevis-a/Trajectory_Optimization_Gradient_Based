//
// Created by klevis on 4/21/18.
//

#ifndef M20IA_TRAJ_OPT_XMLPARSER_H
#define M20IA_TRAJ_OPT_XMLPARSER_H

#include <tinyxml2.h>
#include <vector>
#include <string>

class XMLParser {
public:
    XMLParser(const char* fileName);
    unsigned int parseUnsignedInt(const char *paramName) const;
    double parseDouble(const char *paramName) const;
    bool parseBool(const char *paramName) const;
    const char* parseText(const char *paramName) const;
    static void XMLCheckResult(tinyxml2::XMLError result);
    static std::vector<double> stringToVector(const std::string &line);

protected:
    //root node of document
    tinyxml2::XMLDocument _doc;
    tinyxml2::XMLElement * rootNode;

};

#endif //M20IA_TRAJ_OPT_XMLPARSER_H
