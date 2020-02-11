//
// Created by klevis on 12/9/17.
//

#ifndef M20IA_TRAJ_OPT_CSVWRITER_H
#define M20IA_TRAJ_OPT_CSVWRITER_H

#include <vector>
#include <string>

class CsvWriter {
public:
    CsvWriter(const std::vector<std::vector<double>> &matrix);
    void writeToFile(const std::string &file) const;
private:
    const std::vector<std::vector<double>> &_matrix;
};


#endif //M20IA_TRAJ_OPT_CSVWRITER_H
