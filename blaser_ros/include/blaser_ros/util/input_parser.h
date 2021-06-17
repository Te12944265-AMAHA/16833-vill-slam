//
// Created by dcheng on 6/17/21.
//

#ifndef SRC_INPUT_PARSER_H
#define SRC_INPUT_PARSER_H
#include <vector>

class InputParser{
public:
  InputParser (int &argc, char **argv);
  const std::string& getCmdOption(const std::string &option) const;
  bool cmdOptionExists(const std::string &option) const;
private:
  std::vector <std::string> tokens;
};

#endif //SRC_INPUT_PARSER_H
