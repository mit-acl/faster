#include <iterator>
#include <iostream>
#include <fstream>
#include <decomp_basis/data_type.h>

template <int Dim>
bool read_path(std::string file_name, vec_Vecf<Dim>& path) {
  std::ifstream myfile(file_name);
  if (!myfile) {
    std::cout << "Unable to open file: " << file_name << std::endl;
    return false;
  }

  if (myfile.is_open()) {
    std::string line;
    while (getline(myfile, line)) {
      /// Seperate the line by single space
      std::istringstream buf(line);
      std::istream_iterator<std::string> beg(buf), end;

      std::vector<std::string> tokens(beg, end);

      if (tokens.size() != Dim) {
        std::cout << "Invalid format!" << std::endl;
        std::cout << line << '\n';
        return false;
      }

      /// Extract the digit value
      Vecf<Dim> pt;
      for(int i = 0; i < Dim; i++)
        pt(i) = atof(tokens[i].c_str());
      path.push_back(pt);
    }
    myfile.close();
  }

  return true;
}
