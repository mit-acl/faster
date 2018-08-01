#include <iterator>
#include <iostream>
#include <fstream>
#include <decomp_util/data_type.h>

bool read_path(std::string file_name, vec_Vec3f& path) {
  std::ifstream myfile(file_name);
  if (!myfile) {
    std::cout << "Unable to open file: " << file_name << std::endl;
    return false;
  }

  if (myfile.is_open())
  {
    std::string line;
    while ( getline (myfile,line) )
    {
      ///Seperate the line by single space
      std::istringstream buf(line);
      std::istream_iterator<std::string> beg(buf), end;

      std::vector<std::string> tokens(beg, end);

      if(tokens.size() != 3) {
	std::cout << "Invalid format!" << std::endl;
	std::cout << line << '\n';
	return false;
      }

      ///Extract the digit value
      path.push_back(Vec3f(atof(tokens[0].c_str()), 
	    atof(tokens[1].c_str()), atof(tokens[2].c_str())));
    }
    myfile.close();
  }

  return true;
}
