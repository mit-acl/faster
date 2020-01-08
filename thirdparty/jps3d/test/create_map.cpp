#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>

int main()
{
  // Set start & goal
  std::vector<double> start{ 0.5, 9.5, 0 };
  std::vector<double> goal{ 19.5, 0.5, 0 };
  // Create a map
  std::vector<double> origin{ 0, 0, 0 };  // set origin at (0, 0, 0)
  std::vector<int> dim{ 199, 99, 1 };     // set the number of cells in each dimension as 20, 10, 1
  double res = 0.1;                       // set resolution as 1m
  std::vector<int> data;  // occupancy data, the subscript follows: id = x + dim.x * y + dim.x * dim.y * z;
  data.resize(dim[0] * dim[1] * dim[2], 0);  // initialize as free map, free cell has 0 occupancy

  // Add the first block
  for (int x = dim[0] / 2 + 1; x < dim[0]; x++)
  {
    for (int y = dim[1] / 2 + 1; y < dim[1]; y++)
    {
      int id = x + dim[0] * y;
      data[id] = 100;
    }
  }

  // Add the second block
  for (int x = 2 / res; x < 3 / res; x++)
  {
    for (int y = 3 / res; y < 5 / res; y++)
    {
      int id = x + dim[0] * y;
      data[id] = 100;
    }
  }

  YAML::Emitter out;
  out << YAML::BeginSeq;
  // Encode start coordinate
  out << YAML::BeginMap;
  out << YAML::Key << "start" << YAML::Value << YAML::Flow << start;
  out << YAML::EndMap;
  // Encode goal coordinate
  out << YAML::BeginMap;
  out << YAML::Key << "goal" << YAML::Value << YAML::Flow << goal;
  out << YAML::EndMap;
  // Encode origin coordinate
  out << YAML::BeginMap;
  out << YAML::Key << "origin" << YAML::Value << YAML::Flow << origin;
  out << YAML::EndMap;
  // Encode dimension as number of cells
  out << YAML::BeginMap;
  out << YAML::Key << "dim" << YAML::Value << YAML::Flow << dim;
  out << YAML::EndMap;
  // Encode resolution
  out << YAML::BeginMap;
  out << YAML::Key << "resolution" << YAML::Value << res;
  out << YAML::EndMap;
  // Encode occupancy
  out << YAML::BeginMap;
  out << YAML::Key << "data" << YAML::Value << YAML::Flow << data;
  out << YAML::EndMap;

  out << YAML::EndSeq;
  std::cout << "Here is the example map:\n" << out.c_str() << std::endl;

  std::ofstream file;
  file.open("simple.yaml");
  file << out.c_str();
  file.close();

  return 0;
}
