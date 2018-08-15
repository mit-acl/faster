#ifndef UTILS_HPP
#define UTILS_HPP

float solvePolyOrder2(Eigen::Vector3f coeff)
{
  // printf("In solvePolyOrder2\n");

  std::cout << "solving\n" << coeff << std::endl;
  float a = coeff[0];
  float b = coeff[1];
  float c = coeff[2];
  float dis = b * b - 4 * a * c;
  if (dis >= 0)
  {
    float x1 = (-b - sqrt(dis)) / (2 * a);  // x1 will always be smaller than x2
    float x2 = (-b + sqrt(dis)) / (2 * a);

    if (x1 >= 0)
    {
      return x1;
    }
    if (x2 >= 0)
    {
      return x2;
    }
  }
  printf("No solution found to the equation\n");
  return std::numeric_limits<float>::max();
}

#endif