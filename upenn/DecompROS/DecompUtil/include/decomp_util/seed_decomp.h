/**
 * @file seed_decomp.h
 * @brief SeedDecomp Class
 */
#ifndef SEED_DECOMP_H
#define SEED_DECOMP_H

#include <decomp_util/decomp_base.h>

/**
 * @brief Seed Decomp Class
 *
 * Dilate around the given point
 */
template <int Dim>
class SeedDecomp : public DecompBase<Dim>
{
public:
  /// Simple constructor
  SeedDecomp(){};
  /**
   * @brief Basic constructor
   * @param p1 One end of the line seg
   * @param p2 The other end of the line seg
   */
  SeedDecomp(const Vecf<Dim> &p) : p_(p)
  {
  }
  /**
   * @brief Inflate the seed with a sphere
   * @param radius Robot radius
   */
  void dilate(decimal_t radius)
  {
    this->ellipsoid_ = Ellipsoid<Dim>(radius * Matf<Dim, Dim>::Identity(), p_);
    this->find_polyhedron();
    // std::cout << "Number of obstacles" << this->obs_.size() << std::endl;
    // std::cout << "Before adding the local box, matrix A=" << std::endl;
    /*    for (unsigned int j = 0; j < (this->polyhedron_.vs_).size(); j++)  // For all the hyperplanes in one polyhedro
        {
          std::cout << (this->polyhedron_.vs_)[j].n_.transpose() << std::endl;
        }
    */
    add_local_bbox(this->polyhedron_);
  }

  /// Get the center
  Vecf<Dim> get_seed() const
  {
    return p_;
  }

  void set_seed(Vecf<Dim> seed)
  {
    p_ = seed;
  }

  void shrink_polyhedron(double shrink_poly_distance)  // Jesus added this function
  {
    LinearConstraint3D cs1(p_, this->polyhedron_.hyperplanes());
    /*    printf("i: %zu\n", i);
        std::cout << "A: " << cs.A() << std::endl;
        std::cout << "b: " << cs.b() << std::endl;
        std::cout << "point: " << path[i].transpose();*/
    if (cs1.inside(p_))
    {
      // std::cout << "*****Before shrinking: The seed point is inside!" << std::endl;
    }
    else
    {
      ROS_ERROR("*****Before shrinking: The seed point is outside!*****");
    }

    // vec_E<Hyperplane<Dim>> *hyperplanes_ptr = &(this->polyhedron_.vs_);
    for (unsigned int j = 0; j < (this->polyhedron_.vs_).size(); j++)  // For all the hyperplanes in one polyhedro
    {
      // First compute distance between point and plane
      Vecf<Dim> n = ((this->polyhedron_.vs_)[j].n_).normalized();  // normal to the plane
      Vecf<Dim> m = (this->polyhedron_.vs_)[j].p_;                 // m is a point on the plane

      /*      std::cout << "n= " << n << std::endl;
            std::cout << "Point_in_plane= " << n << std::endl;*/

      double dot_product = n.dot(p_ - m);
      double distance = fabs(dot_product);

      // std::cout << "*******************distance antes= " << distance;

      double shrink_distance = std::min(shrink_poly_distance, distance - 0.001);  // 0.001 to avoid numerical issues

      // std::cout << ", going to shrink" << shrink_distance << std::endl;

      int sign = std::copysign(1.0, dot_product);

      (this->polyhedron_.vs_)[j].p_ =
          (this->polyhedron_.vs_)[j].p_ + shrink_distance * sign * (this->polyhedron_.vs_)[j].n_;
    }

    /*    for (unsigned int j = 0; j < (this->polyhedron_.vs_).size(); j++)  // For all the hyperplanes in one polyhedro
        {
          // First compute distance between point and plane
          Vecf<Dim> n = (this->polyhedron_.vs_)[j].n_.normalized();  // normal to the plane
          Vecf<Dim> m = (this->polyhedron_.vs_)[j].p_;               // m is a point on the plane

          double distance = fabs(n.dot(p_ - m));

          std::cout << "*******************distance despues= " << distance << std::endl;
        }*/

    LinearConstraint3D cs(p_, this->polyhedron_.hyperplanes());
    /*    printf("i: %zu\n", i);
        std::cout << "A: " << cs.A() << std::endl;
        std::cout << "b: " << cs.b() << std::endl;
        std::cout << "point: " << path[i].transpose();*/
    if (cs.inside(p_))
    {
      // std::cout << "*****After shrinking: The seed point is inside!" << std::endl;
    }
    else
    {
      ROS_ERROR("*****After shrinking: The seed point is outside!*****");
    }
  }

protected:
  /// Add the bounding box
  void add_local_bbox(Polyhedron<Dim> &Vs)
  {
    if (this->local_bbox_.norm() == 0)
      return;

    //**** virtual walls x-y-z
    Vecf<Dim> dir = Vecf<Dim>::UnitX();
    Vecf<Dim> dir_h = Vecf<Dim>::UnitY();

    Vecf<Dim> pp1 = p_ + dir_h * this->local_bbox_(1);
    Vecf<Dim> pp2 = p_ - dir_h * this->local_bbox_(1);
    Vs.add(Hyperplane<Dim>(pp1, dir_h));
    Vs.add(Hyperplane<Dim>(pp2, -dir_h));

    // along y
    Vecf<Dim> pp3 = p_ + dir * this->local_bbox_(0);
    Vecf<Dim> pp4 = p_ - dir * this->local_bbox_(0);
    Vs.add(Hyperplane<Dim>(pp3, dir));
    Vs.add(Hyperplane<Dim>(pp4, -dir));

    // along z
    if (Dim > 2)
    {
      Vecf<Dim> dir_v = Vecf<Dim>::UnitZ();
      Vecf<Dim> pp5 = p_ + dir_v * this->local_bbox_(2);
      Vecf<Dim> pp6 = p_ - dir_v * this->local_bbox_(2);
      Vs.add(Hyperplane<Dim>(pp5, dir_v));
      Vs.add(Hyperplane<Dim>(pp6, -dir_v));
    }
  }

  /// Seed location
  Vecf<Dim> p_;
};

typedef SeedDecomp<2> SeedDecomp2D;

typedef SeedDecomp<3> SeedDecomp3D;

#endif
