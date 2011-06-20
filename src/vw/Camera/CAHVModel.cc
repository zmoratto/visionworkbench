// __BEGIN_LICENSE__
// Copyright (C) 2006-2011 United States Government as represented by
// the Administrator of the National Aeronautics and Space Administration.
// All Rights Reserved.
// __END_LICENSE__


//
#include <vw/Core/Log.h>
#include <vw/Camera/CAHVModel.h>
#include <boost/algorithm/string.hpp>
#include <vw/Math/Vector.h>

namespace vw {
namespace camera {

  // FIXME -- Double check anything related to PinholeModel
  CAHVModel CAHVModel::operator= (PinholeModel const& pin_model) {

    //  Pinhole model parameters (in pixel units)
    double fH, fV, Hc, Vc;
    pin_model.intrinsic_parameters(fH, fV, Hc, Vc);

    //  Unit vectors defining camera coordinate frame
    Vector3 u,v,w;
    pin_model.coordinate_frame(u,v,w);

    //  The true rotation between world and camera coordinate
    //  frames includes the rotation R --AND-- a rotation from
    //  specifying the directions of increasing u,v,w pixels
    Matrix<double,3,3> R = pin_model.camera_pose().rotation_matrix();

    //  Now create the components of the CAHV model...
    Vector3 Hvec = R*u;
    Vector3 Vvec = R*v;

    C = pin_model.camera_center();
    A = R*w;
    H = fH*Hvec + Hc*A;
    V = fV*Vvec + Vc*A;

    return *this;
  }


  /// This constructor takes a filename and reads in a camera model
  /// from the file.  The file may contain either CAHV parameters or
  /// pinhole camera parameters.
  CAHVModel::CAHVModel(std::string const& filename) {
    if (filename.empty())
      vw_throw( IOErr() << "CAHVModel: null file name passed to constructor." );

    if (boost::ends_with(filename, ".cahv"))
      read_cahv(filename);
    else if (boost::ends_with(filename, ".pin"))
      read_pinhole(filename);
    else
      vw_throw( IOErr() << "CAHVModel: Unknown camera file suffix." );
  }

  Vector2 CAHVModel::point_to_pixel(Vector3 const& point) const {
    double dDot = dot_prod(point-C, A);
    return Vector2( dot_prod(point-C, H) / dDot,
                    dot_prod(point-C, V) / dDot );
  }

  Vector3 CAHVModel::pixel_to_vector(Vector2 const& pix) const {

    // Find vector
    Vector3 vec =
      normalize(cross_prod(V - pix.y() * A,
                           H - pix.x() * A));

    // The vector VxH should be pointing in the same directions as A,
    // if it isn't (because we have a left handed system), flip the
    // vector.
    if (dot_prod(cross_prod(V, H), A) < 0.0)
      vec *= -1.0;
    return vec;
  }

  Quat CAHVModel::camera_pose(Vector2 const& /*pix*/) const {
    // rotation from camera to world
    Quat rot1( normalize(cross_prod(Vector3(0,0,1),A)),
               acos(dot_prod(Vector3(0,0,1),normalize(A))) );
    Vector3 x_p_axis = rot1.rotate(Vector3(1,0,0));
    // We don't use H directly as it is tilted forward to set the
    // origin of the pixels.
    Vector3 x_obj = normalize(cross_prod(V,A));
    Quat rot2( A, -acos(dot_prod(x_p_axis,x_obj)) );
    std::cout << "Degree: " << -acos(dot_prod(x_p_axis,x_obj)) << "\n";
    std::cout << "A:      " << A << "\n";
    std::cout << "Cross of rotate: " << cross_prod(x_p_axis,x_obj) << "\n";
    return rot2 * rot1;
  }

  // --------------------------------------------------
  //                 Private Methods
  // --------------------------------------------------
  void CAHVModel::read_cahv(std::string const& filename) {

    try {
      std::ifstream input(filename.c_str(), std::ifstream::in);
      input.exceptions(std::ifstream::failbit | std::ifstream::badbit);

      vw_out(InfoMessage, "camera") << "Reading CAHV file: "
                                    << filename << ".\n";

      char r1, r2;

      while (true) {
        input.ignore(1024, 'C');
        input >> r1;
        if (r1 == '=')
          break;
      }
      input >> C[0] >> C[1] >> C[2];

      input >> r1 >> r2;
      if (r1 != 'A' || r2 != '=')
        vw_throw( IOErr() << "CAHVModel: Could not read A vector\n" );
      input >> A(0) >> A(1) >> A(2);

      input >> r1 >> r2;
      if (r1 != 'H' || r2 != '=')
        vw_throw( IOErr() << "CAHVModel: Could not read H vector\n" );
      input >> H(0) >> H(1) >> H(2);

      input >> r1 >> r2;
      if (r1 != 'V' || r2 != '=')
        vw_throw( IOErr() << "CAHVModel: Could not read V vector\n" );
      input >> V(0) >> V(1) >> V(2);

    } catch ( const std::ifstream::failure& e ) {
      vw_throw( IOErr() << "CAHVModel: Could not read file: " << filename << " (" << e.what() << ")" );
    }
  }

  void CAHVModel::read_pinhole(std::string const& filename) {
    FILE *camFP = fopen(filename.c_str(), "r");

    if (camFP == 0)
      vw_throw( IOErr() << "CAHVModel::read_pinhole: Could not open file\n" );

    char line[2048];
    double f, fH, fV, Hc, Vc;
    Vector2 pixelSize;
    Vector3 Hvec, Vvec;

    // Read intrinsic parameters
    fgets(line, sizeof(line), camFP);
    if (sscanf(line,"f = %lf", &f) != 1) {
      vw_throw( IOErr() << "CAHVModel::read_pinhole: Could not read focal length\n" );
      fclose(camFP);
    }

    fgets(line, sizeof(line), camFP);
    if (sscanf(line,"SP = %lf %lf", &pixelSize.x(), &pixelSize.y()) != 2) {
      vw_throw( IOErr() << "CAHVModel::read_pinhole: Could not read pixel size\n" );
      fclose(camFP);
    }

    fgets(line, sizeof(line), camFP);
    if (sscanf(line,"IC = %lf %lf", &Hc, &Vc) != 2) {
      vw_throw( IOErr() << "CAHVModel::ReadPinhole: Could not read image center pos\n" );
      fclose(camFP);
    }

    // Read extrinsic parameters
    fgets(line, sizeof(line), camFP);
    if (sscanf(line,"C = %lf %lf %lf", &C(0), &C(1), &C(2)) != 3) {
      vw_throw( IOErr() << "CAHVModel::read_pinhole: Could not read C vector\n" );
      fclose(camFP);
    }

    fgets(line, sizeof(line), camFP);
    if (sscanf(line,"A = %lf %lf %lf", &A(0), &A(1), &A(2)) != 3) {
      vw_throw( IOErr() << "CAHVModel::read_pinhole: Could not read A vector\n" );
      fclose(camFP);
    }

    fgets(line, sizeof(line), camFP);
    if (sscanf(line,"Hv = %lf %lf %lf", &Hvec(0), &Hvec(1), &Hvec(2)) != 3) {
      vw_throw( IOErr() << "CAHVModel::read_pinhole: Could not read Hvec\n" );
      fclose(camFP);
    }

    fgets(line, sizeof(line), camFP);
    if (sscanf(line,"Vv = %lf %lf %lf", &Vvec(0), &Vvec(1), &Vvec(2)) != 3) {
      vw_throw( IOErr() << "CAHVModel::read_pinhole: Could not read Vvec\n" );
      fclose(camFP);
    }

    // In the future, we should also read in a view matrix -- LJE
    //     double dummy
    //     if (sscanf(line, "VM = %lf %lf %lf %f %lf %lf %lf %f "
    //        "%lf %lf %lf %f %lf %lf %lf %f ",
    //        &Hvec(0), &Hvec(1), &Hvec(2), &dummy,
    //        &Vvec(0), &Vvec(1), &Vvec(2), &dummy,
    //        &A(0), &A(1), &A(2), &dummy,
    //        &C(0), &C(1), &C(2), &dummy) != 16)
    //     {
    //       vw_throw( IOErr()
    //  << "CAHVModel::ReadPinhole: Could not read view matrix\n" );
    //       fclose(camFP);
    //     }

    fH = f/pixelSize.x();
    fV = f/pixelSize.y();

    H = fH*Hvec + Hc*A;
    V = fV*Vvec + Vc*A;

    fclose(camFP);
  }

  void epipolar(CAHVModel const src_camera0, CAHVModel const src_camera1,
                CAHVModel &dst_camera0, CAHVModel &dst_camera1) {

    // Compute a common image center and scale for the two models
    double hc = dot_prod(src_camera0.H, src_camera0.A) / 2.0 +
      dot_prod(src_camera1.H, src_camera1.A) / 2.0;
    double vc = dot_prod(src_camera0.V, src_camera0.A) / 2.0 +
      dot_prod(src_camera1.V, src_camera1.A) / 2.0;

    double hs = norm_2(cross_prod(src_camera0.A, src_camera0.H))/2.0 +
      norm_2(cross_prod(src_camera1.A, src_camera1.H))/2.0;

    double vs = norm_2(cross_prod(src_camera0.A, src_camera0.V))/2.0 +
      norm_2(cross_prod(src_camera1.A, src_camera1.V))/2.0;

    // Use common center and scale to construct common A, H, V
    Vector3 app  = src_camera0.A + src_camera1.A;

    // Note the directionality of 1 to 0, for consistency later
    Vector3 f = cross_prod(cross_prod(app, src_camera1.C - src_camera0.C), app);

    Vector3 hp;
    if (dot_prod(f, src_camera0.H) > 0)
      hp = f * hs / (norm_2(f));
    else
      hp = -f * hs / (norm_2(f));

    app *= 0.5;
    Vector3 g = hp * dot_prod(app,hp) / (hs * hs);
    Vector3 a = normalize(app - g);
    Vector3 vp = cross_prod(a, hp) * vs / hs;

    dst_camera0.C = src_camera0.C;
    dst_camera1.C = src_camera1.C;

    dst_camera0.A = dst_camera1.A = a;
    dst_camera0.H = dst_camera1.H = hp + hc * a;
    dst_camera0.V = dst_camera1.V = vp + vc * a;
  }

  void CAHVModel::write(std::string const& filename ) {
    try {
      std::ofstream output(filename.c_str(), std::ofstream::out);
      output.exceptions(std::ofstream::failbit | std::ofstream::badbit);
      output.precision(20);

      vw_out(InfoMessage, "camera") << "Writing CAHV file: " << filename << "\n";

      output << "C = " << C[0] << " " << C[1] << " " << C[2] << "\n"
             << "A = " << A[0] << " " << A[1] << " " << A[2] << "\n"
             << "H = " << H[0] << " " << H[1] << " " << H[2] << "\n"
             << "V = " << V[0] << " " << V[1] << " " << V[2] << "\n";
    } catch ( const std::ofstream::failure& e ) {
      vw_throw( IOErr() << "CAHVModel: Could not write file: " << filename << "(" << e.what() << ")" );
    }
  }

}} // namespace vw::camera
