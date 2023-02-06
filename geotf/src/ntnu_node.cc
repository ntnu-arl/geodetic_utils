#include <geotf/geodetic_converter.h>
#include <iomanip> 
#include <ros/ros.h>
int main(int argc, char* argv[]) {
  ros::init(argc, argv, "bla");
  ros::NodeHandle nh;

  geotf::GeodeticConverter converter;
  converter.initFromRosParam();

  // Wait for TF to setup
  sleep(1.0);

  // A few vectors for conversions
  Eigen::Vector3d GM17C_borehole_utm, GM17C_borehole_gps,
      GM17C_borehole_euref89, GM17C_borehole_enu;

  // Initialize GM17C_borehole based on UTM coordinates
  GM17C_borehole_utm << 7032663.7528, 570092.5139, 88.152;

  // Output GM17C_borehole in GPS frame
  if (converter.canConvert("UTM", "GPS")) {
    converter.convert("UTM", GM17C_borehole_utm,
                      "GPS", &GM17C_borehole_gps);

    ROS_INFO_STREAM("GM17C_borehole WGS84 = " << std::setprecision(16)
                                                << GM17C_borehole_gps);

  } else {
    ROS_WARN_STREAM("Frames not loaded.");
  }

  // Output GM17C_borehole in ETRS89/EUREF89 frame
  if (converter.canConvert("UTM", "ETRS89")) {
    converter.convert("UTM", GM17C_borehole_utm,
                      "ETRS89", &GM17C_borehole_euref89);

    ROS_INFO_STREAM("GM17C_borehole ETRS89/EUREF89 = " << std::setprecision(16)
                                                       << GM17C_borehole_euref89);

  } else {
    ROS_WARN_STREAM("Frames not loaded.");
  }

  // Output GM17C_borehole in ENU frame based on NTNU electronics department
  if (converter.canConvert("UTM", "ENU_NTNU")) {
    converter.convert("UTM", GM17C_borehole_utm,
                      "ENU_NTNU", &GM17C_borehole_enu);

    ROS_INFO_STREAM(" GM17C_borehole in ENU Frame based on TNU electronics department = "
                        << std::setprecision(16)
                        << GM17C_borehole_enu);

  } else {
    ROS_WARN_STREAM("Frames not loaded.");
  }


  ROS_INFO_STREAM("Please start rviz for visualization and press enter.");
  std::cin.get();

  // Example of directly converting TF locations into geo locations

  // Here we convert location 0/0/0 in tf frame "body" to UTM conversions
  // Note that we do not have to specify explictely how this is converted,
  // as we already configured the equivalence of Geoframe ENU_LEE and
  // tf frame enu in the launch file.


  Eigen::Affine3d body_coords(Eigen::Affine3d::Identity());
  Eigen::Affine3d utm_body_coords(Eigen::Affine3d::Identity());
  converter.convertFromTf("body",
                          body_coords,
                          "UTM",
                          &utm_body_coords);
  ROS_INFO_STREAM("UTM coordinates of body origin:");
  std::cout << utm_body_coords.translation() << std::endl;


  // Example of Publishing Geolocations as TF frames for visualization.

  // Publish TF Frame CornerUTM based on UTM coordinates
  Eigen::Affine3d utm_building_point(Eigen::Affine3d::Identity());
  utm_building_point.translation().x() = 465727;
  utm_building_point.translation().y() = 5247291;
  utm_building_point.translation().z() = 489.619;
  std::cout << converter.publishAsTf("UTM", utm_building_point, "CornerUTM") << std::endl;

  // Publish TF Frame P5 nail based on WGS84 coordinates
  Eigen::Affine3d P5_nail(Eigen::Affine3d::Identity());
  P5_nail.translation().x() = 63.415018;
  P5_nail.translation().y() = 10.407384;
  P5_nail.translation().z() = 79.608;
  converter.publishAsTf("GPS", P5_nail, "P5_nail");

  // Publish TF Frame GM17E bolt in top of wall based on WGS84 coordinates
  Eigen::Affine3d GM17E(Eigen::Affine3d::Identity());
  GM17E.translation().x() = 63.417629;
  GM17E.translation().y() = 10.401332;
  GM17E.translation().z() = 87.002;
  converter.publishAsTf("GPS", GM17E, "GM17E");

  // Publish TF Frame GM17E bolt in top of wall based on ENU coordinates
  //Eigen::Affine3d ENU_building_point(Eigen::Affine3d::Identity());
  //ENU_building_point.translation().x() = 14.58;
  //ENU_building_point.translation().y() = 6.64;
  //ENU_building_point.translation().z() = 0.0;
  //converter.publishAsTf("ENU_LEE", ENU_building_point, "CornerENU");

  // Publish TF Frame P1 to P4 based on EUREF89 coordinates
  Eigen::Affine3d P1(Eigen::Affine3d::Identity());
  P1.translation().x() = 7032516.8044;
  P1.translation().y() = 570406.8605;
  P1.translation().z() = 80.3184;
  converter.publishAsTf("ETRS89", P1, "P1");

  Eigen::Affine3d P2(Eigen::Affine3d::Identity());
  P2.translation().x() = 7032503.4822;
  P2.translation().y() = 570413.3669;
  P2.translation().z() = 80.1282;
  converter.publishAsTf("ETRS89", P2, "P2");

  Eigen::Affine3d P3(Eigen::Affine3d::Identity());
  P3.translation().x() = 7032512.5002;
  P3.translation().y() = 570431.9676;
  P3.translation().z() = 80.4122;
  converter.publishAsTf("ETRS89", P3, "P3");

  Eigen::Affine3d P4(Eigen::Affine3d::Identity());
  P4.translation().x() = 7032526.0900;
  P4.translation().y() = 570425.3519;
  P4.translation().z() = 80.6571;
  converter.publishAsTf("ETRS89", P4, "P4");

  ros::spin();

  return 0;
}