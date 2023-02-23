#include <rclcpp/rclcpp.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <cmath>
#include <memory>
#include <utility>

#include <jsoncpp/json/json.h>
#include <curl/curl.h>
#include <string.h>

// #include <Poco/Net/HTTPClientSession.h>
// #include <Poco/Net/HTTPRequest.h>
// #include <Poco/Net/HTTPResponse.h>
// #include <Poco/JSON/Parser.h>
// #include <Poco/JSON/Object.h>

void shiftedCoordinates(double home_lat, double home_lon, double dx, double dy, double *shiftedLat, double *shiftedLon){
  double R = 6378137;
  double dlon = dx/R;
  double dlat = dy/R;
  *shiftedLon = home_lon + dlon;
  *shiftedLat = home_lat + dlat;
}

std::string reqURL(double lat, double lon){
  
  std::string request_url = "https://api.opentopodata.org/v1/eudem25m?locations=" + std::to_string(lat) + "," + std::to_string(lon);
  // std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>requested url: " << request_url << std::endl;
  return request_url;
}

static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}


double getElevation(double lat, double lon) {
  auto x = lon + lat;
  reqURL(lat, lon);
  
  CURL *curl;
  //CURLcode res;
  std::string readBuffer;
  curl = curl_easy_init();
      if(curl) {
            std::cout<< "curl initiated" << std::endl;
            curl_easy_setopt(curl, CURLOPT_URL, "https://api.opentopodata.org/v1/eudem25m?locations=32.330000,43.200000");
            // curl_easy_setopt(curl, CURLOPT_URL, reqURL(lat, lon));
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
            curl_easy_perform(curl);
            curl_easy_cleanup(curl);
      }
  std::cout<< readBuffer << "." << std::endl;
  return x;
}

int main(int argc, char ** argv)
{
  // Initialize node and publisher.
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("grid_map_simple_demo");
  auto publisher = node->create_publisher<grid_map_msgs::msg::GridMap>(
    "grid_map", rclcpp::QoS(1).transient_local());

  // Create grid map.
  grid_map::GridMap map({"elevation"});
  map.setFrameId("map");
  map.setGeometry(grid_map::Length(3, 3), 0.03);
  RCLCPP_INFO(
    node->get_logger(),
    "Created map with size %f x %f m (%i x %i cells).",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1));

  getElevation(32.33, 43.2);
  // Work with grid map in a loop.
  rclcpp::Rate rate(30.0);
  rclcpp::Clock clock;
  while (rclcpp::ok()) {
    // Add data to grid map.
    rclcpp::Time time = node->now();
    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
      grid_map::Position position;
      map.getPosition(*it, position);
      map.at(
        "elevation",
        *it) = -0.04 + 0.2 * std::sin(3.0 * time.seconds() + 5.0 * position.y()) * position.x();
    }
    getElevation(32.33, 43.2);

    // Publish grid map.
    map.setTimestamp(time.nanoseconds());
    std::unique_ptr<grid_map_msgs::msg::GridMap> message;
    message = grid_map::GridMapRosConverter::toMessage(map);
    publisher->publish(std::move(message));
    RCLCPP_INFO_THROTTLE(node->get_logger(), clock, 1000, "Grid map published.");

    // Wait for next cycle.
    rate.sleep();
  }

  return 0;
}
