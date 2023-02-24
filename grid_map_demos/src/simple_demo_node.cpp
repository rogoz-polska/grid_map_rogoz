#include <rclcpp/rclcpp.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <cmath>
#include <memory>
#include <utility>

#include <nlohmann/json.hpp>
#include <curl/curl.h>
#include <string.h>

// #include <Poco/Net/HTTPClientSession.h>
// #include <Poco/Net/HTTPRequest.h>
// #include <Poco/Net/HTTPResponse.h>
// #include <Poco/JSON/Parser.h>
// #include <Poco/JSON/Object.h>

using json = nlohmann::json;

void shiftCoordinates(double home_lat, double home_lon, double dx, double dy, double *shiftedLat, double *shiftedLon){
  double R = 6378137.0;
  double dlon = dx/R;
  double dlat = dy/R;
  *shiftedLon = home_lon + dlon;
  *shiftedLat = home_lat + dlat;
}

static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

double getElevation(double lat, double lon) {
    // set the API endpoint and the coordinate to retrieve elevation for
    std::string url = "https://api.opentopodata.org/v1/eudem25m?locations=";
    //double lat = 48.8584;
    //double lon = 2.2945;
    url += std::to_string(lat) + "," + std::to_string(lon);

    // use curl to make the API request and retrieve the response
    CURL *curl;
    //CURLcode res;
    std::string readBuffer;

    curl = curl_easy_init();
    if(curl) {
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &readBuffer);
        curl_easy_perform(curl);
        curl_easy_cleanup(curl);
    }
    // parse the JSON response and extract the elevation value
    json j = json::parse(readBuffer);
    double elevation = j["results"][0]["elevation"];
    //std::cout << readBuffer;
    // print the elevation value
    //std::cout << "Elevation at (" << lat << ", " << lon << ") is " << elevation << " meters." << std::endl;

    return elevation;
}

int main(int argc, char ** argv)
{
  // Initialize node and publisher.
  double sLat = 0.0, sLon = 0.0, elv=0.0;
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("grid_map_simple_demo");
  auto publisher = node->create_publisher<grid_map_msgs::msg::GridMap>(
    "grid_map", rclcpp::QoS(1).transient_local());

  // Create grid map.
  grid_map::GridMap map({"elevation"});
  map.setFrameId("map");
  map.setGeometry(grid_map::Length(1.0, 1.0), 0.1);
  RCLCPP_INFO(
    node->get_logger(),
    "Created map with size %f x %f m (%i x %i cells).",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1));

  getElevation(61.806815, 6.625353);
  // Work with grid map in a loop.
  rclcpp::Rate rate(30.0);
  rclcpp::Clock clock;
  while (rclcpp::ok()) {
    // Add data to grid map.
    rclcpp::Time time = node->now();
    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
      grid_map::Position position;
      map.getPosition(*it, position);
      shiftCoordinates(61.806815, 6.625353, (double)position.x()*100000.0, (double)position.y()*100000.0, &sLat, &sLon);
      elv = getElevation(sLat,sLon);
      std::cout<<elv<<"\n";
      map.at(
        "elevation",
        *it) = elv/1500.0;/*-0.04 + 0.2 * std::sin(3.0 * time.seconds() + 5.0 * position.y()) * position.x();*/
    }
    //getElevation(61.806815, 6.625353);
    
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
