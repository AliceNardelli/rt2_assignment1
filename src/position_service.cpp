#include <inttypes.h>
#include <memory>
#include <functional>
#include "rt2_assignment1/srv/random_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"


using std::placeholders::_1;  // they will be replaced by the actual message when callback is called
using std::placeholders::_2;
using std::placeholders::_3;

namespace rt2_assignment1{

/*! Position_Service Class:
* The implementation of the FSM Node through a class stucture
* allow to execute it as a component and communicates
* through ros1_bridge package with ros nodes
*/
class Position_Service : public rclcpp::Node
{
public:
	/** Initialisation of the  random Position service. 
	* It takes as argument the callback and the three message fields.
	* Trough bind() sys call the callback is executed as soon as
	* the client makes a request of /random_server.
	*/
  Position_Service(const rclcpp::NodeOptions & options)
  : Node("position_server", options)
  {
    service_ = this->create_service<rt2_assignment1::srv::RandomPosition>(
      "/position_server", std::bind(&Position_Service::handle_service, this, _1, _2, _3)); /**< callback */
  }

private:

  /** Callback definition.
	*/
	
 /**
 * Documentation for the handle_service function.
 *
 * It takes as service request some maximum and minimum values for x and y coordinates. 
 * It returns a set of x, y, theta random values as response.
 *
 * @param request_header
 * @param request it retrieves an interval of values for x and y
 * @param response it defines a random set of x,y and theta
 */
  void handle_service(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<rt2_assignment1::srv::RandomPosition::Request> request,
  const std::shared_ptr<rt2_assignment1::srv::RandomPosition::Response> response)
  {
  (void)request_header;
    std::cout<<"received request"<<std::endl;
    response->x = request->x_min + (rand() / ( RAND_MAX / (request->x_max-request->x_min) ) );
    response->y = request->y_min + (rand() / ( RAND_MAX / (request->y_max-request->y_min) ) );
    response->theta =  -3.14+ (rand() / ( RAND_MAX / (6.28)));
    std::cout<<"sent random"<<std::endl;
  }
  rclcpp::Service<rt2_assignment1::srv::RandomPosition>::SharedPtr service_; 
  
  
};
}
RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::Position_Service)
