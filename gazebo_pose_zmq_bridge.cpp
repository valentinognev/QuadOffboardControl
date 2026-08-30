#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/poses_stamped.pb.h>
#include <gazebo/transport/transport.hh>

#include <zmq.h>

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>

#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>

static volatile bool g_shutdown = false;
bool verbose_pose_bridge = false;
std::string publish_ip = "192.168.1.1";

void SignalHandler(int /*sig*/)
{
  g_shutdown = true;
}

std::string EscapeJson(const std::string &input)
{
  std::ostringstream escaped;
  for (char c : input) {
    switch (c) {
      case '"': escaped << "\\\""; break;
      case '\\': escaped << "\\\\"; break;
      case '\b': escaped << "\\b"; break;
      case '\f': escaped << "\\f"; break;
      case '\n': escaped << "\\n"; break;
      case '\r': escaped << "\\r"; break;
      case '\t': escaped << "\\t"; break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          escaped << "\\u"
                  << std::hex << std::uppercase << std::setw(4) << std::setfill('0')
                  << static_cast<int>(static_cast<unsigned char>(c));
        } else {
          escaped << c;
        }
    }
  }
  return escaped.str();
}

class ZmqBridge
{
public:
  ZmqBridge(const std::string &endpoint)
  {
    context_ = zmq_ctx_new();
    if (!context_) {
      throw std::runtime_error("Failed to create ZMQ context");
    }

    publisher_ = zmq_socket(context_, ZMQ_PUB);
    if (!publisher_) {
      zmq_ctx_term(context_);
      throw std::runtime_error("Failed to create ZMQ publisher socket");
    }

    int rc = zmq_bind(publisher_, endpoint.c_str());
    if (rc != 0) {
      zmq_close(publisher_);
      zmq_ctx_term(context_);
      throw std::runtime_error("Failed to bind ZMQ publisher socket to " + endpoint + ": " + std::string(zmq_strerror(zmq_errno())));
    }
  }

  ~ZmqBridge()
  {
    if (publisher_) {
      zmq_close(publisher_);
    }
    if (context_) {
      zmq_ctx_term(context_);
    }
  }

  void Publish(const std::string &topic, const std::string &payload)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (verbose_pose_bridge) {
      std::cout << "Transmitting ZMQ message for topic " << topic
                << " payload_size=" << payload.size() << std::endl;
    }

    int flags = ZMQ_SNDMORE;
    int rc = zmq_send(publisher_, topic.c_str(), topic.size(), flags);
    if (rc < 0) {
      std::cerr << "ZMQ publish failed for topic header: " << zmq_strerror(zmq_errno()) << std::endl;
      return;
    }

    rc = zmq_send(publisher_, payload.c_str(), payload.size(), 0);
    if (rc < 0) {
      std::cerr << "ZMQ publish failed for payload: " << zmq_strerror(zmq_errno()) << std::endl;
    }
  }

private:
  void *context_{nullptr};
  void *publisher_{nullptr};
  std::mutex mutex_;
};

void PoseCallback(const boost::shared_ptr<const gazebo::msgs::PosesStamped> &_msg,
                  ZmqBridge &bridge, const std::string &topic)
{
  if (!_msg) return;

  std::ostringstream payload;
  payload << "{";
  payload << "\"topic\":\"" << EscapeJson(topic) << "\",";

  // Gazebo Classic Pose_V doesn't include a header timestamp here

  payload << "\"poses\":[";
  for (int i = 0; i < _msg->pose_size(); ++i) {
    const auto &p = _msg->pose(i);
    if (i > 0) payload << ",";
    payload << "{";
    payload << "\"name\":\"" << EscapeJson(p.name()) << "\",";
    payload << "\"position\":{";
    payload << "\"x\":" << p.position().x() << ",";
    payload << "\"y\":" << p.position().y() << ",";
    payload << "\"z\":" << p.position().z() << "},";
    payload << "\"orientation\":{";
    payload << "\"w\":" << p.orientation().w() << ",";
    payload << "\"x\":" << p.orientation().x() << ",";
    payload << "\"y\":" << p.orientation().y() << ",";
    payload << "\"z\":" << p.orientation().z() << "}";
    payload << "}";
  }
  payload << "]}";

  bridge.Publish(topic, payload.str());
}

int main(int argc, char **argv)
{
  std::string topic = "/gazebo/default/pose/info";
  std::string endpoint = "tcp://" + publish_ip + ":5556";

  if (argc > 1) {
    topic = argv[1];
  }
  if (argc > 2) {
    endpoint = argv[2];
  }

  if (verbose_pose_bridge) {
    std::cout << "Starting Gazebo pose -> ZMQ bridge" << std::endl;
    std::cout << "  Gazebo topic: " << topic << std::endl;
    std::cout << "  ZMQ endpoint: " << endpoint << std::endl;
  }

  signal(SIGINT, SignalHandler);
  signal(SIGTERM, SignalHandler);

  gazebo::client::setup(argc, argv);

  ZmqBridge bridge(endpoint);

  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  boost::function<void(const boost::shared_ptr<const gazebo::msgs::PosesStamped> &)> callback =
    [&bridge, &topic](const boost::shared_ptr<const gazebo::msgs::PosesStamped> &_msg)
    {
      PoseCallback(_msg, bridge, topic);
    };

  auto sub = node->Subscribe(topic, callback);
  if (!sub) {
    std::cerr << "FAILED to subscribe to pose topic: " << topic << std::endl;
    return 1;
  }

  std::cout << "Pose ZMQ bridge is connected and listening. Press Ctrl+C to stop." << std::endl;

  while (!g_shutdown) {
    gazebo::common::Time::MSleep(50);
  }

  gazebo::client::shutdown();
  return 0;
}
