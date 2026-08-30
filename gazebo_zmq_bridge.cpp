#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/image.pb.h>
#include <gazebo/msgs/image_stamped.pb.h>
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

/*was static volatile bool g_shutdown = false; but got the error
./gazebo_zmq_bridge.cpp: line 18: static: command not found
*/
static volatile bool g_shutdown = false;
bool verbose = false;
int frame_count = 0;
int which_frame = 6; // Only publish every Nth frame, 0 or 1 = publish all frames

//idk what sig is supposed to be
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

std::string Base64Encode(const std::string &input)
{
  static const char kEncodeLookup[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string output;
  output.reserve(((input.size() + 2) / 3) * 4);

  size_t pos = 0;
  while (pos < input.size()) {
    size_t chunkStart = pos;
    unsigned int octet_a = static_cast<unsigned char>(input[pos++]);
    unsigned int octet_b = pos < input.size() ? static_cast<unsigned char>(input[pos++]) : 0;
    unsigned int octet_c = pos < input.size() ? static_cast<unsigned char>(input[pos++]) : 0;
    unsigned int triple = (octet_a << 16) + (octet_b << 8) + octet_c;
    size_t bytesRemaining = input.size() - chunkStart;

    output.push_back(kEncodeLookup[(triple >> 18) & 0x3F]);
    output.push_back(kEncodeLookup[(triple >> 12) & 0x3F]);
    output.push_back(bytesRemaining > 1 ? kEncodeLookup[(triple >> 6) & 0x3F] : '=');
    output.push_back(bytesRemaining > 2 ? kEncodeLookup[triple & 0x3F] : '=');
  }

  return output;
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

    if (verbose) {
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

void PublishImage(const gazebo::msgs::Image &_image,
                  ZmqBridge &bridge, const std::string &topic)
{
  if (verbose) {
    std::cout << "Received Gazebo image message for topic " << topic
              << " (width=" << _image.width() << ", height=" << _image.height()
              << ", data_size=" << _image.data().size() << ")" << std::endl;
  }

  std::string image_base64 = Base64Encode(_image.data());

  std::ostringstream payload;
  payload << "{";
  payload << "\"topic\":\"" << EscapeJson(topic) << "\",";
  payload << "\"width\":" << _image.width() << ",";
  payload << "\"height\":" << _image.height() << ",";
  payload << "\"pixel_format\":" << _image.pixel_format() << ",";
  payload << "\"data_size\":" << _image.data().size() << ",";
  payload << "\"step\":" << _image.step() << ",";
  payload << "\"image_data\":\"" << EscapeJson(image_base64) << "\"";
  payload << "}";

  bridge.Publish(topic, payload.str());
  /* std::cout << "Published image message for topic " << topic
            << " (" << _image.width() << "x" << _image.height()
            << ", encoded_size=" << image_base64.size() << ")" << std::endl;
*/
            }

void ImageCallback(const boost::shared_ptr<const gazebo::msgs::Image> &_msg,
                   ZmqBridge &bridge, const std::string &topic)
{
  if (!_msg) return;
  frame_count++;
  if (which_frame > 0 && frame_count % which_frame != 0) {
    return;
  }
  PublishImage(*_msg, bridge, topic);
}

void ImageStampedCallback(const boost::shared_ptr<const gazebo::msgs::ImageStamped> &_msg,
                           ZmqBridge &bridge, const std::string &topic)
{
  if (!_msg || !_msg->has_image()) return;
  frame_count++;
  if (which_frame > 0 && frame_count % which_frame != 0) {
    return;
  }
  PublishImage(_msg->image(), bridge, topic);
}


int main(int argc, char **argv)
{
	//std::string topic = "/iris/front_camera/image";
  std::string topic = "/gazebo/default/iris/base_link/front_camera/image";
  std::string endpoint = "tcp://127.0.0.1:5555";

  if (argc > 1) {
    topic = argv[1];
  }
  if (argc > 2) {
    endpoint = argv[2];
  }

  if (verbose) {
    std::cout << "Starting Gazebo Classic -> ZMQ bridge" << std::endl;
    std::cout << "  Gazebo topic: " << topic << std::endl;
    std::cout << "  ZMQ endpoint: " << endpoint << std::endl;
  }

  signal(SIGINT, SignalHandler);
  signal(SIGTERM, SignalHandler);

  gazebo::client::setup(argc, argv);

  ZmqBridge bridge(endpoint);

  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  bool subscribed = false;


  boost::function<void(const boost::shared_ptr<const gazebo::msgs::ImageStamped> &)> stampedCallback =
    [&bridge, &topic](const boost::shared_ptr<const gazebo::msgs::ImageStamped> &_msg)
    {
      ImageStampedCallback(_msg, bridge, topic);
    };

  auto sub = node->Subscribe(topic, stampedCallback);
  if (sub) {
    subscribed = true;
    std::cout << "Successfully subscribed to ImageStamped topic: " << topic << std::endl;
  }
  else {
    std::cerr << "FAILED to subscribe to ImageStamped topic: " << topic << std::endl;
  }


  if (!subscribed) {
    boost::function<void(const boost::shared_ptr<const gazebo::msgs::Image> &)> callback =
      [&bridge, &topic](const boost::shared_ptr<const gazebo::msgs::Image> &_msg)
      {
        ImageCallback(_msg, bridge, topic);
      };

    auto sub = node->Subscribe(topic, callback);
    if (sub) {
      subscribed = true;
      std::cout << "Successfully subscribed to Image topic: " << topic << std::endl;
    }
  }

  if (!subscribed) {
    std::cerr << "FAILED to subscribe to topic: " << topic << std::endl;
    return 1;
  }

  std::cout << "Bridge is connected and listening. Press Ctrl+C to stop." << std::endl;

  while (!g_shutdown) {
    gazebo::common::Time::MSleep(50);
  }

  gazebo::client::shutdown();
  return 0;
}
