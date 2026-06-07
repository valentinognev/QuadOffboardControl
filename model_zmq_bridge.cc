#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/image.pb.h>
#include <gazebo/transport/transport.hh>
#include <zmq.hpp>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <algorithm>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

static const std::string CAMERA_TOPIC = "/gazebo/default/iris/base_link/front_camera/image";
static const std::string ZMQ_ENDPOINT = "tcp://*:5555";

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
    size_t bytes_in = std::min(input.size() - chunkStart, static_cast<size_t>(3));

    output.push_back(kEncodeLookup[(triple >> 18) & 0x3F]);
    output.push_back(kEncodeLookup[(triple >> 12) & 0x3F]);
    output.push_back(bytes_in > 1 ? kEncodeLookup[(triple >> 6) & 0x3F] : '=');
    output.push_back(bytes_in > 2 ? kEncodeLookup[triple & 0x3F] : '=');
  }

  return output;
}

int main(int argc, char **argv)
{
  std::string topic = CAMERA_TOPIC;
  std::string endpoint = ZMQ_ENDPOINT;

  if (argc > 1) {
    topic = argv[1];
  }
  if (argc > 2) {
    endpoint = argv[2];
  }

  std::cout << "Starting Gazebo image -> ZMQ bridge" << std::endl;
  std::cout << "  Gazebo topic: " << topic << std::endl;
  std::cout << "  ZMQ endpoint: " << endpoint << std::endl;

  gazebo::client::setup(argc, argv);
  zmq::context_t context(1);
  zmq::socket_t publisher(context, ZMQ_PUB);
  publisher.bind(endpoint);

  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  boost::function<void(const boost::shared_ptr<const gazebo::msgs::Image> &)> callback =
    [&](const boost::shared_ptr<const gazebo::msgs::Image> &_msg) {
      if (!_msg) {
        return;
      }

      std::string image_base64 = Base64Encode(_msg->data());
      std::ostringstream payload;
      payload << "{";
      payload << "\"topic\":\"" << EscapeJson(topic) << "\",";
      payload << "\"width\":" << _msg->width() << ",";
      payload << "\"height\":" << _msg->height() << ",";
      payload << "\"pixel_format\":" << _msg->pixel_format() << ",";
      payload << "\"data_size\":" << _msg->data().size() << ",";
      payload << "\"step\":" << _msg->step() << ",";
      payload << "\"image_data\":\"" << EscapeJson(image_base64) << "\"";
      payload << "}";

      zmq::message_t topic_msg(topic.size());
      memcpy(topic_msg.data(), topic.data(), topic.size());
      publisher.send(topic_msg, zmq::send_flags::sndmore);

      std::string payload_str = payload.str();
      zmq::message_t data_msg(payload_str.size());
      memcpy(data_msg.data(), payload_str.data(), payload_str.size());
      publisher.send(data_msg, zmq::send_flags::none);

      std::cout << "Published Gazebo image frame for topic " << topic
                << " (" << _msg->width() << "x" << _msg->height()
                << ", encoded_size=" << image_base64.size() << ")" << std::endl;
    };

  node->Subscribe(topic, callback);

  while (true) {
    gazebo::common::Time::MSleep(50);
  }

  gazebo::client::shutdown();
  return 0;
}
