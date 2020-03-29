#include <Ethernet.h>
#include <PubSubClient.h>
#include <map>

// MQTT interface
//
// This library aims to interface with the MQTT broker.
// It can be used both as input and output.
//
// INPUT
// The MQTT topic to receive data shall be in the form:
//   mvm/<id>/<parameter_name>
// E.g., on a Linux terminal:
//   $ mosquitto_pub -t mvm/1234/max_flow -m 17
//
// OUTPUT
// The MQTT topic of published variables will be in the form:
//   mvm/<id>/out/<parameter_name>
// E.g., on a Linux terminal:
//   $ mosquitto_sub -t mvm/1234/out/pressure

namespace network {

// The type of the object that can be passed to register_parameter
typedef bool (*callback_t)(String const& value);

// String interface is a nightmare. Help me to find something better than
// this, if you can.
inline
String to_String(byte const* p, unsigned len)
{
  String s;
  for (auto i = 0u; i != len; ++i) {
    s += p[i];
  }

  return s;
}

class Remote
{
  PubSubClient m_client;
  String m_base_topic;
  std::map<String, callback_t> m_cbmap;

  void callback(char* topic, byte* payload, unsigned int length)
  {
    auto par_name = topic + m_base_topic.length();
    auto it = m_cbmap.find(par_name);

    if (it != m_cbmap.end()) {
      // this might be necessary. Is payload \0-terminated?
      String data = to_String(payload, length);
      it->second(data);
    }
  }

 public:

  // Constructor
  // Arguments:
  // - IPaddress server_ip # the ip of the MQTT broker
  // - EthernetClient eth  # reference to an external EthernetClient object
  // - int ventilator_id   # the ventilator ID
  // - uint16_t port=1883  # the MQTT server port, default is 1883
  Remote(
      IPAddress const& server_ip, EthernetClient& eth
    , int ventilator_id, int port = 1883
  ) : m_client(
        server_ip, port
      , [this](char* topic, byte* payload, unsigned int length) {
          this->callback(topic, payload, length);
        }
      , eth
      )
    , m_base_topic(String("mvm/") + String(ventilator_id) + String("/"))
  {}

  // loop member function
  // Calls the PubSubClient loop function. Call it repeadedly
  bool loop()
  {
    return m_client.loop();
  }

  // register a parameter-callback pair
  // Arguments:
  // - String par_name # the parameter unique name
  // - callback_t cb   # the function to call when the par_name is
  //                     received from the broker.
  bool register_parameter(String const& par_name, callback_t cb)
  {
    m_cbmap[par_name] = cb;

    m_client.subscribe((m_base_topic + par_name).c_str());
  }

  // send a named value to the broker
  // Arguments:
  // - String name # the name the value refers to
  // - Value value # the value. Can be any value acceptable by String
  //                 constructor
  template<class Value>
  bool publish(String const& name, Value const& value)
  {
    auto const svalue = String(value);
    m_client.publish(
        (m_base_topic + "out/" + name).c_str()
      , svalue.c_str()
      , svalue.length()
    );
  }
};

} // ns network
