#include <remote.hpp>

// Update these with values suitable for your network.
byte mac[] = { 0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xED };
IPAddress ip(172, 16, 0, 100);
IPAddress server(172, 16, 0, 2);

EthernetClient ethClient;
network::Remote remote(server, ethClient, 1234);

bool simple_callback(String const& value)
{
  // do something with value
}

void setup()
{
  Ethernet.begin(mac, ip);
  remote.register_parameter("max_flow", simple_callback);
  remote.publish("Hello", "World!");
  remote.publish("pressure", 101300);
}

void loop()
{
  remote.loop();
}
