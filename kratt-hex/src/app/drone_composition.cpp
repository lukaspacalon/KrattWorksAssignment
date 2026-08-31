#include "app/drone_composition.hpp"

namespace kratt::app {

DroneComposition::DroneComposition(adapters::mavlink::IMessageChannel& channel, Config config,
                                   domain::IEventLog& event_log)
    : channel_{channel},
      publisher_{channel},
      service_{config.service, publisher_, event_log},
      receiver_{service_, publisher_} {}

void DroneComposition::tick(domain::Seconds dt, domain::Instant now) {
    receiver_.handle_all(channel_.poll(), now);
    service_.step(dt, now);
}

}  // namespace kratt::app
