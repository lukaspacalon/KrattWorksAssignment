#include "app/gcs_composition.hpp"

namespace kratt::app {

GcsComposition::GcsComposition(adapters::mavlink::IMessageChannel& channel, Config config,
                               domain::IEventLog& event_log)
    : channel_{channel},
      transmitter_{channel},
      service_{config.service, transmitter_, event_log},
      receiver_{service_, service_} {}

void GcsComposition::tick(domain::Instant now) {
    receiver_.handle_all(channel_.poll(), now);
    service_.update(now);
}

}  // namespace kratt::app
