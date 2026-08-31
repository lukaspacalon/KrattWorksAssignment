#include "adapters/mavlink/gcs_mavlink_adapter.hpp"

#include <cmath>

namespace kratt::adapters::mavlink {
namespace {

constexpr double kManualScale = 1000.0;

[[nodiscard]] std::int16_t to_manual_axis(double normalised) noexcept {
    return static_cast<std::int16_t>(
        std::lround(domain::clamp(normalised, -1.0, 1.0) * kManualScale));
}

}  // namespace

MavlinkCommandTransmitter::MavlinkCommandTransmitter(IMessageChannel& channel,
                                                     std::uint8_t system_id,
                                                     std::uint8_t target_system)
    : channel_{channel}, system_id_{system_id}, target_system_{target_system} {}

void MavlinkCommandTransmitter::send_set_mode(bool armed) {
    mavlink_message_t message{};
    const std::uint8_t base_mode = armed ? static_cast<std::uint8_t>(MAV_MODE_GUIDED_ARMED)
                                         : static_cast<std::uint8_t>(MAV_MODE_GUIDED_DISARMED);
    mavlink_msg_set_mode_pack_status(system_id_, MAV_COMP_ID_MISSIONPLANNER, &tx_status_, &message,
                                     target_system_, base_mode, /*custom_mode=*/0);
    channel_.send(message);
}

void MavlinkCommandTransmitter::transmit_arm() { send_set_mode(true); }
void MavlinkCommandTransmitter::transmit_disarm() { send_set_mode(false); }

void MavlinkCommandTransmitter::transmit_land() {
    mavlink_message_t message{};
    mavlink_msg_command_long_pack_status(system_id_, MAV_COMP_ID_MISSIONPLANNER, &tx_status_,
                                         &message, target_system_, MAV_COMP_ID_AUTOPILOT1,
                                         MAV_CMD_NAV_LAND, /*confirmation=*/0, 0.F, 0.F, 0.F, 0.F,
                                         0.F, 0.F, 0.F);
    channel_.send(message);
}

void MavlinkCommandTransmitter::transmit_goto(const domain::Vec3& target) {
    mavlink_message_t message{};
    // MAV_CMD_OVERRIDE_GOTO per the common dialect:
    //   param1 = MAV_GOTO_DO_CONTINUE, param2 = hold behaviour at arrival,
    //   param3 = coordinate frame, param4 = yaw, param5/6/7 = x / y / z.
    mavlink_msg_command_long_pack_status(
        system_id_, MAV_COMP_ID_MISSIONPLANNER, &tx_status_, &message, target_system_,
        MAV_COMP_ID_AUTOPILOT1, MAV_CMD_OVERRIDE_GOTO, /*confirmation=*/0, MAV_GOTO_DO_CONTINUE,
        MAV_GOTO_HOLD_AT_SPECIFIED_POSITION, MAV_FRAME_LOCAL_ENU, /*yaw=*/0.F,
        static_cast<float>(target.x), static_cast<float>(target.y), static_cast<float>(target.z));
    channel_.send(message);
}

void MavlinkCommandTransmitter::transmit_manual(const domain::Vec3& axes) {
    mavlink_message_t message{};
    // x = forward/north, y = right/east, z = vertical. MAVLink defines z as a
    // 0..1000 throttle for aircraft; for a simulated multirotor a signed climb
    // rate is the only sensible reading, so [-1000, 1000] is used and documented.
    mavlink_msg_manual_control_pack_status(system_id_, MAV_COMP_ID_MISSIONPLANNER, &tx_status_,
                                           &message, target_system_, to_manual_axis(axes.x),
                                           to_manual_axis(axes.y), to_manual_axis(axes.z),
                                           /*r=*/0, /*buttons=*/0, /*buttons2=*/0,
                                           /*enabled_extensions=*/0, 0, 0, 0, 0, 0, 0, 0, 0);
    channel_.send(message);
}

// ---------------------------------------------------------------------------

MavlinkTelemetryReceiver::MavlinkTelemetryReceiver(domain::ITelemetrySink& sink,
                                                   domain::ILinkLiveness& liveness)
    : sink_{sink}, liveness_{liveness} {}

bool MavlinkTelemetryReceiver::handle(const mavlink_message_t& message, domain::Instant now) {
    switch (message.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_heartbeat_t payload{};
            mavlink_msg_heartbeat_decode(&message, &payload);
            snapshot_.armed = (payload.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
            snapshot_.mode = static_cast<domain::FlightMode>(payload.custom_mode);
            ++heartbeats_;
            // Only the heartbeat drives the liveness clock, so a stale position
            // message can never keep a dead link looking alive.
            liveness_.on_heartbeat(now);
            sink_.publish(snapshot_, now);
            return true;
        }
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
            mavlink_local_position_ned_t payload{};
            mavlink_msg_local_position_ned_decode(&message, &payload);
            snapshot_.position = domain::Vec3{payload.x, payload.y, -payload.z};
            snapshot_.velocity = domain::Vec3{payload.vx, payload.vy, -payload.vz};
            sink_.publish(snapshot_, now);
            return true;
        }
        case MAVLINK_MSG_ID_COMMAND_ACK:
            ++acks_;
            return false;
        default:
            return false;
    }
}

void MavlinkTelemetryReceiver::handle_all(const std::vector<Inbound>& inbound,
                                          domain::Instant now) {
    for (const auto& item : inbound) {
        handle(item.message, now);
    }
}

}  // namespace kratt::adapters::mavlink
