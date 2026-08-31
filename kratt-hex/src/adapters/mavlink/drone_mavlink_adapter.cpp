#include "adapters/mavlink/drone_mavlink_adapter.hpp"

#include <cmath>

namespace kratt::adapters::mavlink {
namespace {

/// MANUAL_CONTROL carries int16 axes in [-1000, 1000].
constexpr double kManualScale = 1000.0;

[[nodiscard]] double from_manual_axis(std::int16_t raw) noexcept {
    return domain::clamp(static_cast<double>(raw) / kManualScale, -1.0, 1.0);
}

[[nodiscard]] MAV_RESULT to_mav_result(domain::CommandResult result) noexcept {
    switch (result) {
        case domain::CommandResult::Accepted:
            return MAV_RESULT_ACCEPTED;
        case domain::CommandResult::AcceptedAsLand:
            // Understood but not executable as asked; TEMPORARILY_REJECTED tells
            // the GCS to expect it later, once the automatic landing finishes.
            return MAV_RESULT_TEMPORARILY_REJECTED;
        case domain::CommandResult::Rejected:
            return MAV_RESULT_DENIED;
    }
    return MAV_RESULT_UNSUPPORTED;
}

/// True when the message targets us, or is a broadcast (target_system == 0).
[[nodiscard]] bool addressed_to(const mavlink_message_t& message, std::uint8_t system_id) {
    switch (message.msgid) {
        case MAVLINK_MSG_ID_SET_MODE: {
            mavlink_set_mode_t payload{};
            mavlink_msg_set_mode_decode(&message, &payload);
            return payload.target_system == 0 || payload.target_system == system_id;
        }
        case MAVLINK_MSG_ID_COMMAND_LONG: {
            mavlink_command_long_t payload{};
            mavlink_msg_command_long_decode(&message, &payload);
            return payload.target_system == 0 || payload.target_system == system_id;
        }
        case MAVLINK_MSG_ID_MANUAL_CONTROL: {
            mavlink_manual_control_t payload{};
            mavlink_msg_manual_control_decode(&message, &payload);
            return payload.target == 0 || payload.target == system_id;
        }
        default:
            return true;
    }
}

}  // namespace

// ---------------------------------------------------------------------------
// Secondary adapter
// ---------------------------------------------------------------------------

MavlinkTelemetryPublisher::MavlinkTelemetryPublisher(IMessageChannel& channel,
                                                     std::uint8_t system_id)
    : channel_{channel}, system_id_{system_id} {}

void MavlinkTelemetryPublisher::publish(const domain::Telemetry& telemetry, domain::Instant now) {
    mavlink_message_t heartbeat{};
    // Only the two MAV_MODE values required by the specification go in
    // base_mode. custom_mode carries the finer internal mode: that is the
    // standard MAVLink slot for autopilot-specific state, so no custom message
    // has to be invented.
    const std::uint8_t base_mode =
        telemetry.armed ? static_cast<std::uint8_t>(MAV_MODE_GUIDED_ARMED)
                        : static_cast<std::uint8_t>(MAV_MODE_GUIDED_DISARMED);
    mavlink_msg_heartbeat_pack_status(system_id_, MAV_COMP_ID_AUTOPILOT1, &tx_status_, &heartbeat,
                                      MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, base_mode,
                                      static_cast<std::uint32_t>(telemetry.mode),
                                      telemetry.armed ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY);
    channel_.send(heartbeat);

    mavlink_message_t position{};
    const auto boot_ms = static_cast<std::uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
    // NED: z and vz point down; the domain's z points up. This single sign flip
    // is the entire coordinate-convention boundary of the project.
    mavlink_msg_local_position_ned_pack_status(
        system_id_, MAV_COMP_ID_AUTOPILOT1, &tx_status_, &position, boot_ms,
        static_cast<float>(telemetry.position.x), static_cast<float>(telemetry.position.y),
        static_cast<float>(-telemetry.position.z), static_cast<float>(telemetry.velocity.x),
        static_cast<float>(telemetry.velocity.y), static_cast<float>(-telemetry.velocity.z));
    channel_.send(position);
}

void MavlinkTelemetryPublisher::acknowledge(std::uint16_t command, domain::CommandResult result,
                                            std::uint8_t target_system) {
    mavlink_message_t ack{};
    mavlink_msg_command_ack_pack_status(system_id_, MAV_COMP_ID_AUTOPILOT1, &tx_status_, &ack,
                                        command, static_cast<std::uint8_t>(to_mav_result(result)),
                                        /*progress=*/255, /*result_param2=*/0, target_system,
                                        MAV_COMP_ID_MISSIONPLANNER);
    channel_.send(ack);
}

// ---------------------------------------------------------------------------
// Primary adapter
// ---------------------------------------------------------------------------

MavlinkCommandReceiver::MavlinkCommandReceiver(domain::IFlightCommands& commands,
                                               MavlinkTelemetryPublisher& publisher,
                                               std::uint8_t system_id)
    : commands_{commands}, publisher_{publisher}, system_id_{system_id} {}

void MavlinkCommandReceiver::handle(const mavlink_message_t& message, domain::Instant now) {
    if (!addressed_to(message, system_id_)) {
        return;
    }

    switch (message.msgid) {
        case MAVLINK_MSG_ID_SET_MODE: {
            mavlink_set_mode_t payload{};
            mavlink_msg_set_mode_decode(&message, &payload);
            const bool arm = (payload.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
            // SET_MODE has no COMMAND_ACK in MAVLink; the effect is observable
            // in the next HEARTBEAT, which is what the GCS already watches.
            if (arm) {
                (void)commands_.arm(now);
            } else {
                (void)commands_.disarm(now);
            }
            return;
        }
        case MAVLINK_MSG_ID_COMMAND_LONG: {
            mavlink_command_long_t payload{};
            mavlink_msg_command_long_decode(&message, &payload);
            switch (payload.command) {
                case MAV_CMD_OVERRIDE_GOTO: {
                    // param5/6/7 = x / y / z, z as altitude (up positive),
                    // consistent with the MAV_FRAME_LOCAL_ENU announced in param3.
                    const domain::Vec3 target{payload.param5, payload.param6, payload.param7};
                    const auto result = commands_.goto_target(target, now);
                    publisher_.acknowledge(MAV_CMD_OVERRIDE_GOTO, result, message.sysid);
                    return;
                }
                case MAV_CMD_NAV_LAND: {
                    const auto result = commands_.land(now);
                    publisher_.acknowledge(MAV_CMD_NAV_LAND, result, message.sysid);
                    return;
                }
                default:
                    return;  // unknown command: silently ignored, as MAVLink expects
            }
        }
        case MAVLINK_MSG_ID_MANUAL_CONTROL: {
            mavlink_manual_control_t payload{};
            mavlink_msg_manual_control_decode(&message, &payload);
            (void)commands_.manual_input(domain::Vec3{from_manual_axis(payload.x),
                                                      from_manual_axis(payload.y),
                                                      from_manual_axis(payload.z)},
                                         now);
            return;
        }
        default:
            return;
    }
}

void MavlinkCommandReceiver::handle_all(const std::vector<Inbound>& inbound, domain::Instant now) {
    for (const auto& item : inbound) {
        handle(item.message, now);
    }
}

}  // namespace kratt::adapters::mavlink
