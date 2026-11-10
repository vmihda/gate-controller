#include "matter_app.h"

#include <atomic>
#include <cstdint>
#include <optional>

#include <app/server/Server.h>
#include <esp_log.h>
#include <esp_matter.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <platform/CHIPDeviceLayer.h>
#include <sdkconfig.h>

#include "gate_task.h"
#include "gate_view.h"
#include "setup_code.h"
#include "status_led.h"

#if CONFIG_GATE_ENDPOINT_CLOSURE
#include <clusters/closure_control/integration.h>
#elif CONFIG_GATE_ENDPOINT_WINDOW_COVERING
#include <app-common/zap-generated/attributes/Accessors.h>
#include <app/clusters/window-covering-server/WindowCoveringCluster.h>
#elif CONFIG_GATE_ENDPOINT_DOOR_LOCK
#include <app/clusters/door-lock-server/door-lock-server.h>
#endif

#if CONFIG_GATE_ENDPOINT_CLOSURE && !CONFIG_SUPPORT_CLOSURE_CONTROL_CLUSTER
#error "The Closure endpoint needs CONFIG_SUPPORT_CLOSURE_CONTROL_CLUSTER (sdkconfig.matter)"
#endif
#if CONFIG_GATE_ENDPOINT_WINDOW_COVERING && !CONFIG_SUPPORT_WINDOW_COVERING_CLUSTER
#error "The Window Covering endpoint needs CONFIG_SUPPORT_WINDOW_COVERING_CLUSTER (sdkconfig.window-covering)"
#endif
#if CONFIG_GATE_ENDPOINT_DOOR_LOCK && !CONFIG_SUPPORT_DOOR_LOCK_CLUSTER
#error "The Door Lock endpoint needs CONFIG_SUPPORT_DOOR_LOCK_CLUSTER (sdkconfig.door-lock)"
#endif

using namespace esp_matter;
using chip::app::DataModel::Nullable;
using Status = chip::Protocols::InteractionModel::Status;

namespace {

const char *const TAG = "matter_app";

matter_command_handler s_on_command = nullptr;
uint16_t s_endpoint = 0;

// Every status from the gate task is folded into s_view at once, on the gate task (gate_view_update is
// pure and short), so a status that a later one supersedes before the Matter task runs still counts: CLOSED
// then UNKNOWN must leave the view closed. The Matter task shows a copy.
portMUX_TYPE s_view_lock = portMUX_INITIALIZER_UNLOCKED;
gate_view_t s_view{};
std::atomic<bool> s_started{false};
std::atomic<bool> s_apply_scheduled{false};
esp_timer_handle_t s_retry_timer = nullptr;  // retries a ScheduleWork that failed, so the newest status is not lost
constexpr uint64_t schedule_retry_us = 200'000;

// esp_matter::factory_reset() only schedules the CHIP part on the Matter task (Server::ScheduleFactoryReset,
// then ConfigurationManagerImpl::DoFactoryReset erases fabrics and Wi-Fi and calls esp_restart()). This bounds
// the wait for it.
constexpr TickType_t factory_reset_timeout = pdMS_TO_TICKS(10'000);

// Matter task only.
gate_error_t s_shown_error = GATE_ERR_NONE;  // the error the endpoint shows: tells a new error from a shown one

// Each kind's show() writes the whole view to the endpoint, every time, and returns whether the error step
// was done; only then does s_shown_error advance, so a failed error write is retried at the next apply.

// False when the gate task dropped the command (it logs that); each kind then reports a failure to Home.
bool command_from_home(gate_cmd_t cmd)
{
    ESP_LOGI(TAG, "command from Home: %s", cmd == GATE_CMD_OPEN ? "open" : "close");
    return s_on_command(cmd);
}

// ============================================================================================================
// Closure (device type 0x0230), Closure Control cluster 0x0104, feature Positioning only.
// Attributes (zzz_generated/app-common/clusters/ClosureControl/AttributeIds.h): MainState 0x0001,
// CurrentErrorList 0x0002, OverallCurrentState 0x0003, OverallTargetState 0x0004. The cluster object
// (app/clusters/closure-control-server/ClosureControlCluster.h) owns them; the endpoint is driven through it.
// ============================================================================================================
#if CONFIG_GATE_ENDPOINT_CLOSURE
namespace ClosureControl = chip::app::Clusters::ClosureControl;

// ClosureErrorEnum (ClosureControl/Enums.h): the Nice obstacle reversal is "physically blocked"; every other
// failed command (no reaction, wrong direction, unexpected stop, travel timeout, unknown state) is
// "internal interference".
ClosureControl::ClosureErrorEnum closure_error(gate_error_t error)
{
    return error == GATE_ERR_OBSTACLE ? ClosureControl::ClosureErrorEnum::kPhysicallyBlocked
                                      : ClosureControl::ClosureErrorEnum::kInternalInterference;
}

class closure_delegate final : public ClosureControl::ClosureControlClusterDelegate {
public:
    // The gate has no stop command (spec, "Commands"): Stop is answered UNSUPPORTED_COMMAND and the gate
    // goes on. The cluster lists Stop as accepted, because Positioning without Instantaneous requires it.
    Status HandleStopCommand() override { return Status::UnsupportedCommand; }

    Status HandleMoveToCommand(const chip::Optional<ClosureControl::TargetPositionEnum> &position,
                               const chip::Optional<bool> &,
                               const chip::Optional<chip::app::Clusters::Globals::ThreeLevelAutoEnum> &) override
    {
        if (!position.HasValue()) {
            return Status::InvalidCommand;
        }
        // A dropped command is answered BUSY; the cluster then leaves MainState and the target unchanged.
        switch (position.Value()) {
        case ClosureControl::TargetPositionEnum::kMoveToFullyOpen:
            return command_from_home(GATE_CMD_OPEN) ? Status::Success : Status::Busy;
        case ClosureControl::TargetPositionEnum::kMoveToFullyClosed:
            return command_from_home(GATE_CMD_CLOSE) ? Status::Success : Status::Busy;
        default:
            return Status::ConstraintError;
        }
    }

    Status HandleCalibrateCommand() override { return Status::UnsupportedCommand; }
    bool IsReadyToMove() override { return true; }
    chip::ElapsedS GetCalibrationCountdownTime() override { return 0; }
    chip::ElapsedS GetMovingCountdownTime() override { return 0; }
    chip::ElapsedS GetWaitingForMotionCountdownTime() override { return 0; }
};

closure_delegate s_closure_delegate;

endpoint_t *create_gate_endpoint(node_t *node)
{
    endpoint::closure::config_t config;
    config.closure_control.feature_flags = cluster::closure_control::feature::positioning::get_id();
    config.closure_control.delegate = &s_closure_delegate;
    return endpoint::closure::create(node, &config, ENDPOINT_FLAG_NONE, nullptr);
}

ClosureControl::CurrentPositionEnum closure_position(gate_position_t position)
{
    return position == GATE_POSITION_CLOSED ? ClosureControl::CurrentPositionEnum::kFullyClosed
                                            : ClosureControl::CurrentPositionEnum::kFullyOpened;
}

ClosureControl::TargetPositionEnum closure_target(gate_position_t position)
{
    return position == GATE_POSITION_CLOSED ? ClosureControl::TargetPositionEnum::kMoveToFullyClosed
                                            : ClosureControl::TargetPositionEnum::kMoveToFullyOpen;
}

// Logs a failed cluster write; true when it succeeded.
bool closure_ok(const char *what, CHIP_ERROR err)
{
    if (err != CHIP_NO_ERROR) {
        ESP_LOGE(TAG, "Closure %s failed: %" CHIP_ERROR_FORMAT, what, err.Format());
        return false;
    }
    return true;
}

// MainState: Moving while the gate moves or a command runs, else Stopped. A new error goes into
// CurrentErrorList, which also sends the OperationalError event and sets MainState to Error; MainState then
// returns to Stopped, because the cluster refuses MoveTo in the Error state and Home must be able to send the
// next command. The list stays filled until the next command starts. Each step runs whatever the others did.
// False before the cluster exists.
bool show(const gate_view_t &view, gate_error_t shown_error)
{
    ClosureControl::ClosureControlCluster *const cluster = ClosureControl::GetClusterInstance(s_endpoint);
    if (cluster == nullptr) {
        return false;
    }

    // Null until the first recognized state, and the cluster refuses MoveTo while it is null. A recognized
    // motion before any stationary state (a restart during motion) gives a state with a null position and a
    // null SecureState, so Home can send commands again.
    Nullable<ClosureControl::GenericOverallCurrentState> current;
    if (view.position != GATE_POSITION_UNKNOWN) {
        // SecureState: closed. Without MotionLatching, the cluster accepts true only for kFullyClosed.
        current.SetNonNull(ClosureControl::GenericOverallCurrentState(
            chip::MakeOptional(chip::app::DataModel::MakeNullable(closure_position(view.position))), chip::NullOptional,
            chip::NullOptional, chip::app::DataModel::MakeNullable(view.position == GATE_POSITION_CLOSED)));
    } else if (view.motion != GATE_MOTION_NONE) {
        current.SetNonNull(ClosureControl::GenericOverallCurrentState(
            chip::MakeOptional(Nullable<ClosureControl::CurrentPositionEnum>()), chip::NullOptional, chip::NullOptional,
            chip::app::DataModel::NullNullable));
    }
    closure_ok("OverallCurrentState", cluster->SetOverallCurrentState(current));

    gate_position_t target = GATE_POSITION_UNKNOWN;
    if (gate_view_target(&view, &target)) {
        Nullable<ClosureControl::GenericOverallTargetState> overall_target;
        if (target != GATE_POSITION_UNKNOWN) {
            overall_target.SetNonNull(ClosureControl::GenericOverallTargetState(
                chip::MakeOptional(chip::app::DataModel::MakeNullable(closure_target(target)))));
        }
        closure_ok("OverallTargetState", cluster->SetOverallTargetState(overall_target));
    }

    bool error_shown = true;
    if (view.error != shown_error) {
        cluster->ClearCurrentErrorList();
        if (view.error != GATE_ERR_NONE) {
            error_shown =
                closure_ok("CurrentErrorList", cluster->AddErrorToCurrentErrorList(closure_error(view.error)));
        }
    }
    closure_ok("MainState", cluster->SetMainState(gate_view_active(&view) ? ClosureControl::MainStateEnum::kMoving
                                                                         : ClosureControl::MainStateEnum::kStopped));
    return error_shown;
}

// ============================================================================================================
// Window Covering (device type 0x0202), Window Covering cluster 0x0102, features Lift and PositionAwareLift.
// Attributes (zzz_generated/app-common/clusters/WindowCovering/AttributeIds.h): OperationalStatus 0x000A,
// TargetPositionLiftPercent100ths 0x000B, CurrentPositionLiftPercent100ths 0x000E, SafetyStatus 0x001A.
// Percent100ths: 0 is fully open, 10000 fully closed. OperationalStatus is computed by the SDK from target
// against current (WindowCoveringCluster.cpp, PostAttributeChange): moving while they differ.
// ============================================================================================================
#elif CONFIG_GATE_ENDPOINT_WINDOW_COVERING
namespace WindowCovering = chip::app::Clusters::WindowCovering;

class window_covering_delegate final : public WindowCovering::WindowCoveringDelegate {
public:
    // UpOrOpen, DownOrClose and GoToLiftPercentage set TargetPositionLiftPercent100ths first. Fully closed
    // is "close"; any other target is "open", because the gate has no partial position.
    CHIP_ERROR HandleMovement(WindowCovering::WindowCoveringType type) override
    {
        if (type != WindowCovering::WindowCoveringType::Lift) {
            return CHIP_NO_ERROR;
        }
        WindowCovering::NPercent100ths target;
        if (WindowCovering::Attributes::TargetPositionLiftPercent100ths::Get(mEndpoint, target) != Status::Success ||
            target.IsNull()) {
            return CHIP_ERROR_INCORRECT_STATE;
        }
        if (command_from_home(target.Value() >= WC_PERCENT100THS_MAX_CLOSED ? GATE_CMD_CLOSE : GATE_CMD_OPEN)) {
            return CHIP_NO_ERROR;
        }
        // Dropped. The SDK answers SUCCESS whatever this returns (it only logs the error), so the target it
        // wrote is put back to the current position: the SDK then reports the covering stopped.
        WindowCovering::NPercent100ths current;
        if (WindowCovering::Attributes::CurrentPositionLiftPercent100ths::Get(mEndpoint, current) == Status::Success) {
            WindowCovering::Attributes::TargetPositionLiftPercent100ths::Set(mEndpoint, current);
        }
        return CHIP_ERROR_BUSY;
    }

    // The gate has no stop command: StopMotion is ignored. The SDK always answers it SUCCESS; IN_PROGRESS
    // keeps it from resetting the target to the current position (WindowCoveringCluster.cpp,
    // emberAfWindowCoveringClusterStopMotionCallback).
    CHIP_ERROR HandleStopMotion() override { return CHIP_ERROR_IN_PROGRESS; }
};

window_covering_delegate s_window_covering_delegate;

endpoint_t *create_gate_endpoint(node_t *node)
{
    endpoint::window_covering::config_t config(chip::to_underlying(WindowCovering::EndProductType::kRollerShutter));
    auto &wc = config.window_covering;
    wc.feature_flags = cluster::window_covering::feature::lift::get_id() |
                       cluster::window_covering::feature::position_aware_lift::get_id();
    wc.type = chip::to_underlying(WindowCovering::Type::kShutter);
    wc.config_status = chip::to_underlying(WindowCovering::ConfigStatus::kOperational) |
                       chip::to_underlying(WindowCovering::ConfigStatus::kLiftPositionAware);
    wc.delegate = &s_window_covering_delegate;
    endpoint_t *const endpoint = endpoint::window_covering::create(node, &config, ENDPOINT_FLAG_NONE, nullptr);
    if (endpoint == nullptr) {
        return nullptr;
    }
    // SafetyStatus is optional; it carries the command error.
    cluster_t *const cluster = cluster::get(endpoint, WindowCovering::Id);
    if (cluster == nullptr || cluster::window_covering::attribute::create_safety_status(cluster, 0) == nullptr) {
        return nullptr;
    }
    s_window_covering_delegate.SetEndpoint(endpoint::get_id(endpoint));
    return endpoint;
}

WindowCovering::NPercent100ths lift_percent100ths(gate_position_t position)
{
    WindowCovering::NPercent100ths value;
    if (position != GATE_POSITION_UNKNOWN) {
        value.SetNonNull(position == GATE_POSITION_CLOSED ? WC_PERCENT100THS_MAX_CLOSED : WC_PERCENT100THS_MIN_OPEN);
    }
    return value;
}

// SafetyStatus bits (WindowCovering/Enums.h): the Nice obstacle reversal is ObstacleDetected 0x0020, every
// other failed command PositionFailure 0x0008. Cleared to 0 when the next command starts.
chip::BitMask<WindowCovering::SafetyStatus> safety_status(gate_error_t error)
{
    chip::BitMask<WindowCovering::SafetyStatus> status;
    if (error == GATE_ERR_OBSTACLE) {
        status.Set(WindowCovering::SafetyStatus::kObstacleDetected);
    } else if (error != GATE_ERR_NONE) {
        status.Set(WindowCovering::SafetyStatus::kPositionFailure);
    }
    return status;
}

// Lift (and so global) OperationalStatus: the motion's direction, else stopped. Nothing while a command waits
// for the gate to move: the SDK's value from the command's target stays.
std::optional<WindowCovering::OperationalState> lift_state(const gate_view_t &view)
{
    switch (view.motion) {
    case GATE_MOTION_OPENING:
        return WindowCovering::OperationalState::MovingUpOrOpen;
    case GATE_MOTION_CLOSING:
        return WindowCovering::OperationalState::MovingDownOrClose;
    case GATE_MOTION_NONE:
        break;
    }
    if (view.busy) {
        return std::nullopt;
    }
    return WindowCovering::OperationalState::Stall;
}

// Logs a failed attribute write; true when it succeeded.
bool window_covering_ok(const char *what, Status status)
{
    if (status != Status::Success) {
        ESP_LOGE(TAG, "Window Covering %s failed: 0x%02x", what, chip::to_underlying(status));
        return false;
    }
    return true;
}

// The target goes first, so the SDK computes OperationalStatus from the new target and the new position.
bool show(const gate_view_t &view, gate_error_t)
{
    gate_position_t target = GATE_POSITION_UNKNOWN;
    if (gate_view_target(&view, &target)) {
        window_covering_ok("TargetPositionLiftPercent100ths",
                           WindowCovering::Attributes::TargetPositionLiftPercent100ths::Set(
                               s_endpoint, lift_percent100ths(target)));
    }
    // Only the Percent100ths position: WindowCovering::LiftPositionSet would also write the absolute and the
    // percent position, which this endpoint does not have.
    window_covering_ok("CurrentPositionLiftPercent100ths",
                       WindowCovering::Attributes::CurrentPositionLiftPercent100ths::Set(
                           s_endpoint, lift_percent100ths(view.position)));
    // OperationalStatus last, from the recognised motion: the SDK's own value from target against current says
    // "stopped" when they are equal or null, as after a stop halfway (position open, then OPENING towards
    // open) or a restart during motion (position null). Writing OperationalStatus recomputes nothing
    // (PostAttributeChange has no case for it).
    if (const std::optional<WindowCovering::OperationalState> state = lift_state(view)) {
        WindowCovering::OperationalStateSet(s_endpoint, WindowCovering::OperationalStatus::kLift, *state);
    }
    // The attribute accessor, not WindowCovering::SafetyStatusSet: the SDK header declares that one with a
    // const reference but defines it with a non-const one, so the declared function does not link.
    return window_covering_ok("SafetyStatus",
                              WindowCovering::Attributes::SafetyStatus::Set(s_endpoint, safety_status(view.error)));
}

// ============================================================================================================
// Door Lock (device type 0x000A), Door Lock cluster 0x0101, no optional features.
// Attributes (zzz_generated/app-common/clusters/DoorLock/AttributeIds.h): LockState 0x0000; event
// DoorLockAlarm 0x0000 (EventIds.h). The lock has no moving state: Locked only while the gate is closed and
// still, Unlocked as soon as a motion is recognized or the gate is open (gate_view_shut), so an opening
// gate does not stay "locked" until it stops.
// ============================================================================================================
#elif CONFIG_GATE_ENDPOINT_DOOR_LOCK
namespace DoorLock = chip::app::Clusters::DoorLock;

endpoint_t *create_gate_endpoint(node_t *node)
{
    endpoint::door_lock::config_t config;
    config.door_lock.lock_state = nullable<uint8_t>();  // unknown until the first recognized state, not NotFullyLocked
    config.door_lock.lock_type = chip::to_underlying(DoorLock::DlLockType::kOther);
    config.door_lock.actuator_enabled = true;
    config.door_lock.operating_mode = chip::to_underlying(DoorLock::OperatingModeEnum::kNormal);
    // A set bit marks an unsupported mode: Normal and NoRemoteLockUnlock (both mandatory) are clear.
    config.door_lock.supported_operating_modes = 0xFFF6;
    return endpoint::door_lock::create(node, &config, ENDPOINT_FLAG_NONE, nullptr);
}

// A new command error sends one DoorLockAlarm event with AlarmCode LockJammed (DoorLock/Enums.h,
// AlarmCodeEnum 0x00). An event cannot be withdrawn, so there is nothing to clear at the next command.
bool show(const gate_view_t &view, gate_error_t shown_error)
{
    DoorLockServer &server = DoorLockServer::Instance();
    bool shut = false;
    if (gate_view_shut(&view, &shut) &&
        !server.SetLockState(s_endpoint, shut ? DlLockState::kLocked : DlLockState::kUnlocked)) {
        ESP_LOGE(TAG, "Door Lock LockState failed");
    }
    if (view.error != GATE_ERR_NONE && view.error != shown_error &&
        !server.SendLockAlarmEvent(s_endpoint, AlarmCodeEnum::kLockJammed)) {
        ESP_LOGE(TAG, "Door Lock DoorLockAlarm failed");
        return false;
    }
    return true;
}
#else
#error "Unknown GATE_ENDPOINT_KIND"
#endif

// ============================================================================================================

void apply_pending(intptr_t)
{
    s_apply_scheduled.store(false);
    taskENTER_CRITICAL(&s_view_lock);
    const gate_view_t view = s_view;
    taskEXIT_CRITICAL(&s_view_lock);

    // Logged here, not on the gate task, so console output never delays the gate loop.
    const gate_status_t status = gate_task_status();
    ESP_LOGI(TAG, "gate: state=%s busy=%d error=%s", gate_state_name(status.state), status.busy ? 1 : 0,
             gate_error_name(status.error));

    if (show(view, s_shown_error)) {
        s_shown_error = view.error;
    }
}

void schedule_apply()
{
    if (s_apply_scheduled.exchange(true)) {
        return;
    }
    if (chip::DeviceLayer::PlatformMgr().ScheduleWork(apply_pending) != CHIP_NO_ERROR) {
        // No log here: this can run on the gate task, and console output must never delay its loop. The retry timer
        // callback logs instead.
        s_apply_scheduled.store(false);
        (void)esp_timer_start_once(s_retry_timer, schedule_retry_us);  // already running: that retry covers it
    }
}

void retry_schedule(void *)
{
    ESP_LOGW(TAG, "gate status was not scheduled, retrying");  // esp_timer task, not the gate task
    schedule_apply();
}

void refresh_connection_state()
{
    status_led_set_network(chip::Server::GetInstance().GetFabricTable().FabricCount() > 0,
                           chip::DeviceLayer::ConnectivityMgr().IsWiFiStationConnected());
}

void on_device_event(const chip::DeviceLayer::ChipDeviceEvent *event, intptr_t)
{
    using namespace chip::DeviceLayer::DeviceEventType;
    switch (event->Type) {
    case kServerReady:
        setup_code_print();
        refresh_connection_state();
        apply_pending(0);  // statuses that came before the cluster existed
        break;
    case kCommissioningComplete:
    case kFabricRemoved:
    case kWiFiConnectivityChange:
    case kInterfaceIpAddressChanged:
        refresh_connection_state();
        break;
    default:
        break;
    }
}

esp_err_t on_attribute_update(attribute::callback_type_t, uint16_t, uint32_t, uint32_t, esp_matter_attr_val_t *,
                              void *)
{
    return ESP_OK;
}

esp_err_t on_identify(identification::callback_type_t, uint16_t, uint8_t, uint8_t, void *)
{
    return ESP_OK;
}

}  // namespace

#if CONFIG_GATE_ENDPOINT_DOOR_LOCK
// Cluster init hook that esp-matter's legacy Door Lock cluster calls and the application must supply (as in
// esp-matter examples/door_lock). InitServer sets LockState to null; the kServerReady apply then shows the
// gate's state.
void emberAfDoorLockClusterInitCallback(chip::EndpointId endpoint)
{
    DoorLockServer::Instance().InitServer(endpoint);
}

// Door Lock server hooks (door-lock-server.h). No PIN: the cluster has no PIN or user features, so the
// server refuses commands that carry one before calling these. Lock is "close", Unlock is "open".
bool emberAfPluginDoorLockOnDoorLockCommand(chip::EndpointId, const Nullable<chip::FabricIndex> &,
                                            const Nullable<chip::NodeId> &, const chip::Optional<chip::ByteSpan> &,
                                            OperationErrorEnum &err)
{
    // A dropped command fails: the server answers FAILURE and sends LockOperationError with this reason.
    if (!command_from_home(GATE_CMD_CLOSE)) {
        err = OperationErrorEnum::kUnspecified;
        return false;
    }
    return true;
}

bool emberAfPluginDoorLockOnDoorUnlockCommand(chip::EndpointId, const Nullable<chip::FabricIndex> &,
                                              const Nullable<chip::NodeId> &, const chip::Optional<chip::ByteSpan> &,
                                              OperationErrorEnum &err)
{
    if (!command_from_home(GATE_CMD_OPEN)) {
        err = OperationErrorEnum::kUnspecified;
        return false;
    }
    return true;
}
#endif

esp_err_t matter_app_start(matter_command_handler on_command)
{
    if (on_command == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    s_on_command = on_command;
    gate_view_init(&s_view);
    const esp_timer_create_args_t retry_args = {
        .callback = retry_schedule,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "matter_retry",
        .skip_unhandled_events = true,
    };
    if (esp_timer_create(&retry_args, &s_retry_timer) != ESP_OK) {
        ESP_LOGE(TAG, "retry timer create failed");
        return ESP_ERR_NO_MEM;
    }

    node::config_t node_config;
    node_t *const node = node::create(&node_config, on_attribute_update, on_identify);
    if (node == nullptr) {
        ESP_LOGE(TAG, "node create failed");
        return ESP_FAIL;
    }
    endpoint_t *const gate = create_gate_endpoint(node);
    if (gate == nullptr) {
        ESP_LOGE(TAG, "gate endpoint create failed");
        return ESP_FAIL;
    }
    s_endpoint = endpoint::get_id(gate);

    setup_code_install();
    const esp_err_t err = esp_matter::start(on_device_event);
    if (err == ESP_OK) {
        s_started.store(true);
        schedule_apply();
    }
    return err;
}

void matter_app_publish(gate_state_t state, bool busy, gate_error_t error)
{
    taskENTER_CRITICAL(&s_view_lock);
    gate_view_update(&s_view, state, busy, error);
    taskEXIT_CRITICAL(&s_view_lock);
    if (s_started.load()) {
        schedule_apply();
    }
}

void matter_app_factory_reset()
{
    // An error here means only the esp-matter attribute store was not erased: factory_reset() schedules the
    // CHIP reset either way (esp_matter_core.cpp), so the wait below still lets it erase and restart.
    const esp_err_t err = esp_matter::factory_reset();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "factory reset: attribute store not erased: %s", esp_err_to_name(err));
    }
    // Runs on the button task, which the task watchdog does not watch; the gate task keeps running meanwhile.
    vTaskDelay(factory_reset_timeout);
    ESP_LOGE(TAG, "factory reset did not restart the chip in time, restarting");
    esp_restart();
}
