#include "franka_eit_vmc/runner.hpp"

#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include <array>
#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <thread>

#include "franka_eit_vmc/msg/eit_contact.hpp"
#include "franka_eit_vmc/msg/experiment_state.hpp"

namespace franka_eit_vmc {
namespace {
double steady_s() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
// One writer and one reader; fixed storage, no mutex, allocation or logging in the torque loop.
template <class T, size_t N>
class Ring {
 public:
  bool push(const T& value) {
    const auto w = write_.load(std::memory_order_relaxed), next = (w + 1) % N;
    if (next == read_.load(std::memory_order_acquire)) return false;
    values_[w] = value;
    write_.store(next, std::memory_order_release);
    return true;
  }
  bool pop(T& value) {
    const auto r = read_.load(std::memory_order_relaxed);
    if (r == write_.load(std::memory_order_acquire)) return false;
    value = values_[r];
    read_.store((r + 1) % N, std::memory_order_release);
    return true;
  }

 private:
  std::array<T, N> values_{};
  std::atomic<size_t> write_{0}, read_{0};
};
struct Telemetry {
  Output result;
  V7 q;
  V3 external_force;
};
class Bridge : public rclcpp::Node {
 public:
  explicit Bridge(Task task)
      : Node(task == Task::Joystick ? "eit_joystick"
             : task == Task::Ball   ? "eit_ball_roll"
                                    : "eit_move_ready") {
    c.task = task;
    host = declare_parameter<std::string>("robot_ip", "192.168.1.1");
    geometry_confirmed = declare_parameter<bool>("tool_transform_confirmed", false);
    mounting_confirmed = declare_parameter<bool>("mount_orientation_confirmed", false);
    pad_frame = declare_parameter<std::string>("pad_frame", "eit_pad");
    pad_numbering = declare_parameter<std::string>("pad_numbering", "ccw");
    if (pad_numbering != "ccw" && pad_numbering != "cw")
      throw std::invalid_argument("pad_numbering must be ccw or cw");
    expected_model = declare_parameter<std::string>("expected_model_sha256", "");
    const std::string prefix = declare_parameter<std::string>("joint_prefix", "fr3_");
    joint_names.reserve(7);
    for (int i = 1; i <= 7; ++i) joint_names.push_back(prefix + "joint" + std::to_string(i));
    auto vector = [this](const std::string& name, auto value) {
      const auto data = declare_parameter<std::vector<double>>(
          name, std::vector<double>(value.data(), value.data() + value.size()));
      if (data.size() != static_cast<size_t>(value.size()))
        throw std::invalid_argument(name + " has wrong length");
      for (int i = 0; i < value.size(); ++i) {
        value[i] = data[i];
      }
      return value;
    };
    c.ee_to_pad = vector("ee_to_pad_m", c.ee_to_pad);
    c.ready_pad_position = vector("ready_pad_position_m", c.ready_pad_position);
    c.ready_pad_position.x() += declare_parameter<double>("landing_offset_x_m", 0.);
    c.ready_pad_position.y() += declare_parameter<double>("landing_offset_y_m", 0.);
    c.workspace_min = vector("workspace_min_m", c.workspace_min);
    c.workspace_max = vector("workspace_max_m", c.workspace_max);
    c.q_min = vector("joint_min_rad", c.q_min);
    c.q_max = vector("joint_max_rad", c.q_max);
    c.torque_limit = vector("torque_limit_Nm", c.torque_limit);
    c.posture_reference = vector("posture_reference_rad", c.posture_reference);
    c.use_saved_posture = declare_parameter<bool>("use_saved_posture", false);
    auto quat = [this](const std::string& name) {
      const auto a = declare_parameter<std::vector<double>>(name, {0., 0., 0., 1.});
      if (a.size() != 4) throw std::invalid_argument(name + " needs XYZW");
      Eigen::Quaterniond q(a[3], a[0], a[1], a[2]);
      if (!q.coeffs().allFinite() || std::abs(q.norm() - 1.) > .001)
        throw std::invalid_argument(name + " must be a unit quaternion");
      return q.normalized().toRotationMatrix();
    };
    c.ee_R_pad = quat("ee_R_pad_xyzw");
    const double yaw = declare_parameter<double>("ready_pad_yaw_rad", 0.);
    c.ready_pad_rotation =
        (Eigen::AngleAxisd(yaw, V3::UnitZ()) * Eigen::AngleAxisd(3.141592653589793, V3::UnitX()))
            .toRotationMatrix();
#define P(x) c.x = declare_parameter<double>(#x, c.x)
    P(ready_speed);
    P(ready_angular_speed);
    P(ready_timeout);
    P(approach_speed);
    P(approach_distance);
    P(approach_timeout);
    P(retract_distance);
    P(preload);
    P(preload_time);
    P(hold_k);
    P(hold_d);
    P(hold_kr);
    P(hold_dr);
    P(coupling_k);
    P(coupling_d);
    P(force_xy_limit);
    P(normal_k);
    P(normal_d);
    P(normal_force_limit);
    P(orientation_k);
    P(orientation_d);
    P(moment_limit);
    P(null_k);
    P(null_d);
    P(torque_rate);
    P(joint_margin);
    P(joint_speed_limit);
    P(point_speed_limit);
    P(angular_speed_limit);
    P(external_force_limit);
    P(external_moment_limit);
    P(q7_tolerance);
    P(sensor_timeout);
    P(heartbeat_timeout);
    P(max_period);
    P(max_tracking_error);
    P(active_radius);
    P(joystick_length);
    P(joystick_angle);
    P(joystick_duration);
    P(angle_speed);
    P(angle_acceleration);
    P(initial_ball_x);
    P(roll_distance);
    P(ball_duration);
    P(virtual_mass);
    P(virtual_damping);
    P(goal_k);
    P(goal_force_limit);
    P(progress_speed);
#undef P
    c.update_ball_attachment = declare_parameter<bool>("update_ball_attachment", false);
    use_eit.store(declare_parameter<bool>("use_eit", true));
    c.validate();
    contact_sub = create_subscription<msg::EitContact>(
        declare_parameter<std::string>("contact_topic", "/eit/contact"), rclcpp::SensorDataQoS(),
        [this](msg::EitContact::ConstSharedPtr m) {
          Contact value;
          const double age = (get_clock()->now() - rclcpp::Time(m->header.stamp)).seconds();
          if (m->header.frame_id == pad_frame && m->numbering == pad_numbering &&
              std::isfinite(age) && age >= -.01 && age <= c.sensor_timeout &&
              std::isfinite(m->x_m) && std::isfinite(m->y_m) && m->model_sha256.size() == 64 &&
              (expected_model.empty() || m->model_sha256 == expected_model)) {
            value.sequence = m->sequence;
            value.acquired_steady_s = steady_s() - std::max(age, 0.);
            value.contact = m->contact;
            value.position_valid = m->position_valid;
            value.physical_model = m->model_is_physical && m->measurement_valid;
            value.x = m->x_m;
            value.y = m->y_m;
          }
          contacts.push(value);
        });
    heartbeat_sub = create_subscription<std_msgs::msg::Bool>(
        "~/heartbeat", rclcpp::QoS(1).best_effort(), [this](std_msgs::msg::Bool::ConstSharedPtr m) {
          last_heartbeat.store(m->data ? steady_s() : -1.);
        });
    for (const auto& item :
         std::array<std::pair<const char*, Request>, 3>{{{"move_ready", Request::Ready},
                                                         {"start_trial", Request::Start},
                                                         {"stop", Request::Stop}}}) {
      services.push_back(create_service<std_srvs::srv::Trigger>(
          std::string("~/") + item.first,
          [this, action = item.second](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                       std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            if (action != Request::Stop &&
                (mode.load() != Mode::Hold && mode.load() != Mode::Done)) {
              response->success = false;
              response->message = "Controller must be in hold or done";
              return;
            }
            if (action == Request::Stop) {
              request.store(Request::Stop);
              response->success = true;
              response->message = "Stop queued";
              return;
            }
            Request none = Request::None;
            response->success = request.compare_exchange_strong(none, action);
            response->message = response->success ? "Queued; observe status for actual execution"
                                                  : "A command is already pending";
          }));
    }
    condition_service = create_service<std_srvs::srv::SetBool>(
        "~/use_eit", [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> q,
                            std::shared_ptr<std_srvs::srv::SetBool::Response> a) {
          a->success = (mode.load() == Mode::Hold || mode.load() == Mode::Done) &&
                       request.load() == Request::None;
          if (a->success) use_eit.store(q->data);
          a->message = a->success ? "Condition queued for the next control step"
                                  : "Change condition only between trials";
        });
    status_pub = create_publisher<msg::ExperimentState>("~/status", 10);
    joint_pub = create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 10);
    timer = create_wall_timer(std::chrono::milliseconds(20), [this] { publish(); });
    parameter_guard = add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter>&) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = false;
      result.reason = "Restart to apply configuration; use ~/use_eit service between trials";
      return result;
    });
  }
  Config c;
  std::string host, pad_frame, pad_numbering, expected_model;
  bool geometry_confirmed = false, mounting_confirmed = false;
  Ring<Contact, 64> contacts;
  Ring<Telemetry, 64> telemetry;
  std::atomic<Request> request{Request::None};
  std::atomic<bool> use_eit{true};
  std::atomic<Mode> mode{Mode::Hold};
  std::atomic<double> last_heartbeat{-1.};

 private:
  void publish() {
    Telemetry value;
    bool found = false;
    while (telemetry.pop(value)) found = true;
    if (!found) return;
    const auto& o = value.result;
    msg::ExperimentState s;
    s.header.stamp = now();
    s.header.frame_id = "franka_base";
    s.mode = static_cast<uint8_t>(o.mode);
    s.fault = static_cast<uint8_t>(o.fault);
    s.mode_name = mode_name(o.mode);
    s.fault_name = fault_name(o.fault);
    s.use_eit = o.use_eit;
    s.trial_time_s = o.trial_time;
    s.progress_m = o.progress;
    s.progress_velocity_m_s = o.progress_velocity;
    s.ball_goal_x_m = o.goal;
    s.joystick_angle_command_rad = o.angle_command;
    for (int i = 0; i < 3; ++i) {
      s.pad_position_m[i] = o.pad_position[i];
      s.controlled_point_m[i] = o.point[i];
      s.reference_m[i] = o.reference[i];
      s.attachment_in_pad_m[i] = o.attachment[i];
      s.coupling_force_n[i] = o.coupling_force[i];
      s.external_force_n[i] = value.external_force[i];
    }
    for (int i = 0; i < 7; ++i) {
      s.joint_position_rad[i] = value.q[i];
      s.commanded_torque_nm[i] = o.torque[i];
    }
    status_pub->publish(s);
    sensor_msgs::msg::JointState js;
    js.header = s.header;
    js.name = joint_names;
    js.position.assign(value.q.data(), value.q.data() + 7);
    joint_pub->publish(js);
  }
  std::vector<std::string> joint_names;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_guard;
  rclcpp::Subscription<msg::EitContact>::SharedPtr contact_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr heartbeat_sub;
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> services;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr condition_service;
  rclcpp::Publisher<msg::ExperimentState>::SharedPtr status_pub;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub;
  rclcpp::TimerBase::SharedPtr timer;
};
Sample sample(const franka::RobotState& state, franka::Model& model) {
  Sample s;
  s.q = Eigen::Map<const V7>(state.q.data());
  s.dq = Eigen::Map<const V7>(state.dq.data());
  s.previous_torque = Eigen::Map<const V7>(state.tau_J_d.data());
  const Eigen::Map<const Eigen::Matrix4d> T(state.O_T_EE.data());
  s.ee_position = T.topRightCorner<3, 1>();
  s.ee_rotation = T.topLeftCorner<3, 3>();
  const auto J = model.zeroJacobian(franka::Frame::kEndEffector, state);
  s.jacobian = Eigen::Map<const J67>(J.data());
  const auto bias = model.coriolis(state);
  s.coriolis = Eigen::Map<const V7>(bias.data());
  s.external_force = Eigen::Map<const V3>(state.O_F_ext_hat_K.data());
  s.external_moment = Eigen::Map<const V3>(state.O_F_ext_hat_K.data() + 3);
  return s;
}
}  // namespace
int run_controller(int argc, char** argv, Task task) {
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    auto node = std::make_shared<Bridge>(task);
    if (!node->geometry_confirmed || !node->mounting_confirmed)
      throw std::runtime_error(
          "Measure EE-to-pad geometry and resolve the engraved-number orientation before setting "
          "both confirmation parameters true");
    RCLCPP_INFO(node->get_logger(),
                "Connecting directly through libfranka. Other FCI drivers must be stopped.");
    franka::Robot robot(node->host, franka::RealtimeConfig::kEnforce);
    franka::Model model = robot.loadModel();
    Core core(node->c);
    core.activate(sample(robot.readOnce(), model));
    core.set_condition(node->use_eit.load());
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin([&] { executor.spin(); });
    try {
      Contact contact;
      bool first = true, ready_requested = false, ready_seen = false;
      unsigned ticks = 0;
      double previous_robot_time = -1., stale = 0.;
      robot.control(
          [&](const franka::RobotState& state, franka::Duration duration) -> franka::Torques {
            Contact fresh;
            while (node->contacts.pop(fresh)) contact = fresh;
            double dt = duration.toSec();
            if (first && dt == 0.) dt = .001;
            first = false;
            const double rt = state.time.toSec();
            if (previous_robot_time >= 0. && rt <= previous_robot_time)
              stale += std::isfinite(dt) ? dt : .001;
            else
              stale = 0.;
            if (stale > .02 || (previous_robot_time >= 0. && rt < previous_robot_time) ||
                static_cast<bool>(state.current_errors)) {
              core.robot_fault();
            }
            previous_robot_time = rt;
            core.set_condition(node->use_eit.load());
            Request request = node->request.exchange(Request::None);
            if (!rclcpp::ok()) request = Request::Stop;
            bool heartbeat = steady_s() - node->last_heartbeat.load() < node->c.heartbeat_timeout;
            if (task == Task::Ready) {
              heartbeat = rclcpp::ok();
              if (!ready_requested) {
                request = Request::Ready;
                ready_requested = true;
              }
              if (core.mode() == Mode::Ready) ready_seen = true;
              if (ready_seen && core.mode() == Mode::Hold) request = Request::Stop;
            }
            const Sample measured = sample(state, model);
            const auto out = core.step(measured, contact, steady_s(), dt, heartbeat, request);
            node->mode.store(out.mode);
            if (++ticks % 20 == 0 || out.finished)
              node->telemetry.push(Telemetry{out, measured.q, measured.external_force});
            std::array<double, 7> tau{};
            for (int i = 0; i < 7; ++i) tau[i] = out.torque[i];
            if (out.finished) {
              return franka::MotionFinished(franka::Torques(tau));
            }
            return franka::Torques(tau);
          },
          true);
      if (core.fault() != Fault::None) {
        code = 2;
        RCLCPP_ERROR(node->get_logger(), "Stopped: %s", fault_name(core.fault()));
      }
    } catch (...) {
      executor.cancel();
      spin.join();
      throw;
    }
    executor.cancel();
    spin.join();
  } catch (const std::exception& e) {
    std::cerr << "franka_eit_vmc: " << e.what() << '\n';
    code = 2;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return code;
}
}  // namespace franka_eit_vmc
