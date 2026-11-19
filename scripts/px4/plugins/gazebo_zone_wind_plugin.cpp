// Fan-bank wind for Gazebo Classic, attached to the vehicle model (x8 / x500 SDF): while the vehicle is in a
// wind zone it gets the zone's wind, so that the disturbance comes only where the fans are (the test map's final
// demonstration scenario B: three blowers 5 m beside the WP2 -> WP3 lane, ~5 m/s cross wind).
//
// Two effects, both from the same wind vector w(t) (world ENU):
//   rotor drag  - w is published on ~/world_wind (physics_msgs::msgs::Wind), which PX4's gazebo_motor_model
//                 already turns into per-rotor drag and inflow; this is how PX4's own world wind works
//   body push   - F = 0.5 rho CdA |w| w on the link: the fan jet on the airframe (PX4's model has no body drag:
//                 10 m/s of wind gave only ~11.6 N of rotor drag on the X8, 2026-10-12). Wind only, not the
//                 airspeed: a drag on the vehicle's own speed would exist inside the box and nowhere else, a step
//                 in the dynamics at its edge.
// w(t) = sum over zones of weight(position) * (mean + gust(t)); weight is 1 inside the box and falls off
// smoothly over ramp_m outside it; the gust is an Ornstein-Uhlenbeck process per axis (std, time constant).
//
// The zones come from a text file (no file = the plugin does nothing, so the vehicle flies as before):
//   <configFile> (SDF, default /home/user/workspace/scenario_zone_wind.txt = the PX4 workspace the test map's
//   container mounts; scripts/dev/scenario_super.sh writes it), one zone per line, '#' comments:
//     name xmin xmax ymin ymax zmin zmax wx wy wz gust_std gust_tau cd_area ramp_m [airspeed]
//   (Gazebo world ENU metres, m/s, m/s, s, m^2, m). It is read when the plugin loads (the vehicle spawns).
//   airspeed (optional, default 0): 1 = the body drag acts on the air relative to the vehicle,
//   0.5 rho CdA |w - v| (w - v) with v the vehicle's horizontal velocity (a steady wind over the whole field,
//   scripts/dev/mc: the drag of the vehicle's own speed is real there); 0 = the fan jet only (wind w alone).
//   A line "seed N" sets the gust random seed (default 12345, so a run is repeatable; one seed per Monte
//   Carlo run). The environment variable ZONE_WIND_FILE overrides configFile (the compose stack's Gazebo
//   container, where the PX4 workspace is /home/user/workspace/px4).
// Each change of zone and a 5 Hz trace of the wind go to gzserver's log ("[zone_wind] ...").
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math/Vector3.hh>

#include "Wind.pb.h"

namespace gazebo
{
class GazeboZoneWindPlugin : public ModelPlugin
{
  struct Zone {
    std::string name;
    ignition::math::Vector3d min, max, mean;
    double gust_std{0.0}, gust_tau{1.0}, cd_area{0.0}, ramp{1.0};
    bool airspeed{false};
    ignition::math::Vector3d gust;
  };

public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    world_ = model->GetWorld();
    std::string link = sdf->HasElement("linkName") ? sdf->Get<std::string>("linkName") : "base_link";
    link_ = model->GetLink(link);
    if (!link_)
      link_ = model->GetLink();
    std::string file = sdf->HasElement("configFile") ? sdf->Get<std::string>("configFile")
                                                     : "/home/user/workspace/scenario_zone_wind.txt";
    const char *env_file = std::getenv("ZONE_WIND_FILE");
    if (env_file && *env_file)
      file = env_file;
    std::string topic = sdf->HasElement("windPubTopic") ? sdf->Get<std::string>("windPubTopic") : "world_wind";
    std::string ns = sdf->HasElement("robotNamespace") ? sdf->Get<std::string>("robotNamespace") : "";
    if (!ReadZones(file))
      return;   // no zones: stay out of the way (no wind messages, no force)
    node_ = transport::NodePtr(new transport::Node());
    node_->Init(ns);
    pub_ = node_->Advertise<physics_msgs::msgs::Wind>("~/" + topic, 10);
    rng_.seed(seed_);
    gzmsg << "[zone_wind] " << file << ", gust seed " << seed_ << std::endl;
    update_ = event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboZoneWindPlugin::OnUpdate, this));
    for (const Zone &z : zones_)
      gzmsg << "[zone_wind] " << z.name << ": wind " << z.mean << " m/s (gust std " << z.gust_std << ", tau "
            << z.gust_tau << " s), box " << z.min << " .. " << z.max << ", ramp " << z.ramp << " m, CdA "
            << z.cd_area << " m^2 on the " << (z.airspeed ? "airspeed" : "wind") << ", on " << model_->GetName()
            << "/" << link_->GetName() << std::endl;
  }

private:
  bool ReadZones(const std::string &file)
  {
    std::ifstream in(file);
    if (!in)
      return false;
    std::string line;
    while (std::getline(in, line)) {
      const size_t hash = line.find('#');
      if (hash != std::string::npos)
        line.erase(hash);
      std::istringstream s(line);
      std::string first;
      if (std::istringstream(line) >> first && first == "seed") {
        unsigned long v = 0;
        if (s >> first >> v)
          seed_ = static_cast<unsigned>(v);
        continue;
      }
      Zone z;
      double x0, x1, y0, y1, z0, z1, wx, wy, wz;
      if (!(s >> z.name >> x0 >> x1 >> y0 >> y1 >> z0 >> z1 >> wx >> wy >> wz >> z.gust_std >> z.gust_tau >> z.cd_area >>
            z.ramp))
        continue;
      z.min.Set(std::min(x0, x1), std::min(y0, y1), std::min(z0, z1));
      z.max.Set(std::max(x0, x1), std::max(y0, y1), std::max(z0, z1));
      z.mean.Set(wx, wy, wz);
      z.gust_tau = std::max(z.gust_tau, 0.05);
      z.ramp = std::max(z.ramp, 1e-3);
      int airspeed = 0;
      if (s >> airspeed)
        z.airspeed = airspeed != 0;
      zones_.push_back(z);
    }
    if (zones_.empty())
      gzwarn << "[zone_wind] " << file << " has no zone lines - plugin idle" << std::endl;
    return !zones_.empty();
  }

  // 1 inside the box, smoothly to 0 at ramp metres outside it (cosine), per axis
  static double Weight(const Zone &z, const ignition::math::Vector3d &p)
  {
    double w = 1.0;
    for (int i = 0; i < 3; ++i) {
      const double d = std::max({z.min[i] - p[i], p[i] - z.max[i], 0.0});
      if (d >= z.ramp)
        return 0.0;
      w *= 0.5 * (1.0 + std::cos(M_PI * d / z.ramp));
    }
    return w;
  }

  void OnUpdate()
  {
    const double t = world_->SimTime().Double();
    const double dt = last_t_ < 0.0 ? 0.0 : std::max(t - last_t_, 0.0);
    last_t_ = t;
    const ignition::math::Vector3d p = link_->WorldPose().Pos();
    const ignition::math::Vector3d vel = link_->WorldLinearVel();
    ignition::math::Vector3d wind(0, 0, 0), drag(0, 0, 0);
    std::string where;
    for (Zone &z : zones_) {
      if (dt > 0.0 && z.gust_std > 0.0) {   // Ornstein-Uhlenbeck: stationary std gust_std, correlation gust_tau
        const double a = std::exp(-dt / z.gust_tau), b = z.gust_std * std::sqrt(1.0 - a * a);
        for (int i = 0; i < 3; ++i)
          z.gust[i] = a * z.gust[i] + b * normal_(rng_);
        z.gust.Z(0.2 * z.gust.Z());   // fans blow mostly horizontally
      }
      const double w = Weight(z, p);
      if (w <= 0.0)
        continue;
      const ignition::math::Vector3d wz = w * (z.mean + z.gust);
      wind += wz;
      if (z.airspeed) {   // the air relative to the vehicle (horizontal own speed), weighted like the wind
        const ignition::math::Vector3d rel = z.mean + z.gust - ignition::math::Vector3d(vel.X(), vel.Y(), 0.0);
        drag += w * 0.5 * 1.225 * z.cd_area * rel.Length() * rel;
      } else {
        drag += 0.5 * 1.225 * z.cd_area * wz.Length() * wz;
      }
      if (w >= 1.0)
        where = z.name;
    }
    if (where != zone_) {
      gzmsg << "[zone_wind] t=" << t << " s vehicle " << (where.empty() ? "left " + zone_ : "in " + where) << " at " << p
            << std::endl;
      zone_ = where;
    }
    if (drag.Length() > 0.0)
      link_->AddForce(drag);
    // ~/world_wind for the motor models: 100 Hz while blowing, and one zero after leaving
    const bool blowing = wind.Length() > 1e-3;
    if ((blowing || was_blowing_) && t - last_pub_ >= 0.01) {
      physics_msgs::msgs::Wind msg;
      msg.set_frame_id("world");
      msg.set_time_usec(static_cast<int64_t>(t * 1e6));
      gazebo::msgs::Vector3d *v = new gazebo::msgs::Vector3d();
      v->set_x(wind.X());
      v->set_y(wind.Y());
      v->set_z(wind.Z());
      msg.set_allocated_velocity(v);
      pub_->Publish(msg);
      last_pub_ = t;
      was_blowing_ = blowing;
    }
    if (blowing && t - last_log_ >= 0.2) {
      gzmsg << "[zone_wind] t=" << t << " pos " << p << " wind " << wind << " drag " << drag << std::endl;
      last_log_ = t;
    }
  }

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  physics::LinkPtr link_;
  transport::NodePtr node_;
  transport::PublisherPtr pub_;
  event::ConnectionPtr update_;
  std::vector<Zone> zones_;
  std::mt19937 rng_;
  unsigned seed_{12345u};
  std::normal_distribution<double> normal_{0.0, 1.0};
  std::string zone_;
  double last_t_{-1.0}, last_pub_{-1.0}, last_log_{-1.0};
  bool was_blowing_{false};
};
GZ_REGISTER_MODEL_PLUGIN(GazeboZoneWindPlugin)
}  // namespace gazebo
