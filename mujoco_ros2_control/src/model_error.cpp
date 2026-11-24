// Copyright (c) 2026 Wei-Hsuan Cheng
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "mujoco_ros2_control/model_error.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace mujoco_ros2_control
{
namespace
{
// One perturbed quantity: a fixed relative change, a relative change drawn per element, or an absolute value.
struct Perturbation
{
  enum class Kind { kNone, kRelative, kUniform, kAbsolute } kind{Kind::kNone};
  double low{0.0};
  double high{0.0};

  static Perturbation parse(const YAML::Node & node, const std::string & name)
  {
    Perturbation p;
    if (!node) return p;
    if (node.IsScalar()) {
      p.kind = Kind::kRelative;
      p.low = p.high = node.as<double>();
    } else if (node.IsMap() && node["uniform"]) {
      const auto range = node["uniform"].as<std::vector<double>>();
      if (range.size() != 2 || !(range[0] <= range[1])) {
        throw std::runtime_error("model_error " + name + ": uniform needs [lo, hi] with lo <= hi");
      }
      p.kind = Kind::kUniform;
      p.low = range[0];
      p.high = range[1];
    } else if (node.IsMap() && node["value"]) {
      p.kind = Kind::kAbsolute;
      p.low = p.high = node["value"].as<double>();
    } else {
      throw std::runtime_error("model_error " + name + ": expected a number, {uniform: [lo, hi]} or {value: v}");
    }
    if (!std::isfinite(p.low) || !std::isfinite(p.high)) {
      throw std::runtime_error("model_error " + name + ": values must be finite");
    }
    return p;
  }

  bool active() const { return kind != Kind::kNone; }

  // The relative change for one element (0 for an absolute value, which apply() handles).
  double draw(std::mt19937_64 & rng) const
  {
    if (kind == Kind::kUniform) return std::uniform_real_distribution<double>(low, high)(rng);
    return kind == Kind::kRelative ? low : 0.0;
  }

  double apply(double nominal, std::mt19937_64 & rng) const
  {
    if (kind == Kind::kAbsolute) return low;
    return nominal * (1.0 + draw(rng));
  }
};

// Independent streams per quantity, so enabling one error does not change the draws of another.
std::mt19937_64 stream(int64_t seed, int quantity)
{
  std::seed_seq sequence{static_cast<uint32_t>(seed), static_cast<uint32_t>(seed >> 32), static_cast<uint32_t>(quantity)};
  return std::mt19937_64(sequence);
}

std::string name_of(const mjModel * model, mjtObj type, int id)
{
  const char * name = mj_id2name(model, type, id);
  return name ? name : "#" + std::to_string(id);
}

std::set<std::string> names(const YAML::Node & node)
{
  std::set<std::string> result;
  if (node) {
    for (const auto & name : node.as<std::vector<std::string>>()) result.insert(name);
  }
  return result;
}

template <typename Values> std::string json_array(const Values & values, int count)
{
  std::ostringstream out;
  out << std::setprecision(10) << '[';
  for (int i = 0; i < count; ++i) out << (i ? "," : "") << values[i];
  out << ']';
  return out.str();
}
}  // namespace

constexpr const char * kRegionPrefix = "ground_region_";

mjModel * load_model(const std::string & model_path, const std::string & file)
{
  const YAML::Node regions = file.empty() ? YAML::Node() : YAML::LoadFile(file)["model_error"]["ground_regions"];
  char error[1000] = "";
  if (!regions) {
    const bool binary = model_path.size() > 4 && model_path.compare(model_path.size() - 4, 4, ".mjb") == 0;
    mjModel * model = binary ? mj_loadModel(model_path.c_str(), nullptr)
                             : mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
    if (!model) throw std::runtime_error("cannot load " + model_path + ": " + error);
    return model;
  }
  const double size = regions["size"] ? regions["size"].as<double>() : 2.0;
  const double extent = regions["extent"] ? regions["extent"].as<double>() : 20.0;
  if (!(size > 0.0) || !(extent >= size / 2.0) || extent / size > 100.0) {
    throw std::runtime_error("model_error ground_regions: need size > 0, extent >= size / 2, at most 200 patches a side");
  }
  mjSpec * spec = mj_parseXML(model_path.c_str(), nullptr, error, sizeof(error));
  if (!spec) throw std::runtime_error("ground_regions need an MJCF model: " + std::string(error));
  // The floor plane stops colliding (it stays visible); flush box patches carry the robot instead.
  mjsBody * world = mjs_findBody(spec, "world");
  mjsGeom * floor = nullptr;
  for (mjsElement * e = mjs_firstChild(world, mjOBJ_GEOM, 0); e; e = mjs_nextChild(world, e, 0)) {
    mjsGeom * geom = mjs_asGeom(e);
    if (geom->type == mjGEOM_PLANE && (geom->contype || geom->conaffinity)) {
      if (floor) {
        mj_deleteSpec(spec);
        throw std::runtime_error("ground_regions need exactly one colliding floor plane in the world body");
      }
      floor = geom;
    }
  }
  if (!floor) {
    mj_deleteSpec(spec);
    throw std::runtime_error("ground_regions need a colliding floor plane in the world body");
  }
  const int count = static_cast<int>(std::ceil(2.0 * extent / size - 1e-9));
  const double thickness = 0.1, start = -0.5 * count * size + 0.5 * size;
  for (int i = 0; i < count; ++i) {
    for (int j = 0; j < count; ++j) {
      mjsGeom * patch = mjs_addGeom(world, nullptr);
      patch->type = mjGEOM_BOX;
      patch->size[0] = patch->size[1] = 0.5 * size;
      patch->size[2] = 0.5 * thickness;
      patch->pos[0] = floor->pos[0] + start + i * size;
      patch->pos[1] = floor->pos[1] + start + j * size;
      patch->pos[2] = floor->pos[2] - 0.5 * thickness;
      patch->contype = floor->contype;
      patch->conaffinity = floor->conaffinity;
      patch->condim = floor->condim;
      patch->priority = 10;  // a patch's friction governs its contacts, not the larger of the pair's
      for (int k = 0; k < 3; ++k) patch->friction[k] = floor->friction[k];
      const float shade = (i + j) % 2 ? 0.55f : 0.6f;
      patch->rgba[0] = patch->rgba[1] = patch->rgba[2] = shade;
      patch->rgba[3] = 1.0f;
      mjs_setName(patch->element, (kRegionPrefix + std::to_string(i) + "_" + std::to_string(j)).c_str());
    }
  }
  floor->contype = floor->conaffinity = 0;
  mjModel * model = mj_compile(spec, nullptr);
  const std::string compile_error = model ? "" : mjs_getError(spec);
  mj_deleteSpec(spec);
  if (!model) throw std::runtime_error("ground_regions: cannot compile " + model_path + ": " + compile_error);
  return model;
}

std::string apply_model_error(mjModel * model, const std::string & file, int64_t seed, std::string & summary)
{
  const YAML::Node root = YAML::LoadFile(file)["model_error"];
  if (!root || !root.IsMap()) throw std::runtime_error("model error file " + file + " has no 'model_error' map");
  for (const auto & entry : root) {
    const auto key = entry.first.as<std::string>();
    if (key != "seed" && key != "inertia" && key != "joint_friction" && key != "contact_friction" &&
        key != "ground_regions" && key != "delay") {
      throw std::runtime_error("model_error: unknown entry '" + key + "'");
    }
  }
  if (seed < 0) seed = root["seed"] ? root["seed"].as<int64_t>() : 0;
  std::ostringstream json, text;
  json << std::setprecision(10) << "{\"file\":\"" << file << "\",\"seed\":" << seed;
  text << std::setprecision(4) << "model error " << file << " (seed " << seed << "):";

  // Inertia: mass, CoM and principal moments of the selected bodies.
  if (const YAML::Node inertia = root["inertia"]) {
    const auto mass = Perturbation::parse(inertia["mass"], "inertia.mass");
    const auto com = Perturbation::parse(inertia["com"], "inertia.com");
    const auto moments = Perturbation::parse(inertia["inertia"], "inertia.inertia");
    if (com.kind == Perturbation::Kind::kAbsolute) {
      throw std::runtime_error("model_error inertia.com: a shift relative to the radius of gyration, not {value}");
    }
    const auto only = names(inertia["bodies"]), excluded = names(inertia["exclude"]);
    auto mass_rng = stream(seed, 1), com_rng = stream(seed, 2), moment_rng = stream(seed, 3);
    double total_before = 0.0, total_after = 0.0;
    json << ",\"bodies\":[";
    bool first = true;
    for (int body = 1; body < model->nbody; ++body) {
      const auto name = name_of(model, mjOBJ_BODY, body);
      total_before += model->body_mass[body];
      if ((!only.empty() && !only.count(name)) || excluded.count(name) || model->body_mass[body] <= 0.0) {
        total_after += model->body_mass[body];
        continue;
      }
      double * m = &model->body_mass[body];
      double * c = &model->body_ipos[3 * body];
      double * i = &model->body_inertia[3 * body];
      const double m0 = *m;
      const std::array<double, 3> c0{c[0], c[1], c[2]}, i0{i[0], i[1], i[2]};
      // The body's size for the CoM shift: its radius of gyration, sqrt(trace(I) / (2 m)).
      const double gyration = std::sqrt((i0[0] + i0[1] + i0[2]) / (2.0 * m0));
      if (mass.active()) *m = mass.apply(m0, mass_rng);
      for (int k = 0; k < 3; ++k) {
        if (com.active()) c[k] = c0[k] + com.draw(com_rng) * gyration;
        if (moments.active()) i[k] = moments.apply(i0[k], moment_rng);
      }
      if (!(*m > 0.0) || !(i[0] > 0.0 && i[1] > 0.0 && i[2] > 0.0)) {
        throw std::runtime_error("model_error inertia: body '" + name + "' would get a nonpositive mass or moment");
      }
      // A principal moment cannot exceed the sum of the other two; clip the largest.
      const int largest = static_cast<int>(std::max_element(i, i + 3) - i);
      const double others = i[(largest + 1) % 3] + i[(largest + 2) % 3];
      if (i[largest] > others) i[largest] = others;
      total_after += *m;
      json << (first ? "" : ",") << "{\"name\":\"" << name << "\",\"mass\":[" << m0 << ',' << *m << "],\"com\":["
           << json_array(c0, 3) << ',' << json_array(c, 3) << "],\"inertia\":[" << json_array(i0, 3) << ','
           << json_array(i, 3) << "]}";
      first = false;
    }
    json << "],\"total_mass\":[" << total_before << ',' << total_after << ']';
    text << " total mass " << total_before << " -> " << total_after << " kg;";
  }

  // Joint friction, damping and armature of the selected non-free joints.
  if (const YAML::Node friction = root["joint_friction"]) {
    const std::array<Perturbation, 3> spec{
      Perturbation::parse(friction["frictionloss"], "joint_friction.frictionloss"),
      Perturbation::parse(friction["damping"], "joint_friction.damping"),
      Perturbation::parse(friction["armature"], "joint_friction.armature")};
    const std::array<mjtNum *, 3> fields{model->dof_frictionloss, model->dof_damping, model->dof_armature};
    const std::array<const char *, 3> labels{"frictionloss", "damping", "armature"};
    const auto only = names(friction["joints"]);
    std::array<std::mt19937_64, 3> rng{stream(seed, 4), stream(seed, 5), stream(seed, 6)};
    json << ",\"joints\":[";
    bool first = true;
    for (int joint = 0; joint < model->njnt; ++joint) {
      const auto name = name_of(model, mjOBJ_JOINT, joint);
      if (model->jnt_type[joint] == mjJNT_FREE || (!only.empty() && !only.count(name))) continue;
      const int dofs = model->jnt_type[joint] == mjJNT_BALL ? 3 : 1;
      json << (first ? "" : ",") << "{\"name\":\"" << name << '"';
      for (int f = 0; f < 3; ++f) {
        if (!spec[f].active()) continue;
        std::vector<double> before, after;
        for (int d = model->jnt_dofadr[joint]; d < model->jnt_dofadr[joint] + dofs; ++d) {
          before.push_back(fields[f][d]);
          fields[f][d] = std::max(0.0, spec[f].apply(fields[f][d], rng[f]));
          after.push_back(fields[f][d]);
        }
        json << ",\"" << labels[f] << "\":[" << json_array(before, dofs) << ',' << json_array(after, dofs) << ']';
      }
      json << '}';
      first = false;
    }
    json << ']';
    text << " joint friction applied;";
  }

  // Sliding friction of every geom: MuJoCo uses the larger of the two geoms' coefficients for a contact.
  if (root["contact_friction"]) {
    const auto spec = Perturbation::parse(root["contact_friction"], "contact_friction");
    auto rng = stream(seed, 7);
    double low = INFINITY, high = 0.0;
    for (int geom = 0; geom < model->ngeom; ++geom) {
      model->geom_friction[3 * geom] = std::max(0.0, spec.apply(model->geom_friction[3 * geom], rng));
      low = std::min(low, model->geom_friction[3 * geom]);
      high = std::max(high, model->geom_friction[3 * geom]);
    }
    json << ",\"contact_friction\":[" << low << ',' << high << ']';
    text << " sliding friction " << low << "-" << high << ';';
  }
  // Ground patches (added by load_model): each draws its own sliding friction; the colour shows it (darker = lower).
  if (const YAML::Node regions = root["ground_regions"]) {
    const auto spec = Perturbation::parse(regions["friction"], "ground_regions.friction");
    auto rng = stream(seed, 8);
    double low = INFINITY, high = 0.0;
    int patches = 0;
    json << ",\"ground_regions\":[";
    for (int geom = 0; geom < model->ngeom; ++geom) {
      const char * name = mj_id2name(model, mjOBJ_GEOM, geom);
      if (!name || std::string(name).rfind(kRegionPrefix, 0) != 0) continue;
      const double nominal = model->geom_friction[3 * geom];
      const double mu = std::max(0.0, spec.apply(nominal, rng));
      model->geom_friction[3 * geom] = mu;
      const float shade = static_cast<float>(std::clamp(0.25 + 0.5 * mu / std::max(nominal, 1e-9), 0.1, 0.9));
      model->geom_rgba[4 * geom] = model->geom_rgba[4 * geom + 1] = shade;
      model->geom_rgba[4 * geom + 2] = std::min(1.0f, shade + 0.15f);
      json << (patches ? "," : "") << "[" << model->geom_pos[3 * geom] << ',' << model->geom_pos[3 * geom + 1] << ','
           << mu << ']';
      low = std::min(low, mu);
      high = std::max(high, mu);
      ++patches;
    }
    if (!patches) throw std::runtime_error("model_error ground_regions: the model has no patches (load it with load_model)");
    json << "],\"ground_region_friction\":[" << low << ',' << high << ']';
    text << " " << patches << " ground patches, sliding friction " << low << "-" << high << ';';
  }
  json << '}';
  summary = text.str();
  return json.str();
}
Delays read_delays(const std::string & file, int64_t seed)
{
  Delays delays;
  const YAML::Node delay = YAML::LoadFile(file)["model_error"]["delay"];
  if (!delay) return delays;
  if (seed < 0) {
    const YAML::Node root = YAML::LoadFile(file)["model_error"];
    seed = root["seed"] ? root["seed"].as<int64_t>() : 0;
  }
  const auto seconds = [&](const char * key, int quantity) {
    const YAML::Node node = delay[key];
    if (!node) return 0.0;
    double value = 0.0;
    if (node.IsScalar()) {
      value = node.as<double>();
    } else if (node.IsMap() && node["uniform"]) {
      const auto range = node["uniform"].as<std::vector<double>>();
      if (range.size() != 2 || !(range[0] <= range[1])) {
        throw std::runtime_error(std::string("model_error delay.") + key + ": uniform needs [lo, hi] with lo <= hi");
      }
      auto rng = stream(seed, quantity);
      value = std::uniform_real_distribution<double>(range[0], range[1])(rng);
    } else {
      throw std::runtime_error(std::string("model_error delay.") + key + ": expected seconds or {uniform: [lo, hi]}");
    }
    if (!(value >= 0.0 && value <= 1.0)) {
      throw std::runtime_error(std::string("model_error delay.") + key + ": must lie in [0, 1] s");
    }
    return value;
  };
  for (const auto & entry : delay) {
    const auto key = entry.first.as<std::string>();
    if (key != "sensing" && key != "actuation") throw std::runtime_error("model_error delay: unknown entry '" + key + "'");
  }
  delays.sensing = seconds("sensing", 9);
  delays.actuation = seconds("actuation", 10);
  return delays;
}
}  // namespace mujoco_ros2_control
