/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

// We'll use a string and the gzmsg command below for a brief example.
// Remove these includes if your plugin doesn't need them.
#include <string>
#include <gz/common/Console.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointPositionLimitsCmd.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/ParentEntity.hh>

// This header is required to register plugins. It's good practice to place it
// in the cc file, like it's done here.
#include <gz/plugin/Register.hh>

// Don't forget to include the plugin's header.
#include "OliRoverInterface.hh"

// This is required to register the plugin. Make sure the interfaces match
// what's in the header.
GZ_ADD_PLUGIN(
    oli_rover_interface::OliRoverInterface,
    gz::sim::System,
    oli_rover_interface::OliRoverInterface::ISystemPostUpdate)

using namespace oli_rover_interface;
using namespace gz;
using namespace sim;

// Here we implement the PostUpdate function, which is called at every
// iteration.
void OliRoverInterface::PostUpdate(const gz::sim::UpdateInfo &_info,
                                   const gz::sim::EntityComponentManager &_ecm)
{
  auto joints = _ecm.EntitiesByComponents(
      components::ParentEntity(this->targetEntity), components::Joint());

  auto children = _ecm.Descendants(this->targetEntity);

  for (auto child : joints)
  {
    gzmsg << gz::sim::scopedName(child, _ecm).c_str() << std::endl;
  }
  // Messages printed with gzmsg only show when running with verbosity 3 or
  // higher (i.e. gz sim -v 3)
  // gzmsg << msg << std::endl;
}

void OliRoverInterface::Configure(const gz::sim::Entity &_entity,
                                  const std::shared_ptr<const sdf::Element> &,
                                  gz::sim::EntityComponentManager &_ecm,
                                  gz::sim::EventManager &)
{
  gzmsg << gz::sim::scopedName(_entity, _ecm).c_str() << std::endl;
}