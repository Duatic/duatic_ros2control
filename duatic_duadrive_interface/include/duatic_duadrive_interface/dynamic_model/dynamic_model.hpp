/*
 * Copyright 2026 Duatic AG
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 * disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 * products derived from this software without  hardware_interface::CallbackReturn activate();t specific prior written
 * permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

/* sys */
#include <type_traits>

/* ros */
#include "hardware_interface/hardware_info.hpp"

/* duatic */
#include "duatic_duadrive_interface/duadrive_interface_mock.hpp"
#include "duatic_duadrive_interface/dynamic_model/dynamic_model_pinocchio.hpp"
#include "duatic_duadrive_interface/dynamic_model/dynamic_model_non.hpp"

namespace duatic::duadrive_interface::dynamic_model
{

/*
 * Concepts specifying a suitable DynamicModel Class
 */
template<class DynamicModelT>
concept DynamicModel = requires(DynamicModelT model
  ,const hardware_interface::HardwareComponentInterfaceParams& system_info
) {
  DynamicModelT(); // default constructable
  {model.on_init(system_info)} -> std::same_as<hardware_interface::CallbackReturn>; // initialize as if lifecycle node
};

/*
 * type trait to evaluate a default DynamicModel implementation
 */
template<typename DriveTypeT>
using default_dynamic_model_t = std::conditional_t<is_dua_drive_interface_mock_v<DriveTypeT>
  ,DynamicModelPinocchio
  ,DynamicModelNon>;

}  // namespace duatic::duadrive_interface::dynamic_model
