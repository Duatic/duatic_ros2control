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
 * products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>

#include <cxxopts.hpp>
#include "duatic_duadrive_scanner/Scanner.hpp"

void handle_scan(duadrive_scanner::Scanner& scanner)
{
  std::cout << "Found: " << scanner.device_count() << " devices on the bus: " << scanner.get_interface_name()
            << std::endl;
  // id 0 is always the master
  for (int i = 1; i < scanner.device_count() + 1; i++) {
    std::cout << "Device: " << i << std::endl;
    std::cout << "  Name: " << scanner.get_device_name(i) << std::endl;

    if (scanner.is_duadrive(i)) {
      std::cout << "  Device is a DuaDrive:" << std::endl;

      const auto drive_info = scanner.get_firmware_drive_info(i);
      const auto build_info = scanner.get_firmware_build_info(i);

      std::cout << "  Type: " << drive_info.drive_type << std::endl;
      std::cout << "  Model: " << drive_info.drive_model << std::endl;
      std::cout << "  Name: " << drive_info.drive_name << std::endl;

      std::cout << "  Build: " << std::endl;
      std::cout << "    Date: " << build_info.build_date << std::endl;
      std::cout << "    git tag: " << build_info.git_tag << std::endl;
      std::cout << "    git hash: " << build_info.git_hash << std::endl;
    }
  }
}
void handle_list_adapters()
{
  std::cout << "Available buses:" << std::endl;

  const auto adapters = duadrive_scanner::ethercat::list_adapters();

  for (const auto& adap : adapters) {
    std::cout << "  " << adap << std::endl;
  }
}

int main(int argc, char** argv)
{
  cxxopts::Options options("duadrive_scanner", "Find and identify ");

  // clang-format off
    options.add_options()
        ("verb", "Actions to perform [scan, list_adapters]", cxxopts::value<std::string>())
        ("b,bus", "Ethercat Bus", cxxopts::value<std::string>()->default_value("eth0"))
        ("h,help", "Print usage");
  // clang-format on

  options.parse_positional({ "verb" });
  options.show_positional_help();
  options.set_width(200);

  const auto args = options.parse(argc, argv);

  // Check if help was passed or no verb was passed
  if (args.count("help") || !args.count("verb")) {
    std::cout << options.help() << std::endl;
    return 0;
  }

  const auto verb = args["verb"].as<std::string>();
  // Check if the right verb was passed
  if (verb != "scan" && verb != "list_adapters") {
    std::cout << "Please pass a valid action" << std::endl;
    std::cout << options.help() << std::endl;
    return -1;
  }

  try {
    if (verb == "list_adapters") {
      handle_list_adapters();
      return 0;
    }
    const auto bus = args["bus"].as<std::string>();

    duadrive_scanner::Scanner scanner(bus);
    scanner.connect();
    if (verb == "scan") {
      handle_scan(scanner);
      return 0;
    }
  } catch (const std::exception& ex) {
    std::cerr << "Error: " << ex.what() << std::endl;
    return -2;
  }

  return 0;
}
