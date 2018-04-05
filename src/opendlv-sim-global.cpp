/*
 * Copyright (C) 2018 Ola Benderius
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include "world-object.hpp"

int32_t main(int32_t argc, char **argv) {
  int32_t retCode{0};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if (0 == commandlineArguments.count("cid") || 0 == commandlineArguments.count("freq") || 0 == commandlineArguments.count("frame-id")) {
    std::cerr << argv[0] << " integrates the global position of an object based on its kinematic state." << std::endl;
    std::cerr << "Usage:   " << argv[0] << " --frame-id=<ID of frame to integrate> --freq=<Integration frequency> --cid=<OpenDaVINCI session> [--x=<Initial X position] [--y=<Initial Y position] [--z=<Initial Z position] [--roll=<Initial roll angle (around X)] [--pitch=<Initial pitch angle (around Y)] [--yaw=<Initial yaw angle (around Z)] [--verbose]" << std::endl;
    std::cerr << "Example: " << argv[0] << " --frame-id=0 --freq=100 --cid=111" << std::endl;
    retCode = 1;
  } else {
    float const X{(commandlineArguments["x"].size() != 0) ? static_cast<float>(std::stof(commandlineArguments["x"])) : 0.0f};
    float const Y{(commandlineArguments["y"].size() != 0) ? static_cast<float>(std::stof(commandlineArguments["y"])) : 0.0f};
    float const Z{(commandlineArguments["z"].size() != 0) ? static_cast<float>(std::stof(commandlineArguments["z"])) : 0.0f};
    float const ROLL{(commandlineArguments["roll"].size() != 0) ? static_cast<float>(std::stof(commandlineArguments["roll"])) : 0.0f};
    float const PITCH{(commandlineArguments["pitch"].size() != 0) ? static_cast<float>(std::stof(commandlineArguments["pitch"])) : 0.0f};
    float const YAW{(commandlineArguments["yaw"].size() != 0) ? static_cast<float>(std::stof(commandlineArguments["yaw"])) : 0.0f};
    uint32_t const FRAME_ID = std::stoi(commandlineArguments["frame-id"]);
    bool const VERBOSE{commandlineArguments.count("verbose") != 0};
    uint16_t const CID = std::stoi(commandlineArguments["cid"]);
    float const FREQ = std::stof(commandlineArguments["freq"]);
    double const DT = 1.0 / FREQ;

    WorldObject worldObject{X, Y, Z, ROLL, PITCH, YAW};

    auto onKinematicState{[&FRAME_ID, &worldObject](cluon::data::Envelope &&envelope)
      {
        uint32_t const senderStamp = envelope.senderStamp();
        if (FRAME_ID == senderStamp) {
          auto kinematicState = cluon::extractMessage<opendlv::sim::KinematicState>(std::move(envelope));
          worldObject.setKinematicState(kinematicState);
        }
      }};

    cluon::OD4Session od4{CID};
    od4.dataTrigger(opendlv::sim::KinematicState::ID(), onKinematicState);

    auto atFrequency{[&FRAME_ID, &VERBOSE, &DT, &worldObject, &od4]() -> bool
      {
        opendlv::sim::Frame frame = worldObject.step(DT);

        cluon::data::TimeStamp sampleTime;
        od4.send(frame, sampleTime, FRAME_ID);
        if (VERBOSE) {
          std::cout << "Frame  with id " << FRAME_ID
            << " is at [x=" << frame.x() << ", y=" << frame.y() << ", z="
            << frame.z() << "] with the rotation [roll=" << frame.roll() << ", pitch="
            << frame.pitch() << ", yaw=" << frame.yaw() << "]." << std::endl;
        }

        return true;
      }};


    od4.timeTrigger(FREQ, atFrequency);
  }
  return retCode;
}
