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

#include <Eigen/Eigen>

#include "world-object.hpp"

WorldObject::WorldObject(float x, float y, float z, float roll, float pitch, float yaw) noexcept:
  m_frame{},
  m_kinematicState{},
  m_kinematicStateMutex{}
{
  m_frame.x(x);
  m_frame.y(y);
  m_frame.z(z);
  m_frame.roll(roll);
  m_frame.pitch(pitch);
  m_frame.yaw(yaw);
}

void WorldObject::setKinematicState(opendlv::sim::KinematicState const &kinematicState) noexcept
{
  std::lock_guard<std::mutex> lock(m_kinematicStateMutex);
  m_kinematicState = kinematicState;
}

opendlv::sim::Frame WorldObject::step(double dt) noexcept
{
  opendlv::sim::KinematicState kinematicState;
  {
    kinematicState = m_kinematicState;
    std::lock_guard<std::mutex> lock(m_kinematicStateMutex);
  }

  float const x = m_frame.x();
  float const y = m_frame.y();
  float const z = m_frame.z();
  float const roll = m_frame.roll();
  float const pitch = m_frame.pitch();
  float const yaw = m_frame.yaw();

  float const vx = kinematicState.vx();
  float const vy = kinematicState.vy();
  float const vz = kinematicState.vz();
  float const rollRate = kinematicState.rollRate();
  float const pitchRate = kinematicState.pitchRate();
  float const yawRate = kinematicState.yawRate();

  double const deltaRoll = rollRate * dt;
  double const deltaPitch = pitchRate * dt;
  double const deltaYaw = yawRate * dt;

  Eigen::AngleAxisd const deltaRollAngle(deltaRoll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd const deltaPitchAngle(deltaPitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd const deltaYawAngle(deltaYaw, Eigen::Vector3d::UnitZ());
  Eigen::Quaternion<double> const deltaQ = deltaRollAngle * deltaPitchAngle 
    * deltaYawAngle;

  Eigen::AngleAxisd const rollAngle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd const pitchAngle(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd const yawAngle(yaw, Eigen::Vector3d::UnitZ());
  Eigen::Quaternion<double> const q = rollAngle * pitchAngle * yawAngle;

  Eigen::Quaternion<double> const newQ = deltaQ * q;

  Eigen::Vector3d const newEuler = newQ.toRotationMatrix().eulerAngles(0, 1, 2);


  float const newX = static_cast<float>(x + vx * dt);
  float const newY = static_cast<float>(y + vy * dt);
  float const newZ = static_cast<float>(z + vz * dt);

  float const newRoll = static_cast<float>(newEuler[0]);
  float const newPitch = static_cast<float>(newEuler[1]);
  float const newYaw = static_cast<float>(newEuler[2]);

  opendlv::sim::Frame frame;
  frame.x(newX);
  frame.y(newY);
  frame.z(newZ);
  frame.roll(newRoll);
  frame.pitch(newPitch);
  frame.yaw(newYaw);
  
  m_frame = frame;

  return frame;
}
