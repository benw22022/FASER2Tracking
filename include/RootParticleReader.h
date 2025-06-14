// This file is part of the ACTS project.
//
// Copyright (C) 2016 CERN for the benefit of the ACTS project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "ActsExamples/EventData/SimParticle.hpp"
#include "ActsExamples/Framework/DataHandle.hpp"
#include "ActsExamples/Framework/IReader.hpp"
#include "ActsExamples/Framework/ProcessCode.hpp"
#include <Acts/Definitions/Algebra.hpp>
#include <Acts/Propagator/MaterialInteractor.hpp>
#include <Acts/Utilities/Logger.hpp>
#include <Acts/Utilities/AxisDefinitions.hpp>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

class TChain;

/// @class RootParticleReader
///
/// @brief Reads in Particles information from a root file
class RootParticleReader : public ActsExamples::IReader {
 public:
  /// @brief The nested configuration struct
  struct Config {
    /// particle collection to read
    std::string outputParticles = "particleCollection";
    /// name of the output tree
    std::string treeName = "particles";
    /// The name of the input file
    std::string filePath;
    // Set the orientation of the detector axis
    int axisDirection = 2;
    // Set vertex offset
    Acts::Vector3 offset = {0,0,0};
    //// Map Geant4 particle ID to Acts Particle ID (FATRAS Barcode)
    //// Should really be a private member of the class but adding additional methods to the python binding is too hard
    //// Making it a member of the config I think is the easiest way to allow the user to read it, but it really shouldn't be modified by the user
    //// std::map<long unsigned int, long unsigned int> particleIdMap;
  };

  /// Constructor
  /// @param config The Configuration struct
  RootParticleReader(const Config& config, Acts::Logging::Level level);

  RootParticleReader(std::string filePath, Acts::Logging::Level level, int axisDirection=2, std::string outputParticles="particleCollection", 
    std::string treeName="particles");

  /// Destructor
  ~RootParticleReader() override;

  /// Framework name() method
  std::string name() const override { return "RootParticleReader"; }

  /// Return the available events range.
  std::pair<std::size_t, std::size_t> availableEvents() const override;

  /// Read out data from the input stream
  ///
  /// @param context The algorithm context
  ActsExamples::ProcessCode read(const ActsExamples::AlgorithmContext& context) override;

  /// Readonly access to the config
  const Config& config() const { return m_cfg; }

 private:
  /// Private access to the logging instance
  const Acts::Logger& logger() const { return *m_logger; }

  /// constructor helper function
  void initialise();

  /// The config class
  Config m_cfg;

  /// axis direction
  Acts::AxisDirection m_axisDirection;

  ActsExamples::WriteDataHandle<ActsExamples::SimParticleContainer> m_outputParticles{this,
                                                          "OutputParticles"};

  std::unique_ptr<const Acts::Logger> m_logger;

  /// mutex used to protect multi-threaded reads
  std::mutex m_read_mutex;

  /// The number of events
  std::size_t m_events = 0;

  /// The input tree name
  std::unique_ptr<TChain> m_inputChain;

  /// Event identifier.
  std::uint32_t m_eventId = 0;

  /// The entry numbers for accessing events in increased order (there could be
  /// multiple entries corresponding to one event number)
  std::vector<long long> m_entryNumbers = {};

  std::vector<std::uint64_t>* m_particleId = new std::vector<std::uint64_t>;
  std::vector<std::int32_t>* m_particleType = new std::vector<std::int32_t>;
  std::vector<std::uint32_t>* m_process = new std::vector<std::uint32_t>;
  std::vector<float>* m_vx = new std::vector<float>;
  std::vector<float>* m_vy = new std::vector<float>;
  std::vector<float>* m_vz = new std::vector<float>;
  std::vector<float>* m_vt = new std::vector<float>;
  std::vector<float>* m_px = new std::vector<float>;
  std::vector<float>* m_py = new std::vector<float>;
  std::vector<float>* m_pz = new std::vector<float>;
  std::vector<float>* m_m = new std::vector<float>;
  std::vector<float>* m_q = new std::vector<float>;
  std::vector<float>* m_eta = new std::vector<float>;
  std::vector<float>* m_phi = new std::vector<float>;
  std::vector<float>* m_pt = new std::vector<float>;
  std::vector<float>* m_p = new std::vector<float>;
  std::vector<std::uint32_t>* m_vertexPrimary = new std::vector<std::uint32_t>;
  std::vector<std::uint32_t>* m_vertexSecondary = new std::vector<std::uint32_t>;
  std::vector<std::uint32_t>* m_particle = new std::vector<std::uint32_t>;
  std::vector<std::uint32_t>* m_generation = new std::vector<std::uint32_t>;
  std::vector<std::uint32_t>* m_subParticle = new std::vector<std::uint32_t>;

  std::vector<float>* m_eLoss = new std::vector<float>;
  std::vector<float>* m_pathInX0 = new std::vector<float>;
  std::vector<float>* m_pathInL0 = new std::vector<float>;
  std::vector<std::int32_t>* m_numberOfHits = new std::vector<std::int32_t>;
  std::vector<std::uint32_t>* m_outcome = new std::vector<std::uint32_t>;
};
