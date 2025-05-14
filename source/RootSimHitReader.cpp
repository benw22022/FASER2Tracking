// This file is part of the ACTS project.
//
// Copyright (C) 2016 CERN for the benefit of the ACTS project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "../include/RootSimHitReader.h"

#include "Acts/Definitions/PdgParticle.hpp"
#include "Acts/Utilities/Logger.hpp"
#include "ActsExamples/EventData/SimParticle.hpp"
#include "ActsExamples/Framework/AlgorithmContext.hpp"
#include "ActsFatras/EventData/ProcessType.hpp"

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <stdexcept>

#include <TChain.h>
#include <TMathBase.h>

RootSimHitReader::RootSimHitReader(const RootSimHitReader::Config& config,
                                   Acts::Logging::Level level)
    : IReader(),
      m_cfg(config),
      m_logger(Acts::getDefaultLogger(name(), level)) 
  {
    initialise();
  }

RootSimHitReader::RootSimHitReader(Acts::Logging::Level level, std::string treeName, std::string filePath, int axisDirection)
  : IReader(),
    m_logger(Acts::getDefaultLogger(name(), level))
  {
    m_cfg.filePath = filePath;
    m_cfg.treeName = treeName;
    m_cfg.axisDirection = axisDirection;
    initialise();
  }

void RootSimHitReader::initialise()
{
  m_inputChain = std::make_unique<TChain>(m_cfg.treeName.c_str());

  if (m_cfg.filePath.empty()) {
    throw std::invalid_argument("Missing input filename");
  }
  if (m_cfg.treeName.empty()) {
    throw std::invalid_argument("Missing tree name");
  }

  m_outputSimHits.initialize(m_cfg.outputSimHits);

  // Set the branches
  int f = 0;
  auto setBranches = [&](const auto& keys, auto& columns) {
    for (auto key : keys) {
      columns.insert({key, f++});
    }
    for (auto key : keys) {
      m_inputChain->SetBranchAddress(key, &columns.at(key));
    }
  };

  setBranches(m_floatKeys, m_floatColumns);
  setBranches(m_uint32Keys, m_uint32Columns);
  setBranches(m_uint64Keys, m_uint64Columns);
  setBranches(m_int32Keys, m_int32Columns);

  // add file to the input chain
  m_inputChain->Add(m_cfg.filePath.c_str());
  m_inputChain->LoadTree(0);
  ACTS_DEBUG("Adding File " << m_cfg.filePath << " to tree '" << m_cfg.treeName
                            << "'.");

  // Because each hit is stored in a single entry in the root file, we need to
  // scan the file first for the positions of the events in the file in order to
  // efficiently read the events later on.
  // TODO change the file format to store one event per entry

  // Disable all branches and only enable event-id for a first scan of the file
  m_inputChain->SetBranchStatus("*", false);
  m_inputChain->SetBranchStatus("event_id", true);

  auto nEntries = static_cast<std::size_t>(m_inputChain->GetEntriesFast());
  if (nEntries == 0) {
    throw std::runtime_error("Did not find any entries in input file");
  }

  // Add the first entry
  m_inputChain->GetEntry(0);
  m_eventMap.push_back({m_uint32Columns.at("event_id"), 0ul, 0ul});

  // Go through all entries and store the position of new events
  for (auto i = 1ul; i < nEntries; ++i) {
    m_inputChain->GetEntry(i);
    const auto evtId = m_uint32Columns.at("event_id");

    if (evtId != std::get<0>(m_eventMap.back())) {
      std::get<2>(m_eventMap.back()) = i;
      m_eventMap.push_back({evtId, i, i});
    }
  }

  std::get<2>(m_eventMap.back()) = nEntries;

  // Sort by event id
  std::ranges::sort(m_eventMap, {},
                    [](const auto& m) { return std::get<0>(m); });

  // Re-Enable all branches
  m_inputChain->SetBranchStatus("*", true);
  ACTS_DEBUG("Event range: " << availableEvents().first << " - "
                             << availableEvents().second);

  // Set the axis direction
  m_axisDirection = static_cast<Acts::AxisDirection>(m_cfg.axisDirection);
}

RootSimHitReader::~RootSimHitReader() = default;

std::pair<std::size_t, std::size_t> RootSimHitReader::availableEvents() const {
  return {std::get<0>(m_eventMap.front()), std::get<0>(m_eventMap.back()) + 1};
}

ActsExamples::ProcessCode RootSimHitReader::read(const ActsExamples::AlgorithmContext& context) {
  auto it = std::ranges::find_if(m_eventMap, [&](const auto& a) {
    return std::get<0>(a) == context.eventNumber;
  });

  if (it == m_eventMap.end()) {
    // explicitly warn if it happens for the first or last event as that might
    // indicate a human error
    if ((context.eventNumber == availableEvents().first) &&
        (context.eventNumber == availableEvents().second - 1)) {
      ACTS_WARNING("Reading empty event: " << context.eventNumber);
    } else {
      ACTS_DEBUG("Reading empty event: " << context.eventNumber);
    }

    m_outputSimHits(context, {});

    // Return success flag
    return ActsExamples::ProcessCode::SUCCESS;
  }

  // lock the mutex
  std::lock_guard<std::mutex> lock(m_read_mutex);

  ACTS_DEBUG("Reading event: " << std::get<0>(*it)
                               << " stored in entries: " << std::get<1>(*it)
                               << " - " << std::get<2>(*it));

  ActsExamples::SimHitContainer hits;
  for (auto entry = std::get<1>(*it); entry < std::get<2>(*it); ++entry) {
    m_inputChain->GetEntry(entry);

    auto eventId = m_uint32Columns.at("event_id");
    if (eventId != context.eventNumber) {
      break;
    }

    const Acts::GeometryIdentifier geoid = m_uint64Columns.at("geometry_id");
    const ActsExamples::SimBarcode pid = m_uint64Columns.at("particle_id");
    const auto index = m_int32Columns.at("index");


    Acts::RotationMatrix3 rotation = Acts::RotationMatrix3::Identity();
    if (m_axisDirection == Acts::AxisDirection::AxisX) {
      rotation.col(0) = Acts::Vector3(0, 0, -1);
      rotation.col(1) = Acts::Vector3(0, 1, 0);
      rotation.col(2) = Acts::Vector3(1, 0, 0);
    } else if (m_axisDirection == Acts::AxisDirection::AxisY) {
      rotation.col(0) = Acts::Vector3(1, 0, 0);
      rotation.col(1) = Acts::Vector3(0, 0, -1);
      rotation.col(2) = Acts::Vector3(0, 1, 0);
    }

    Acts::Vector3 pos3 = {
        m_floatColumns.at("tx") * Acts::UnitConstants::mm,
        m_floatColumns.at("ty") * Acts::UnitConstants::mm,
        m_floatColumns.at("tz") * Acts::UnitConstants::mm,
    };
    pos3 = rotation * pos3;

    Acts::Vector3 before3 = {
        m_floatColumns.at("tpx") * Acts::UnitConstants::GeV,
        m_floatColumns.at("tpy") * Acts::UnitConstants::GeV,
        m_floatColumns.at("tpz") * Acts::UnitConstants::GeV,
    };
    before3 = rotation * before3;

    Acts::Vector3 delta3 = {
        m_floatColumns.at("deltapx") * Acts::UnitConstants::GeV,
        m_floatColumns.at("deltapy") * Acts::UnitConstants::GeV,
        m_floatColumns.at("deltapz") * Acts::UnitConstants::GeV,
    };
    delta3 = rotation * delta3;

    const Acts::Vector4 pos4 = {
        pos3.x() * Acts::UnitConstants::mm,
        pos3.y() * Acts::UnitConstants::mm,
        pos3.z() * Acts::UnitConstants::mm,
        m_floatColumns.at("tt") * Acts::UnitConstants::mm,
    };

    const Acts::Vector4 before4 = {
        before3.x() * Acts::UnitConstants::GeV,
        before3.y() * Acts::UnitConstants::GeV,
        before3.z() * Acts::UnitConstants::GeV,
        m_floatColumns.at("te") * Acts::UnitConstants::GeV,
    };

    const Acts::Vector4 delta = {
        delta3.x() * Acts::UnitConstants::GeV,
        delta3.y() * Acts::UnitConstants::GeV,
        delta3.z() * Acts::UnitConstants::GeV,
        m_floatColumns.at("deltae") * Acts::UnitConstants::GeV,
    };

    ActsExamples::SimHit hit(geoid, pid, pos4, before4, before4 + delta, index);

    hits.insert(hit);
  }

  ACTS_DEBUG("Read " << hits.size() << " hits for event "
                     << context.eventNumber);

  m_outputSimHits(context, std::move(hits));

  // Return success flag
  return ActsExamples::ProcessCode::SUCCESS;
}
