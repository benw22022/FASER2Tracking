// This file is part of the ACTS project.
//
// Copyright (C) 2016 CERN for the benefit of the ACTS project
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "../include/RootParticleReader.hpp"

#include <Acts/Definitions/PdgParticle.hpp>
#include <Acts/Utilities/Logger.hpp>
#include <ActsExamples/EventData/SimParticle.hpp>
#include <ActsExamples/Framework/AlgorithmContext.hpp>
#include <ActsExamples/Io/Root/RootUtility.hpp>
#include <ActsFatras/EventData/ParticleOutcome.hpp>
#include <ActsFatras/EventData/ProcessType.hpp>

#include <iostream>
#include <stdexcept>

#include <TChain.h>

RootParticleReader::RootParticleReader(const RootParticleReader::Config& config,
                                       Acts::Logging::Level level)
    : IReader(),
      m_cfg(config),
      m_logger(Acts::getDefaultLogger(name(), level)) {
        initialise();
}

RootParticleReader::RootParticleReader(std::string filePath, Acts::Logging::Level level,  int axisDirection,
                                       std::string outputParticles, std::string treeName)
  : IReader(),
    m_logger(Acts::getDefaultLogger(name(), level))
  {
    m_cfg.outputParticles = outputParticles;
    m_cfg.filePath = filePath;
    m_cfg.treeName = treeName;
    m_cfg.axisDirection = axisDirection;
    initialise();
  }


void RootParticleReader::initialise()
{
  // Set the axis direction
  m_axisDirection = static_cast<Acts::AxisDirection>(m_cfg.axisDirection);

  m_inputChain = std::make_unique<TChain>(m_cfg.treeName.c_str());

  if (m_cfg.filePath.empty()) {
    throw std::invalid_argument("Missing input filename");
  }
  if (m_cfg.treeName.empty()) {
    throw std::invalid_argument("Missing tree name");
  }

  m_outputParticles.initialize(m_cfg.outputParticles);

  // Set the branches
  m_inputChain->SetBranchAddress("event_id", &m_eventId);
  m_inputChain->SetBranchAddress("particle_id", &m_particleId);
  m_inputChain->SetBranchAddress("particle_type", &m_particleType);
  m_inputChain->SetBranchAddress("process", &m_process);
  m_inputChain->SetBranchAddress("vx", &m_vx);
  m_inputChain->SetBranchAddress("vy", &m_vy);
  m_inputChain->SetBranchAddress("vz", &m_vz);
  m_inputChain->SetBranchAddress("vt", &m_vt);
  m_inputChain->SetBranchAddress("p", &m_p);
  m_inputChain->SetBranchAddress("px", &m_px);
  m_inputChain->SetBranchAddress("py", &m_py);
  m_inputChain->SetBranchAddress("pz", &m_pz);
  m_inputChain->SetBranchAddress("m", &m_m);
  m_inputChain->SetBranchAddress("q", &m_q);
  m_inputChain->SetBranchAddress("eta", &m_eta);
  m_inputChain->SetBranchAddress("phi", &m_phi);
  m_inputChain->SetBranchAddress("pt", &m_pt);
  m_inputChain->SetBranchAddress("vertex_primary", &m_vertexPrimary);
  m_inputChain->SetBranchAddress("vertex_secondary", &m_vertexSecondary);
  m_inputChain->SetBranchAddress("particle", &m_particle);
  m_inputChain->SetBranchAddress("generation", &m_generation);
  m_inputChain->SetBranchAddress("sub_particle", &m_subParticle);

  m_inputChain->SetBranchAddress("e_loss", &m_eLoss);
  m_inputChain->SetBranchAddress("total_x0", &m_pathInX0);
  m_inputChain->SetBranchAddress("total_l0", &m_pathInL0);
  m_inputChain->SetBranchAddress("number_of_hits", &m_numberOfHits);
  m_inputChain->SetBranchAddress("outcome", &m_outcome);

  auto path = m_cfg.filePath;

  // add file to the input chain
  m_inputChain->Add(path.c_str());
  ACTS_DEBUG("Adding File " << path << " to tree '" << m_cfg.treeName << "'.");

  m_events = m_inputChain->GetEntries();
  ACTS_DEBUG("The full chain has " << m_events << " entries.");

  // Sort the entry numbers of the events
  {
    // necessary to guarantee that m_inputChain->GetV1() is valid for the
    // entire range
    m_inputChain->SetEstimate(m_events + 1);

    m_entryNumbers.resize(m_events);
    m_inputChain->Draw("event_id", "", "goff");
    ActsExamples::RootUtility::stableSort(m_inputChain->GetEntries(), m_inputChain->GetV1(),
                            m_entryNumbers.data(), false);
  }
}

std::pair<std::size_t, std::size_t> RootParticleReader::availableEvents()
    const {
  return {0u, m_events};
}

RootParticleReader::~RootParticleReader() {
  delete m_particleId;
  delete m_particleType;
  delete m_process;
  delete m_vx;
  delete m_vy;
  delete m_vz;
  delete m_vt;
  delete m_p;
  delete m_px;
  delete m_py;
  delete m_pz;
  delete m_m;
  delete m_q;
  delete m_eta;
  delete m_phi;
  delete m_pt;
  delete m_vertexPrimary;
  delete m_vertexSecondary;
  delete m_particle;
  delete m_generation;
  delete m_subParticle;

  delete m_eLoss;
  delete m_pathInX0;
  delete m_pathInL0;
  delete m_numberOfHits;
  delete m_outcome;
}

ActsExamples::ProcessCode RootParticleReader::read(const ActsExamples::AlgorithmContext& context) {
  ACTS_DEBUG("Trying to read recorded particles.");

  if (m_inputChain == nullptr || context.eventNumber >= m_events) {
    return ActsExamples::ProcessCode::SUCCESS;
  }

  // lock the mutex
  std::lock_guard<std::mutex> lock(m_read_mutex);
  // now read

  // The particle collection to be filled
  ActsExamples::SimParticleContainer particles;

  // Read the correct entry
  auto entry = m_entryNumbers.at(context.eventNumber);
  m_inputChain->GetEntry(entry);
  ACTS_DEBUG("Reading event: " << context.eventNumber
                               << " stored as entry: " << entry);

  unsigned int nParticles = m_particleId->size();

  for (unsigned int i = 0; i < nParticles; i++) {
    ActsExamples::SimParticle p;

    p.setProcess(static_cast<ActsFatras::ProcessType>((*m_process).at(i)));
    p.setPdg(static_cast<Acts::PdgParticle>((*m_particleType).at(i)));
    p.setCharge((*m_q).at(i) * Acts::UnitConstants::e);
    p.setMass((*m_m).at(i) * Acts::UnitConstants::GeV);
    // auto particleId = ActsFatras::Barcode();
    // particleId.setVertexPrimary(0);
    // particleId.setVertexPrimary(1);
    // particleId.setGeneration(0);
    // particleId.setSubParticle(0);
    // particleId.setParticle(i);
    p.setParticleId((*m_particleId).at(i));

    ActsExamples::SimParticleState& initialState = p.initial();

    // Now rotate the particles to match the detector axis
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
      (*m_vx).at(i) * Acts::UnitConstants::mm,
      (*m_vy).at(i) * Acts::UnitConstants::mm,
      (*m_vz).at(i) * Acts::UnitConstants::mm,
    };

    std::cout << std::endl;
    std::cout << "Before: Vertex is at " << pos3.x() << " " << pos3.y() << " " << pos3.z() << std::endl;
    std::cout << "Applying offset of " << m_cfg.offset.x() << " "<< m_cfg.offset.y() << " "<< m_cfg.offset.z() << " to vertex" << std::endl;
    pos3 = pos3 + m_cfg.offset;
    std::cout << "After: Vertex is at " << pos3.x() << " " << pos3.y() << " " << pos3.z() << std::endl;
    std::cout << std::endl;

    pos3 = rotation * pos3;

    initialState.setPosition4(pos3.x(),
                              pos3.y(),
                              pos3.z(),
                              (*m_vt).at(i) * Acts::UnitConstants::mm);

    // NOTE: direction is normalized inside `setDirection`
    Acts::Vector3 mom3 = {
      (*m_px).at(i) * Acts::UnitConstants::GeV,
      (*m_py).at(i) * Acts::UnitConstants::GeV,
      (*m_pz).at(i) * Acts::UnitConstants::GeV,
    };
    mom3 = rotation * mom3;

    initialState.setDirection(mom3.x(), mom3.y(), mom3.z());
    initialState.setAbsoluteMomentum((*m_p).at(i) * Acts::UnitConstants::GeV);

    ActsExamples::SimParticleState& finalState = p.final();

    // TODO eloss cannot be read since we need the final momentum
    finalState.setMaterialPassed((*m_pathInX0).at(i) * Acts::UnitConstants::mm,
                                 (*m_pathInL0).at(i) * Acts::UnitConstants::mm);
    finalState.setNumberOfHits((*m_numberOfHits).at(i));
    finalState.setOutcome(
        static_cast<ActsFatras::ParticleOutcome>((*m_outcome).at(i)));
    
    particles.insert(p);
  }
  
  ACTS_DEBUG("Read " << particles.size() << " particles for event "
                     << context.eventNumber);

  // Write the collections to the EventStore
  m_outputParticles(context, std::move(particles));

  // Return success flag
  return ActsExamples::ProcessCode::SUCCESS;
}

