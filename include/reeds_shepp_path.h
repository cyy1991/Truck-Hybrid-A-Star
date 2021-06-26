/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/*
 * @file
 */

#pragma once

#include <omp.h>

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <cmath>



struct ReedSheppPath {
  std::vector<double> segs_lengths;
  std::vector<char> segs_types;
  double total_length = 0.0;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> phi;
  // true for driving forward and false for driving backward
  // std::vector<bool> gear;
  std::vector<double> gear; 
};

struct RSPParam {
  bool flag = false;
  double t = 0.0;
  double u = 0.0;
  double v = 0.0;
};

class ReedShepp {
 public:
  ReedShepp(double max_kappa);
  virtual ~ReedShepp() = default;
  // Pick the shortest path from all possible combination of movement primitives
  // by Reed Shepp
  bool ShortestRSP(double start_node[3],
                   double end_node[3], double step_size, 
                   ReedSheppPath* optimal_path);

 // protected:
  // Generate all possible combination of movement primitives by Reed Shepp and
  // interpolate them
  bool GenerateRSPs(double start_node[3],
                    double end_node[3],
                    std::vector<ReedSheppPath>* all_possible_paths);
  // Set the general profile of the movement primitives
  bool GenerateRSP(double start_node[3],
                  double end_node[3],
                   std::vector<ReedSheppPath>* all_possible_paths);
  // Set the general profile of the movement primitives, parallel implementation
  bool GenerateRSPPar(double start_node[3],
                      double end_node[3],
                      std::vector<ReedSheppPath>* all_possible_paths);
  // Set local exact configurations profile of each movement primitive
  bool GenerateLocalConfigurations(double start_node[3],
                                   double end_node[3], double step_size, 
                                   ReedSheppPath* shortest_path);
  // Interpolation usde in GenetateLocalConfiguration
  void Interpolation(const int index, const double pd, const char m,
                     const double ox, const double oy, const double ophi,
                     std::vector<double>* px, std::vector<double>* py,
                     std::vector<double>* pphi, std::vector<double>* pgear); 
                     // std::vector<bool>* pgear);
  // motion primitives combination setup function
  bool SetRSP(const int size, const double* lengths, const char* types,
              std::vector<ReedSheppPath>* all_possible_paths);
  // setRSP parallel version
  bool SetRSPPar(const int size, const double* lengths,
                 const std::string& types,
                 std::vector<ReedSheppPath>* all_possible_paths, const int idx);
  // Six different combination of motion primitive in Reed Shepp path used in
  // GenerateRSP()
  bool SCS(const double x, const double y, const double phi,
           std::vector<ReedSheppPath>* all_possible_paths);
  bool CSC(const double x, const double y, const double phi,
           std::vector<ReedSheppPath>* all_possible_paths);
  bool CCC(const double x, const double y, const double phi,
           std::vector<ReedSheppPath>* all_possible_paths);
  bool CCCC(const double x, const double y, const double phi,
            std::vector<ReedSheppPath>* all_possible_paths);
  bool CCSC(const double x, const double y, const double phi,
            std::vector<ReedSheppPath>* all_possible_paths);
  bool CCSCC(const double x, const double y, const double phi,
             std::vector<ReedSheppPath>* all_possible_paths);
  // different options for different combination of motion primitives
  void LSL(const double x, const double y, const double phi, RSPParam* param);
  void LSR(const double x, const double y, const double phi, RSPParam* param);
  void LRL(const double x, const double y, const double phi, RSPParam* param);
  void SLS(const double x, const double y, const double phi, RSPParam* param);
  void LRLRn(const double x, const double y, const double phi, RSPParam* param);
  void LRLRp(const double x, const double y, const double phi, RSPParam* param);
  void LRSR(const double x, const double y, const double phi, RSPParam* param);
  void LRSL(const double x, const double y, const double phi, RSPParam* param);
  void LRSLR(const double x, const double y, const double phi, RSPParam* param);
  std::pair<double, double> calc_tau_omega(const double u, const double v,
                                           const double xi, const double eta,
                                           const double phi);

 protected:

  double max_kappa_;
};
