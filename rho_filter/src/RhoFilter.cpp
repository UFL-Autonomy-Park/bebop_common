// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <rho_filter/RhoFilter.hpp>
#include <unsupported/Eigen/MatrixFunctions>
#include <cmath>

RhoFilter::RhoFilter(
    int state_space_dim,
    double k1,
    double k2,
    double k3,
    double k4
) : 
    state_space_dim(state_space_dim),
    k4(k4)
{   
    I_4 = Eigen::Matrix4d::Identity();
    
    double m_12 = -(3 - std::pow(k1, 2) + (2*k1 + k2 + k3)*(k1 + k2));
    double m_13 = -(k1 + k2);
    double m_14 = -(1 + (k1 + k2)*(k3 - k1));
    double m_32 = -((2*k1 + k2 + k3)*(k1 + k2) + 1);
    double m_33 = -(2*k1 + k2);
    double m_42 = -(2*k1 + k2 + k3);
    double m_43 = -(k1 + k2)*(k3 - k1);

    m << 0,      1,      0,      0,
         m_12,   0,      m_32,   m_42,
         m_13,   0,      m_33,   -1,
         m_14,   0,      m_43,   -k3;

    m_inv = m.colPivHouseholderQr().inverse();

    double n_12 = 3 - std::pow(k1, 2) + (2*k1 + k2 + k3)*(k1 + k2);
    double n_13 = k1 + k2;
    double n_14 = 1 + (k1 + k2)*(k3 - k1);

    n << 0, n_12, n_13, n_14;
    e << 0, 1, 0, 0;

    int zeta_dim = 4 * state_space_dim;
    zeta = Eigen::VectorXd::Zero(zeta_dim);
    next_zeta = Eigen::VectorXd::Zero(zeta_dim);
    sgn_term = Eigen::VectorXd::Zero(state_space_dim);
}

void RhoFilter::propagate_filter(const Eigen::VectorXd& current_position, double dt) 
{
    Eigen::Matrix4d a_d = (m * dt).exp();
    Eigen::Vector4d b_q = m_inv * (a_d - I_4) * n;
    Eigen::Vector4d b_s = m_inv * (a_d - I_4) * e;

    auto z1 = zeta.segment(0, state_space_dim);
    auto z2 = zeta.segment(state_space_dim, state_space_dim);
    auto z3 = zeta.segment(2 * state_space_dim, state_space_dim);
    auto z4 = zeta.segment(3 * state_space_dim, state_space_dim);

    // S = [-1, 0, -1, 0] \otimes I_n
    sgn_term.noalias() = current_position - z1 - z3;
    Eigen::VectorXd sgn_vec = sgn_term.cwiseSign();

    for (int i = 0; i < 4; ++i) {
        auto next_zi = next_zeta.segment(i * state_space_dim, state_space_dim);
        
        next_zi.noalias() =  a_d(i, 0) * z1;
        next_zi.noalias() += a_d(i, 1) * z2;
        next_zi.noalias() += a_d(i, 2) * z3;
        next_zi.noalias() += a_d(i, 3) * z4;

        next_zi.noalias() += b_q(i) * current_position;
        next_zi.noalias() += (k4 * b_s(i)) * sgn_vec;
    }
    
    zeta = next_zeta;
}