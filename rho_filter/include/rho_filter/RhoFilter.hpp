// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef RHO_FILTER_HPP
#define RHO_FILTER_HPP

#include <Eigen/Dense>

class RhoFilter {
public:
    RhoFilter(int state_space_dim, double k1, double k2, double k3, double k4);

    void propagate_filter(const Eigen::VectorXd& current_position, double dt);

    Eigen::VectorXd get_position_estimate() const {
        return zeta.segment(0, state_space_dim);
    }

    Eigen::VectorXd get_velocity_estimate() const {
        return zeta.segment(state_space_dim, state_space_dim);
    }

private:
    int state_space_dim;
    double k4;

    Eigen::Matrix4d m;
    Eigen::Matrix4d m_inv;
    Eigen::Vector4d n;
    Eigen::Vector4d e;
    Eigen::Matrix4d I_4;

    Eigen::VectorXd zeta;
    Eigen::VectorXd next_zeta;
    Eigen::VectorXd sgn_term;
};

#endif