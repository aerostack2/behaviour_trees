/*!*******************************************************************************************
 *  \file       gps_to_cartesian_service.cpp
 *  \brief      GPS to Cartesian service implementation as behaviour tree node
 *  \authors    Pedro Arias Pérez
 *              Miguel Fernández Cortizas
 *              David Pérez Saura
 *              Rafael Pérez Seguí
 *
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#include "behaviour_trees/action/gps_to_cartesian.hpp"

namespace as2_behaviour_tree
{
    GpsToCartesian::GpsToCartesian(const std::string &xml_tag_name, const BT::NodeConfiguration &conf)
        : nav2_behavior_tree::BtServiceNode<as2_msgs::srv::GeopathToPath>(xml_tag_name, conf)
    {
    }

    void GpsToCartesian::on_tick()
    {

        getInput("latitude", geopose.pose.position.latitude);
        getInput("longitude", geopose.pose.position.longitude);
        getInput("altitude", geopose.pose.position.altitude);
        geopath.poses.push_back(geopose);
        this->request_->geo_path = geopath;
        setOutput("out_pose", "hola");
    }

    BT::NodeStatus GpsToCartesian::on_completion(std::shared_ptr<as2_msgs::srv::GeopathToPath::Response> response)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = response->path.poses.at(0).pose.position.x;
        pose.position.y = response->path.poses.at(0).pose.position.y;
        pose.position.z = 2.0;
        
        return response->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
} // namespace as2_behaviour_tree